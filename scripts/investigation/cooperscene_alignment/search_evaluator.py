"""Offline BEV/XYZ frame search with unchanged CooperScene registration gates.

Target caches and isolated method bindings affect allocation/threading only. No
production module globals are modified. The reference is also the fusion prior;
reported agreement is therefore not independent pose accuracy.
"""
from dataclasses import asdict
from pathlib import Path
from time import perf_counter
from types import FunctionType, MethodType
import sys

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.investigation.cooperscene_alignment.evaluation_utils import (
    jsonable,
    map_proxy,
    registration_config,
)
from src.localization.enu_prior_fusion import EnuPriorFuser, EnuPriorFusionConfig
from src.localization.registration import (
    PointCloudRegistrar,
    _as_points,
    _as_transform,
    _estimate_normals,
    _make_cloud,
    _require_open3d,
)


def _isolated_method(function, instance, **overrides):
    """Reuse the original implementation with instance-local dependencies."""
    namespace = dict(function.__globals__)
    namespace.update(overrides)
    copied = FunctionType(
        function.__code__,
        namespace,
        function.__name__,
        function.__defaults__,
        function.__closure__,
    )
    copied.__kwdefaults__ = function.__kwdefaults__
    return MethodType(copied, instance)


class _TargetCache:
    def __init__(self, points, config):
        self.points = np.ascontiguousarray(_as_points(points, "map_points")).copy()
        self.points.flags.writeable = False
        self.tree = cKDTree(self.points)
        self.levels = {}
        self.o3d = _require_open3d()
        # Open3D's Vector3dVector conversion requires a writable array even
        # though it copies the coordinates. Keep the shared NumPy target fixed.
        cloud = _make_cloud(self.o3d, self.points.copy())
        for size in config.refinement_voxel_sizes:
            target = cloud.voxel_down_sample(size)
            if len(target.points) < 3:
                raise ValueError("target downsampling left fewer than three points")
            _estimate_normals(self.o3d, target, size, config.normal_radius_factor)
            self.levels[size] = target

    def voxel_down_sample(self, size):
        return self.levels[size]

    def query(self, *args, **kwargs):
        kwargs["workers"] = 2
        return self.tree.query(*args, **kwargs)

    @property
    def data(self):
        return self.tree.data

    def constructor(self, points):
        if points is not self.points:
            raise ValueError("cached fuser must use its original immutable target")
        return self

    def normals_already_cached(self, o3d, cloud, size, factor):
        if cloud is not self.levels[size]:
            raise ValueError("unexpected target in cached normal estimation")

    def residuals(self, o3d, source, target, transform):
        if target is not self.points:
            raise ValueError("cached registrar must use its original target")
        world = source @ transform[:3, :3].T + transform[:3, 3]
        return self.query(world)[0]


class _CachedRegistrar(PointCloudRegistrar):
    def __init__(self, target, config):
        super().__init__(config)
        self.target = target
        # Original ICP and gate implementations remain intact. Only immutable
        # target allocation, normal rebuilding and NN lookup are substituted.
        self._refine = _isolated_method(
            PointCloudRegistrar._refine,
            self,
            _estimate_normals=target.normals_already_cached,
        )
        self._evaluate = _isolated_method(
            PointCloudRegistrar._evaluate,
            self,
            _nearest_neighbor_residuals=target.residuals,
        )

    def register(self, source_points, map_points, initial_map_T_source=None):
        if map_points is not self.target.points:
            raise ValueError("unexpected map target")
        source = _as_points(source_points, "source_points")
        initial = _as_transform(initial_map_T_source)
        if initial is None:
            raise ValueError("offline search requires an explicit frame initialization")
        started = perf_counter()
        source_cloud = _make_cloud(self.target.o3d, source)
        transform = self._refine(self.target.o3d, source_cloud, self.target, initial)
        return self._evaluate(
            self.target.o3d,
            source,
            self.target.points,
            transform,
            initial_map_T_source=initial,
            coarse_inlier_ratio=None,
            started=started,
        )


class SearchEvaluator:
    """Reuse one fixed full-resolution map while evaluating independent frames.

    ``evaluate(frame)`` expects ``points_lidar``, ``source_T_lidar``,
    ``initial_map_T_source`` and ``reference_map_T_source`` attributes. Points
    must already have the desired finite/dynamic-annotation mask; no additional
    voxel/range mask is introduced. Construct from a map path (opacity>0,
    stride1) or from an array produced with that same map-proxy policy.
    """

    def __init__(self, map_path=None, *, target_points=None, skip_certified=True):
        if (map_path is None) == (target_points is None):
            raise ValueError("supply exactly one of map_path or target_points")
        config = registration_config("cooperscene")
        points = (
            map_proxy(map_path, min_opacity=0.0, stride=1)
            if target_points is None
            else target_points
        )
        self.target = _TargetCache(points, config)
        self.registrar = _CachedRegistrar(self.target, config)
        self.fusers = {}
        for mode in ("bev", "xyz"):
            fuser = EnuPriorFuser(EnuPriorFusionConfig(position_mode=mode))
            fuser.fuse = _isolated_method(
                EnuPriorFuser.fuse, fuser, cKDTree=self.target.constructor
            )
            self.fusers[mode] = fuser
        self.skip_certified = bool(skip_certified)

    @staticmethod
    def _pose(value, name):
        pose = _as_transform(value)
        if (
            pose is None
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
            or np.linalg.det(pose[:3, :3]) < 0.999
        ):
            raise ValueError(f"{name} must contain a proper rigid rotation")
        return pose

    @staticmethod
    def _error(transform, reference, source_pose):
        estimated = transform @ source_pose
        expected = reference @ source_pose
        delta = estimated[:3, 3] - expected[:3, 3]
        rotation = estimated[:3, :3] @ expected[:3, :3].T
        rpy = Rotation.from_matrix(rotation).as_euler("xyz", degrees=True)
        source_delta = reference[:3, :3].T @ delta
        source_rotation = reference[:3, :3].T @ rotation @ reference[:3, :3]
        source_rpy = Rotation.from_matrix(source_rotation).as_euler("xyz", degrees=True)
        return dict(
            dx_m=float(delta[0]),
            dy_m=float(delta[1]),
            dz_m=float(delta[2]),
            xy_m=float(np.linalg.norm(delta[:2])),
            xyz_m=float(np.linalg.norm(delta)),
            roll_deg=float(rpy[0]),
            pitch_deg=float(rpy[1]),
            yaw_deg=float(rpy[2]),
            orientation_deg=float(
                np.degrees(Rotation.from_matrix(rotation).magnitude())
            ),
            error_axes="map_xyz",
            independent_ground_truth=False,
            reference_also_fusion_prior=True,
            estimated_map_T_lidar=estimated,
            reference_map_T_lidar=expected,
            source_enu_error=dict(
                dx_m=float(source_delta[0]),
                dy_m=float(source_delta[1]),
                dz_m=float(source_delta[2]),
                xy_m=float(np.linalg.norm(source_delta[:2])),
                xyz_m=float(np.linalg.norm(source_delta)),
                roll_deg=float(source_rpy[0]),
                pitch_deg=float(source_rpy[1]),
                yaw_deg=float(source_rpy[2]),
                error_axes="source_enu",
            ),
        )

    def _evaluated(
        self, source, transform, initial, reference, source_pose, upstream=True
    ):
        registration = self.registrar.evaluate(
            source, self.target.points, transform, initial
        )
        return dict(
            map_T_source=transform,
            registration=asdict(registration),
            error=self._error(transform, reference, source_pose),
            accepted=bool(upstream and registration.accepted),
        )

    def _certificate(self, points):
        # Same necessary bounds as advance_cooperscene_frames.py, without its
        # module-level environment changes and unrelated MCAP imports.
        config = self.registrar.config
        distances = self.target.query(points)[0]
        displacement = config.max_correction_translation + 2 * np.sin(
            np.radians(config.max_correction_rotation_degrees) / 2
        ) * np.linalg.norm(points, axis=1)
        lower = np.maximum(0, distances - displacement - 1e-9)
        median, p95 = np.percentile(lower, [50, 95])
        radius = config.icp_distance_factor * config.refinement_voxel_sizes[-1]
        ratio = float(np.mean(lower <= radius))
        reasons = []
        if median > config.max_all_point_median_residual:
            reasons.append("all_point_median_residual")
        if p95 > config.max_all_point_p95_residual:
            reasons.append("all_point_p95_residual")
        if ratio < config.min_inlier_ratio:
            reasons.append("inlier_ratio")
        return dict(
            certified_impossible=bool(reasons),
            rejection_reasons=reasons,
            full_retained_point_count=len(lower),
            residual_median_lower_bound_m=float(median),
            residual_p95_lower_bound_m=float(p95),
            inlier_ratio_upper_bound=ratio,
            max_correction_translation_m=config.max_correction_translation,
            max_correction_rotation_deg=config.max_correction_rotation_degrees,
            inlier_radius_m=radius,
            numerical_margin_m=1e-9,
            mathematical_basis="distance-to-map Lipschitz bound with original map-frame relative-correction gates",
        )

    @staticmethod
    def _failure(reason, exception=None):
        return dict(
            accepted=False,
            error={},
            status=reason,
            reason=reason,
            exception=None
            if exception is None
            else f"{type(exception).__name__}: {exception}",
        )

    def evaluate(self, frame):
        started = perf_counter()
        points = _as_points(frame.points_lidar, "points_lidar")
        if len(points) < 3:
            raise ValueError("at least three source points are required")
        source_pose = self._pose(frame.source_T_lidar, "source_T_lidar")
        initial = self._pose(frame.initial_map_T_source, "initial_map_T_source")
        reference = self._pose(frame.reference_map_T_source, "reference_map_T_source")
        source = points @ source_pose[:3, :3].T + source_pose[:3, 3]
        record = dict(
            initial=self._evaluated(source, initial, initial, reference, source_pose),
            shared_scan=None,
            methods={},
            independent_frame_initialization=True,
            no_warm_start=True,
            original_gates_unchanged=True,
            reference_also_fusion_prior=True,
            independent_ground_truth=False,
            registration_config=asdict(self.registrar.config),
            fusion_configs={k: asdict(v.config) for k, v in self.fusers.items()},
            target_policy="map_proxy min_opacity0 stride1",
            source_points=len(source),
        )
        certificate = self._certificate(source @ initial[:3, :3].T + initial[:3, 3])
        record["rejection_certificate"] = certificate
        if self.skip_certified and certificate["certified_impossible"]:
            record["optimized_error_available"] = False
            record["methods"] = {
                mode: self._failure("certified_no_accepted_transform")
                for mode in self.fusers
            }
            record["runtime_seconds"] = perf_counter() - started
            return jsonable(record)
        try:
            scan = self.registrar.register(source, self.target.points, initial)
            checked = self._evaluated(
                source,
                scan.map_T_source,
                initial,
                reference,
                source_pose,
                scan.accepted,
            )
            scan_accepted = checked["accepted"]
            record["shared_scan"] = dict(
                asdict(scan),
                accepted=scan_accepted,
                rechecked=checked["registration"],
                error=checked["error"],
            )
            record["optimized_error_available"] = True
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
            record["methods"] = {
                mode: self._failure("shared_scan_optimizer_failed", error)
                for mode in self.fusers
            }
            record["optimized_error_available"] = False
            record["runtime_seconds"] = perf_counter() - started
            return jsonable(record)
        for mode, fuser in self.fusers.items():
            try:
                fusion = None
                transform = scan.map_T_source
                if mode == "bev" or scan_accepted:
                    fusion = fuser.fuse(
                        source, self.target.points, transform, reference, source_pose
                    )
                    transform = fusion.map_T_source
                converged = bool(fusion.converged) if fusion is not None else False
                result = self._evaluated(
                    source,
                    transform,
                    initial,
                    reference,
                    source_pose,
                    scan_accepted and converged,
                )
                result.update(
                    upstream_scan_accepted=scan_accepted,
                    fusion_attempted=fusion is not None,
                    fusion_converged=converged,
                    fusion=None if fusion is None else asdict(fusion),
                    status="accepted"
                    if result["accepted"]
                    else "upstream_or_original_registration_gates_rejected",
                )
                record["methods"][mode] = result
            except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
                record["methods"][mode] = self._failure(
                    mode + "_optimizer_failed", error
                )
        record["runtime_seconds"] = perf_counter() - started
        return jsonable(record)
