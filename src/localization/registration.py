"""Rigid LiDAR-to-map point-cloud registration.

The public transform returned by this module is ``map_T_source``: it maps a
point expressed in the LiDAR/submap frame into the Gaussian-map frame.  This
module intentionally only consumes point coordinates so that map extraction
and CooperScene loading remain separate concerns.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class RegistrationConfig:
    """Parameters and conservative acceptance gates for SE(3) registration."""

    coarse_voxel_size: float = 1.0
    refinement_voxel_sizes: tuple[float, ...] = (1.0, 0.5, 0.25)
    normal_radius_factor: float = 2.0
    feature_radius_factor: float = 5.0
    ransac_distance_factor: float = 2.0
    ransac_n: int = 4
    ransac_max_iterations: int = 100_000
    ransac_confidence: float = 0.999
    random_seed: int | None = 0
    icp_distance_factor: float = 2.0
    icp_max_iterations: int = 40
    robust_kernel_scale_factor: float = 1.0
    min_inlier_ratio: float = 0.20
    max_median_residual: float = 0.35
    max_p95_residual: float = 0.80
    max_correction_translation: float = 3.0
    max_correction_rotation_degrees: float = 30.0


@dataclass(frozen=True)
class RegistrationResult:
    """Registration estimate plus diagnostics used to decide whether to fuse it."""

    map_T_source: NDArray[np.float64]
    inlier_ratio: float
    median_residual: float
    p90_residual: float
    p95_residual: float
    all_point_median_residual: float
    all_point_p90_residual: float
    all_point_p95_residual: float
    correction_translation: float
    correction_rotation_degrees: float
    runtime_seconds: float
    accepted: bool
    rejection_reasons: tuple[str, ...] = field(default_factory=tuple)
    coarse_inlier_ratio: float | None = None


class PointCloudRegistrar:
    """Registers a source LiDAR cloud to a static map using Open3D.

    With no initial transform, feature matching supplies a global candidate.
    Supplying ``initial_map_T_source`` instead starts robust ICP from that
    prediction, which is the intended online-tracking path.
    """

    def __init__(self, config: RegistrationConfig | None = None) -> None:
        self.config = config or RegistrationConfig()
        if self.config.coarse_voxel_size <= 0:
            raise ValueError("coarse_voxel_size must be positive")
        if not self.config.refinement_voxel_sizes or any(
            size <= 0 for size in self.config.refinement_voxel_sizes
        ):
            raise ValueError("refinement_voxel_sizes must contain only positive values")

    def register(
        self,
        source_points: NDArray[np.floating],
        map_points: NDArray[np.floating],
        initial_map_T_source: NDArray[np.floating] | None = None,
    ) -> RegistrationResult:
        """Estimate ``map_T_source`` from two ``(N, 3)`` numpy point arrays."""

        o3d = _require_open3d()
        source = _as_points(source_points, "source_points")
        target = _as_points(map_points, "map_points")
        initial = _as_transform(initial_map_T_source)
        started = perf_counter()

        source_cloud = _make_cloud(o3d, source)
        target_cloud = _make_cloud(o3d, target)

        coarse_inlier_ratio: float | None = None
        if initial is None:
            source_coarse = source_cloud.voxel_down_sample(self.config.coarse_voxel_size)
            target_coarse = target_cloud.voxel_down_sample(self.config.coarse_voxel_size)
            if len(source_coarse.points) < self.config.ransac_n or len(target_coarse.points) < self.config.ransac_n:
                raise ValueError("coarse voxel downsampling left too few points for global registration")
            _estimate_normals(
                o3d, source_coarse, self.config.coarse_voxel_size, self.config.normal_radius_factor
            )
            _estimate_normals(
                o3d, target_coarse, self.config.coarse_voxel_size, self.config.normal_radius_factor
            )
            initial, coarse_inlier_ratio = self._global_registration(o3d, source_coarse, target_coarse)
        map_T_source = self._refine(o3d, source_cloud, target_cloud, initial)
        residuals = _nearest_neighbor_residuals(o3d, source, target, map_T_source)
        final_threshold = self.config.icp_distance_factor * self.config.refinement_voxel_sizes[-1]
        inlier_mask = residuals <= final_threshold
        inlier_ratio = float(np.mean(inlier_mask))
        accepted_residuals = residuals[inlier_mask]
        if len(accepted_residuals):
            median_residual = float(np.median(accepted_residuals))
            p90_residual = float(np.percentile(accepted_residuals, 90))
            p95_residual = float(np.percentile(accepted_residuals, 95))
        else:
            median_residual = p90_residual = p95_residual = float("inf")
        all_point_median_residual = float(np.median(residuals))
        all_point_p90_residual = float(np.percentile(residuals, 90))
        all_point_p95_residual = float(np.percentile(residuals, 95))
        if initial_map_T_source is None:
            # A global alignment has no prediction to correct.  Its absolute map
            # transform can be arbitrarily far from identity and must not trip
            # online correction gates.
            correction_translation = 0.0
            correction_rotation = 0.0
        else:
            correction = map_T_source @ np.linalg.inv(initial)
            correction_translation = float(np.linalg.norm(correction[:3, 3]))
            correction_rotation = _rotation_degrees(correction[:3, :3])
        reasons = _acceptance_reasons(
            self.config,
            inlier_ratio,
            median_residual,
            p95_residual,
            correction_translation,
            correction_rotation,
        )
        return RegistrationResult(
            map_T_source=map_T_source,
            inlier_ratio=inlier_ratio,
            median_residual=median_residual,
            p90_residual=p90_residual,
            p95_residual=p95_residual,
            all_point_median_residual=all_point_median_residual,
            all_point_p90_residual=all_point_p90_residual,
            all_point_p95_residual=all_point_p95_residual,
            correction_translation=correction_translation,
            correction_rotation_degrees=correction_rotation,
            runtime_seconds=perf_counter() - started,
            accepted=not reasons,
            rejection_reasons=tuple(reasons),
            coarse_inlier_ratio=coarse_inlier_ratio,
        )

    def _global_registration(self, o3d, source, target) -> tuple[NDArray[np.float64], float]:
        if self.config.random_seed is not None:
            o3d.utility.random.seed(self.config.random_seed)
        feature_radius = self.config.feature_radius_factor * self.config.coarse_voxel_size
        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=100)
        )
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target, o3d.geometry.KDTreeSearchParamHybrid(radius=feature_radius, max_nn=100)
        )
        max_distance = self.config.ransac_distance_factor * self.config.coarse_voxel_size
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source,
            target,
            source_fpfh,
            target_fpfh,
            True,
            max_distance,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            self.config.ransac_n,
            [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(max_distance),
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(
                self.config.ransac_max_iterations, self.config.ransac_confidence
            ),
        )
        return np.asarray(result.transformation, dtype=np.float64), float(result.fitness)

    def _refine(self, o3d, source_cloud, target_cloud, initial: NDArray[np.float64]) -> NDArray[np.float64]:
        transform = initial.copy()
        for voxel_size in self.config.refinement_voxel_sizes:
            source = source_cloud.voxel_down_sample(voxel_size)
            target = target_cloud.voxel_down_sample(voxel_size)
            if len(source.points) < 3 or len(target.points) < 3:
                raise ValueError("refinement voxel downsampling left too few points for ICP")
            # Normals belong to the sampling scale, so rebuild them rather than
            # reusing coarse normals at a fine ICP level.
            _estimate_normals(o3d, target, voxel_size, self.config.normal_radius_factor)
            threshold = self.config.icp_distance_factor * voxel_size
            loss = o3d.pipelines.registration.TukeyLoss(
                self.config.robust_kernel_scale_factor * threshold
            )
            result = o3d.pipelines.registration.registration_icp(
                source,
                target,
                threshold,
                transform,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(loss),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.config.icp_max_iterations
                ),
            )
            transform = np.asarray(result.transformation, dtype=np.float64)
        return transform


def _require_open3d():
    try:
        import open3d as o3d
    except ImportError as error:  # pragma: no cover - depends on installation
        raise ImportError("Open3D is required; install the reconstruction dependency group") from error
    return o3d


def _as_points(points: NDArray[np.floating], name: str) -> NDArray[np.float64]:
    array = np.asarray(points, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 3 or len(array) == 0:
        raise ValueError(f"{name} must have shape (N, 3) with N > 0")
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains non-finite coordinates")
    return array


def _as_transform(transform: NDArray[np.floating] | None) -> NDArray[np.float64] | None:
    if transform is None:
        return None
    array = np.asarray(transform, dtype=np.float64)
    if array.shape != (4, 4) or not np.isfinite(array).all() or not np.allclose(array[3], [0, 0, 0, 1]):
        raise ValueError("initial_map_T_source must be a finite 4x4 homogeneous transform")
    return array


def _make_cloud(o3d, points: NDArray[np.float64]):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud


def _estimate_normals(o3d, cloud, voxel_size: float, radius_factor: float) -> None:
    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_factor * voxel_size, max_nn=30)
    )


def _nearest_neighbor_residuals(o3d, source, target, transform):
    transformed = source @ transform[:3, :3].T + transform[:3, 3]
    # KDTreeFlann retains a reference to its PointCloud rather than taking an
    # owning copy; keep the cloud alive for every nearest-neighbor query.
    target_cloud = _make_cloud(o3d, target)
    tree = o3d.geometry.KDTreeFlann(target_cloud)
    return np.fromiter(
        (np.sqrt(tree.search_knn_vector_3d(point, 1)[2][0]) for point in transformed),
        dtype=np.float64,
        count=len(transformed),
    )


def _rotation_degrees(rotation: NDArray[np.float64]) -> float:
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _acceptance_reasons(config, inlier_ratio, median, p95, translation, rotation) -> list[str]:
    reasons = []
    if inlier_ratio < config.min_inlier_ratio:
        reasons.append("inlier_ratio")
    if median > config.max_median_residual:
        reasons.append("median_residual")
    if p95 > config.max_p95_residual:
        reasons.append("p95_residual")
    if translation > config.max_correction_translation:
        reasons.append("correction_translation")
    if rotation > config.max_correction_rotation_degrees:
        reasons.append("correction_rotation")
    return reasons
