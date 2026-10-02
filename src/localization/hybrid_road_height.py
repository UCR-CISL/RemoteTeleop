"""Experimental one-axis road alignment conditional on a frozen BEV estimate.

No recorded/ENU target height enters this estimator. Existing registration and
fusion acceptance gates remain the caller's responsibility after this stage.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from src.localization.ground_plane import GroundPlane


@dataclass(frozen=True)
class HybridRoadHeightConfig:
    min_radius_m: float = 3.
    max_radius_m: float = 15.
    lidar_below_sensor_m: tuple[float, float] = (.3, 4.)
    lidar_distance_m: float = .08
    map_distance_m: float = .10
    lidar_min_inliers: int = 100
    map_min_inliers: int = 40
    lidar_min_ratio: float = .25
    map_min_ratio: float = .05
    max_tilt_deg: float = 12.
    max_normal_disagreement_deg: float = 5.
    min_span_m: float = 6.
    footprint_match_distance_m: float = 2.
    min_footprint_overlap: float = .35
    distinct_layer_separation_m: float = .30
    max_layers: int = 6
    voxel_m: float = .15
    max_fit_points: int = 20000


@dataclass
class HybridRoadHeightResult:
    accepted: bool
    reason: str
    map_T_lidar: np.ndarray
    candidate_map_T_lidar: np.ndarray | None = None
    diagnostics: dict = field(default_factory=dict)


class HybridRoadHeightEstimator:
    """Fit LiDAR ground and uniquely supported local map ground, then shift up.

    ``map_T_lidar`` and ``map_up`` must come from the frozen BEV estimate and
    existing orientation conventions. ``lidar_T_camera`` is the current frame's
    full CV-camera-to-LiDAR extrinsic. In the frozen gravity coordinates,
    road height at the ego footprint is ``h = -d / n_z``. The only update is
    ``t_new = t_bev + map_up * (h_map - h_lidar)``. Camera clearance uses
    ``d_camera = d_lidar + n_lidar @ t_lidar_camera``; CV camera Z is forward,
    so its translation Z alone is not the camera-ground clearance.

    Map points/normals are native GSplat
    geometry, never a reference-seeded height field. Optional ``dynamic_mask``
    marks source points to exclude; its provenance must be reported by callers.
    """

    def __init__(self, config: HybridRoadHeightConfig | None = None):
        self.config = config or HybridRoadHeightConfig()

    @staticmethod
    def _transform(value, name):
        value = np.asarray(value, dtype=float)
        if value.shape != (4, 4) or not np.isfinite(value).all() or not np.allclose(value[3], [0, 0, 0, 1]):
            raise ValueError(f"{name} must be a finite rigid 4x4 transform")
        if not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6) or np.linalg.det(value[:3, :3]) < .999:
            raise ValueError(f"{name} rotation must be right-handed orthonormal")
        return value.copy()

    def _balanced(self, points):
        if not len(points):
            return points
        _, indices = np.unique(np.floor(points / self.config.voxel_m).astype(np.int64), axis=0, return_index=True)
        points = points[indices]
        if len(points) > self.config.max_fit_points:
            points = points[np.random.default_rng(0).choice(len(points), self.config.max_fit_points, replace=False)]
        return points

    def _plane(self, points, *, lidar):
        c = self.config
        plane, metrics = GroundPlane.fit(points, up_hint=[0, 0, 1], max_tilt_deg=c.max_tilt_deg,
                                         distance_threshold_m=c.lidar_distance_m if lidar else c.map_distance_m,
                                         min_inliers=c.lidar_min_inliers if lidar else c.map_min_inliers,
                                         min_inlier_ratio=c.lidar_min_ratio if lidar else c.map_min_ratio,
                                         iterations=384)
        residuals = np.abs(points @ plane.normal + plane.offset_m)
        mask = residuals <= (c.lidar_distance_m if lidar else c.map_distance_m)
        support = points[mask]
        # Principal horizontal spans do not depend on arbitrary tangent axes.
        _, _, axes = np.linalg.svd(support[:, :2] - support[:, :2].mean(axis=0), full_matrices=False)
        spans = np.ptp(support[:, :2] @ axes.T, axis=0)
        metrics['principal_horizontal_spans_m'] = spans.tolist()
        metrics['extent_accepted'] = bool(np.min(spans) >= c.min_span_m)
        if lidar and not metrics['extent_accepted']:
            raise ValueError('plane support lacks two-dimensional road extent')
        return plane, metrics, mask

    def estimate(self, points_lidar, *, map_T_lidar, lidar_T_camera, map_up,
                 map_points, map_normals=None, dynamic_mask=None) -> HybridRoadHeightResult:
        c = self.config
        pose = self._transform(map_T_lidar, 'map_T_lidar')
        camera = self._transform(lidar_T_camera, 'lidar_T_camera')
        up = np.array(map_up, dtype=float, copy=True)
        if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
            raise ValueError('map_up must be a finite nonzero vector')
        up /= np.linalg.norm(up)
        tangent = np.eye(3)[np.argmin(np.abs(up))]
        tangent -= up * (tangent @ up); tangent /= np.linalg.norm(tangent)
        basis = np.column_stack((tangent, np.cross(up, tangent), up))
        source = np.asarray(points_lidar, dtype=float)
        target = np.asarray(map_points, dtype=float)
        if source.ndim != 2 or source.shape[1] < 3 or target.ndim != 2 or target.shape[1] != 3:
            raise ValueError('source must be Nx3/Nx4 and map points Mx3')
        source = source[:, :3]
        keep = np.isfinite(source).all(axis=1)
        if dynamic_mask is not None:
            mask = np.asarray(dynamic_mask)
            if mask.shape != (len(source),) or mask.dtype != bool:
                raise ValueError('dynamic_mask must be a bool per source point')
            keep &= ~mask
        diagnostics = dict(config=asdict(c), map_up=up.tolist(), dynamic_mask_used=dynamic_mask is not None,
                           source_input_count=len(source), source_after_dynamic_finite_count=int(keep.sum()),
                           independence='Conditional on frozen BEV pose, calibration and supplied dynamic mask; no target height input.')
        result = HybridRoadHeightResult(False, '', pose.copy(), diagnostics=diagnostics)
        # Local coordinates share the BEV LiDAR origin and gravity axis, without
        # assuming LiDAR X is forward (nuScenes forward is approximately +Y).
        local_source = source[keep] @ pose[:3, :3].T @ basis
        radius = np.linalg.norm(local_source[:, :2], axis=1)
        candidates = local_source[(radius >= c.min_radius_m) & (radius <= c.max_radius_m)
                                  & (local_source[:, 2] <= -c.lidar_below_sensor_m[0])
                                  & (local_source[:, 2] >= -c.lidar_below_sensor_m[1])]
        candidates = self._balanced(candidates)
        diagnostics['lidar_candidate_count'] = len(candidates)
        try:
            lidar_plane, lidar_fit, lidar_mask = self._plane(candidates, lidar=True)
        except ValueError as error:
            result.reason = 'insufficient_lidar_ground'; diagnostics['detail'] = str(error)
            return result
        n_lidar = np.asarray(lidar_plane.normal)
        source_height = -lidar_plane.offset_m / n_lidar[2]
        map_T_camera = pose @ camera
        camera_local = (map_T_camera[:3, 3] - pose[:3, 3]) @ basis
        camera_normal_clearance = float(n_lidar @ camera_local + lidar_plane.offset_m)
        diagnostics.update(lidar_plane_local=lidar_plane.to_dict(), lidar_fit=lidar_fit,
                           lidar_vertical_clearance_m=float(-source_height),
                           camera_normal_clearance_m=camera_normal_clearance,
                           camera_vertical_clearance_m=camera_normal_clearance / n_lidar[2],
                           camera_optical_forward_dot_ground_normal=float((basis @ n_lidar) @ map_T_camera[:3, 2]),
                           camera_down_dot_ground_normal=float((basis @ n_lidar) @ map_T_camera[:3, 1]))
        # No map-height band: an inherited BEV vertical error can be many metres.
        # Disambiguation uses horizontal footprint, extent and competing layers.
        map_keep = np.isfinite(target).all(axis=1)
        if map_normals is not None:
            normals = np.asarray(map_normals, dtype=float)
            if normals.shape != target.shape:
                raise ValueError('map_normals must match map_points')
            lengths = np.linalg.norm(normals, axis=1)
            map_keep &= np.isfinite(normals).all(axis=1) & (lengths > 1e-9)
            alignment = np.abs(normals @ up) / np.maximum(lengths, 1e-9)
            map_keep &= alignment >= np.cos(np.deg2rad(c.max_tilt_deg))
        local_map = (target[map_keep] - pose[:3, 3]) @ basis
        local_map = local_map[np.linalg.norm(local_map[:, :2], axis=1) <= c.max_radius_m]
        local_map = self._balanced(local_map)
        diagnostics['map_candidate_count'] = len(local_map)
        source_support = candidates[lidar_mask]
        # Equalize the overlap calculation across road area rather than beams.
        _, unique = np.unique(np.floor(source_support[:, :2]), axis=0, return_index=True)
        footprint = source_support[unique, :2]
        remaining = local_map.copy(); layers = []
        for _ in range(c.max_layers):
            try:
                map_plane, quality, inliers = self._plane(remaining, lidar=False)
            except ValueError:
                break
            support = remaining[inliers]
            overlap = float((cKDTree(support[:, :2]).query(footprint)[0] <= c.footprint_match_distance_m).mean())
            n_map = np.asarray(map_plane.normal)
            disagreement = float(np.rad2deg(np.arccos(np.clip(n_map @ n_lidar, -1, 1))))
            layer = dict(plane=map_plane.to_dict(), fit=quality, footprint_overlap=overlap,
                         normal_disagreement_deg=disagreement,
                         height_at_lidar_xy_m=-map_plane.offset_m / n_map[2])
            layer['plausible'] = quality['extent_accepted'] and overlap >= c.min_footprint_overlap and disagreement <= c.max_normal_disagreement_deg
            layers.append(layer)
            remaining = remaining[~inliers]
        diagnostics['map_layers'] = layers
        diagnostics['map_layer_search_truncated'] = False
        if len(layers) == c.max_layers:
            try:
                self._plane(remaining, lidar=False)
            except ValueError:
                pass
            else:
                diagnostics['map_layer_search_truncated'] = True
                result.reason = 'unresolved_map_ground_layers'
                return result
        plausible = [layer for layer in layers if layer['plausible']]
        if not plausible:
            result.reason = 'insufficient_or_incompatible_map_ground'
            return result
        heights = np.array([layer['height_at_lidar_xy_m'] for layer in plausible])
        if np.ptp(heights) > c.distinct_layer_separation_m:
            result.reason = 'ambiguous_map_ground_layers'
            return result
        best = max(plausible, key=lambda layer: layer['footprint_overlap'] * layer['fit']['inlier_count'])
        shift = float(best['height_at_lidar_xy_m'] - source_height)
        candidate = pose.copy(); candidate[:3, 3] += up * shift
        result.candidate_map_T_lidar = candidate.copy()
        result.map_T_lidar = candidate
        result.accepted = True; result.reason = 'road_height_candidate_requires_original_registration_gates'
        n_map = np.asarray(best['plane']['normal'])
        residual = np.abs((source_support + np.array([0, 0, shift])) @ n_map + best['plane']['offset_m'])
        diagnostics.update(selected_map_layer=best, height_shift_m=shift,
                           height_translation_map=(up * shift).tolist(),
                           fixed_rotation_error=float(np.max(np.abs(candidate[:3, :3] - pose[:3, :3]))),
                           transverse_translation_error_m=float(np.linalg.norm((np.eye(3) - np.outer(up, up)) @ (candidate[:3, 3] - pose[:3, 3]))),
                           up_normal_denominator=float(n_map[2]),
                           residual_height_scale_m=float((lidar_fit["median_residual_m"] + best["fit"]["median_residual_m"]) / abs(n_map[2])),
                           residual_height_scale_is_calibrated_uncertainty=False,
                           camera_map_normal_clearance_after_m=float(n_map @ (camera_local + np.array([0, 0, shift])) + best["plane"]["offset_m"]),
                           aligned_lidar_map_plane_median_m=float(np.median(residual)),
                           aligned_lidar_map_plane_p95_m=float(np.percentile(residual, 95)),
                           one_degree_tilt_height_sensitivity_at_roi_edge_m=float(c.max_radius_m * np.tan(np.deg2rad(1.))),
                           original_registration_gates_applied=False)
        return result
