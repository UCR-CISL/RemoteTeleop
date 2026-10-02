"""Separate BEV plus LiDAR-clearance proposal; existing localization is unchanged."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace

import numpy as np

from src.localization.ground_plane import GroundPlane


@dataclass(frozen=True)
class BevLidarHeightConfig:
    """Explicit road-plane assumption and sensor-clearance selection policy.

    A configured plane is assumed to represent the corresponding physical road.
    This module cannot establish that a first-frame plane extrapolates correctly
    across a scene. Callers must describe that assumption in the policy fields.
    """

    ground_plane_policy: str
    clearance_mode: str = 'measured'
    extrapolation_policy: str = 'caller_declared_scene_plane_assumption'
    calibration_id: str | None = None
    min_up_normal_dot: float = .5
    min_clearance_m: float = .2
    max_clearance_m: float = 5.

    def __post_init__(self):
        if not self.ground_plane_policy.strip() or not self.extrapolation_policy.strip():
            raise ValueError('ground-plane and extrapolation policies must be explicit')
        if self.clearance_mode not in ('measured', 'nominal', 'measured_or_nominal'):
            raise ValueError('unknown clearance mode')
        if not 0 < self.min_up_normal_dot <= 1 or not 0 < self.min_clearance_m < self.max_clearance_m:
            raise ValueError('invalid clearance or normal-conditioning limits')


@dataclass(frozen=True)
class BevLidarHeightResult:
    proposal_valid: bool
    reason: str
    input_map_T_lidar: np.ndarray
    proposed_map_T_lidar: np.ndarray | None
    applied_map_T_lidar: np.ndarray
    input_map_T_camera: np.ndarray | None = None
    proposed_map_T_camera: np.ndarray | None = None
    applied_map_T_camera: np.ndarray | None = None
    applied: bool = False
    accepted: bool = False
    diagnostics: dict = field(default_factory=dict)
    method_id: str = field(default='bev_lidar_height', init=False)

    def with_gate_results(self, *, upstream_accepted: bool, registration_accepted: bool):
        """Finalize only after the caller runs the unchanged original gates."""
        diagnostics = dict(self.diagnostics, upstream_accepted=bool(upstream_accepted),
                           original_registration_accepted=bool(registration_accepted),
                           original_registration_gates_applied=True)
        if not self.proposal_valid:
            reason = self.reason
        elif not upstream_accepted:
            reason = 'upstream_bev_rejected'
        elif not registration_accepted:
            reason = 'original_registration_gates_rejected'
        else:
            return replace(self, applied=True, accepted=True, reason='all_gates_passed',
                           applied_map_T_lidar=self.proposed_map_T_lidar.copy(),
                           applied_map_T_camera=None if self.proposed_map_T_camera is None else self.proposed_map_T_camera.copy(),
                           diagnostics=diagnostics)
        return replace(self, applied=False, accepted=False, reason=reason,
                       applied_map_T_lidar=self.input_map_T_lidar.copy(),
                       applied_map_T_camera=None if self.input_map_T_camera is None else self.input_map_T_camera.copy(),
                       diagnostics=diagnostics)


class BevLidarHeightEstimator:
    """Match measured *normal* clearance to a configured map road plane.

    For normalized map plane ``n @ point + d = 0`` and frozen BEV up ``u``,
    the only update is ``delta = u * (clearance - n @ t - d) / (n @ u)``.
    Input rotation is unchanged. Horizontal translation is preserved in the
    frozen BEV basis; another reference basis can have different horizontal axes.

    This assumes corresponding physical source/map road planes. It is not a map
    registration, a reference-height fit, or proof that a scene-wide plane is
    accurate. No target/reference height is accepted by the API.
    """

    def __init__(self, config: BevLidarHeightConfig):
        self.config = config

    @staticmethod
    def _transform(value, name):
        value = np.asarray(value, dtype=float)
        if value.shape != (4, 4) or not np.isfinite(value).all() or not np.allclose(value[3], [0, 0, 0, 1]):
            raise ValueError(f'{name} must be a finite rigid transform')
        if not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-6) or np.linalg.det(value[:3, :3]) < .999:
            raise ValueError(f'{name} must have a right-handed orthonormal rotation')
        return value.copy()

    def estimate(self, *, map_T_lidar, map_up, ground_plane: GroundPlane,
                 lidar_normal_clearance_m: float | None, source_fit_accepted: bool,
                 lidar_T_camera=None, nominal_normal_clearance_m: float | None = None,
                 nominal_fit_accepted: bool = False) -> BevLidarHeightResult:
        pose = self._transform(map_T_lidar, 'map_T_lidar')
        extrinsic = None if lidar_T_camera is None else self._transform(lidar_T_camera, 'lidar_T_camera')
        camera = None if extrinsic is None else pose @ extrinsic
        up = np.array(map_up, dtype=float, copy=True)
        if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
            raise ValueError('map_up must be a finite nonzero vector')
        up /= np.linalg.norm(up)
        if not isinstance(ground_plane, GroundPlane):
            raise TypeError('ground_plane must be an explicit GroundPlane')
        normal = np.asarray(ground_plane.normal)
        denominator = float(normal @ up)
        diagnostics = dict(config=asdict(self.config), ground_plane=ground_plane.to_dict(),
                           map_up=up.tolist(), normal_up_dot=denominator,
                           source_fit_accepted=bool(source_fit_accepted),
                           nominal_fit_accepted=bool(nominal_fit_accepted),
                           clearance_definition='perpendicular distance to physical road plane, not vertical distance',
                           source_map_road_correspondence='assumed by configured ground-plane policy',
                           original_registration_gates_applied=False)
        result = BevLidarHeightResult(False, '', pose, None, pose.copy(),
                                      input_map_T_camera=camera, applied_map_T_camera=None if camera is None else camera.copy(),
                                      diagnostics=diagnostics)
        if denominator < self.config.min_up_normal_dot:
            return replace(result, reason='ground_normal_up_misaligned_or_ill_conditioned')
        mode = self.config.clearance_mode
        if mode == 'nominal' or (mode == 'measured_or_nominal' and not source_fit_accepted):
            clearance, valid, selected = nominal_normal_clearance_m, nominal_fit_accepted, 'nominal'
        else:
            clearance, valid, selected = lidar_normal_clearance_m, source_fit_accepted, 'measured'
        diagnostics['clearance_source'] = selected
        if not valid or clearance is None:
            return replace(result, reason=f'{selected}_clearance_unavailable_or_fit_rejected')
        clearance = float(clearance)
        if not np.isfinite(clearance) or not self.config.min_clearance_m <= clearance <= self.config.max_clearance_m:
            return replace(result, reason='clearance_outside_physical_limits')
        current_distance = float(normal @ pose[:3, 3] + ground_plane.offset_m)
        shift = (clearance - current_distance) / denominator
        proposed = pose.copy(); proposed[:3, 3] += up * shift
        proposed_camera = None if extrinsic is None else proposed @ extrinsic
        diagnostics.update(selected_normal_clearance_m=clearance,
                           initial_map_plane_distance_m=current_distance,
                           proposed_map_plane_distance_m=float(normal @ proposed[:3, 3] + ground_plane.offset_m),
                           height_shift_along_bev_up_m=float(shift), translation_map=(up * shift).tolist(),
                           fixed_rotation_error=float(np.max(np.abs(proposed[:3, :3] - pose[:3, :3]))),
                           frozen_horizontal_translation_error_m=float(np.linalg.norm(
                               (np.eye(3) - np.outer(up, up)) @ (proposed[:3, 3] - pose[:3, 3]))))
        if proposed_camera is not None:
            diagnostics.update(camera_normal_clearance_m=float(normal @ proposed_camera[:3, 3] + ground_plane.offset_m),
                               camera_optical_forward_dot_map_normal=float(normal @ proposed_camera[:3, 2]),
                               camera_down_dot_map_normal=float(normal @ proposed_camera[:3, 1]))
        return replace(result, proposal_valid=True, reason='proposal_ready_requires_original_gates',
                       proposed_map_T_lidar=proposed, proposed_map_T_camera=proposed_camera)
