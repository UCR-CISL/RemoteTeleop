"""Fuse scan registration with a metric ENU pose prior."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class EnuPriorFusionConfig:
    """Configuration for deterministic robust ground-plane MAP fusion."""

    prior_weight: float = 0.14
    weighting_mode: Literal["fixed", "uncertainty"] = "fixed"
    position_mode: Literal["bev", "xyz"] = "bev"
    prior_translation_std_m: float = 0.10
    prior_vertical_std_m: float = 0.25
    prior_heading_std_deg: float = 1.0
    uncertainty_block_size_m: float = 2.0
    uncertainty_min_block_points: int = 20
    scan_translation_std_floor_m: float = 0.02
    scan_heading_std_floor_deg: float = 0.05
    heading_lever_arm_m: float = 32.0
    source_stride: int = 4
    max_correspondence_distance_m: float = 0.8
    huber_scale_m: float = 0.3
    max_iterations: int = 80
    convergence_tolerance: float = 1e-8
    finite_difference_step: float = 1e-6

    def __post_init__(self) -> None:
        positive = (
            self.prior_weight, self.heading_lever_arm_m,
            self.prior_translation_std_m, self.prior_vertical_std_m,
            self.prior_heading_std_deg,
            self.uncertainty_block_size_m, self.scan_translation_std_floor_m,
            self.scan_heading_std_floor_deg,
            self.max_correspondence_distance_m, self.huber_scale_m,
            self.convergence_tolerance, self.finite_difference_step,
        )
        if any(value <= 0 for value in positive):
            raise ValueError("ENU-prior fusion scales and weights must be positive")
        if min(self.source_stride, self.max_iterations, self.uncertainty_min_block_points) < 1:
            raise ValueError("strides, iterations, and block point count must be positive")
        if self.weighting_mode not in ("fixed", "uncertainty"):
            raise ValueError("weighting_mode must be 'fixed' or 'uncertainty'")
        if self.position_mode not in ("bev", "xyz"):
            raise ValueError("position_mode must be 'bev' or 'xyz'")


@dataclass(frozen=True)
class EnuPriorFusionResult:
    map_T_source: NDArray[np.float64]
    iterations: int
    converged: bool
    correspondence_count: int
    prior_bev_error_m: float
    prior_heading_error_deg: float
    prior_vertical_error_m: float | None = None
    prior_translation_error_m: float | None = None
    weighting_mode: str = "fixed"
    position_mode: str = "bev"
    estimated_scan_residual_std_m: float | None = None
    effective_prior_translation_weight: float | None = None
    effective_prior_heading_weight: float | None = None
    uncertainty_total_block_count: int | None = None
    uncertainty_eligible_block_count: int | None = None
    data_hessian_eigenvalues: NDArray[np.float64] | None = None
    data_hessian_condition_number: float | None = None
    scan_transform_covariance: NDArray[np.float64] | None = None
    scan_transform_sandwich_covariance: NDArray[np.float64] | None = None
    scan_transform_block_heterogeneity_covariance: NDArray[np.float64] | None = None
    prior_lidar_pose_covariance: NDArray[np.float64] | None = None
    posterior_transform_covariance: NDArray[np.float64] | None = None
    innovation_nis: float | None = None
    innovation_exceeds_nominal_chi2_95pct: bool | None = None


class EnuPriorFuser:
    """Regularize ICP position/yaw while retaining its roll and pitch."""

    def __init__(self, config: EnuPriorFusionConfig | None = None) -> None:
        self.config = config or EnuPriorFusionConfig()

    def fuse(
        self,
        source_points: NDArray[np.floating],
        map_points: NDArray[np.floating],
        scan_map_T_source: NDArray[np.floating],
        prior_map_T_source: NDArray[np.floating],
        source_T_lidar: NDArray[np.floating],
    ) -> EnuPriorFusionResult:
        source = _points(source_points, "source_points")[::self.config.source_stride]
        target = _points(map_points, "map_points")
        scan = _transform(scan_map_T_source, "scan_map_T_source")
        prior = _transform(prior_map_T_source, "prior_map_T_source")
        source_pose = _transform(source_T_lidar, "source_T_lidar")
        tree = cKDTree(target)

        scan_yaw = math.atan2(scan[1, 0], scan[0, 0])
        roll_pitch = _yaw_rotation(-scan_yaw) @ scan[:3, :3]
        parameters = np.array([
            scan[0, 3] - prior[0, 3],
            scan[1, 3] - prior[1, 3],
            scan_yaw,
        ])
        prior_pose = prior @ source_pose
        correspondence_count = 0
        converged = False

        if self.config.weighting_mode == "uncertainty":
            return self._fuse_with_uncertainty(
                source, tree, scan, prior, source_pose, roll_pitch
            )
        if self.config.position_mode == "xyz":
            return self._fuse_fixed_with_z(
                source, target, tree, scan, prior, source_pose, roll_pitch
            )
        scan_residual_std = None
        prior_weights = np.full(3, self.config.prior_weight, dtype=np.float64)

        def candidate(values: np.ndarray) -> np.ndarray:
            result = np.eye(4, dtype=np.float64)
            result[:3, :3] = _yaw_rotation(values[2]) @ roll_pitch
            result[:3, 3] = (
                prior[0, 3] + values[0],
                prior[1, 3] + values[1],
                scan[2, 3],
            )
            return result

        def prior_error(values: np.ndarray) -> np.ndarray:
            pose = candidate(values) @ source_pose
            heading_error = _wrap_angle(_heading(pose) - _heading(prior_pose))
            return np.array([
                pose[0, 3] - prior_pose[0, 3],
                pose[1, 3] - prior_pose[1, 3],
                self.config.heading_lever_arm_m * heading_error,
            ])

        for iteration in range(1, self.config.max_iterations + 1):
            transform = candidate(parameters)
            rotated = source @ transform[:3, :3].T
            transformed = rotated + transform[:3, 3]
            distances, indices = tree.query(transformed, workers=-1)
            keep = distances < self.config.max_correspondence_distance_m
            correspondence_count = int(np.count_nonzero(keep))
            if correspondence_count < 3:
                raise RuntimeError("ENU-prior fusion found fewer than three correspondences")

            residual = (transformed[keep] - target[indices[keep]])[:, :2]
            weights = np.minimum(
                1.0,
                self.config.huber_scale_m / np.maximum(distances[keep], 1e-9),
            )
            rotated_inliers = rotated[keep]
            jacobian = np.zeros((correspondence_count, 2, 3), dtype=np.float64)
            jacobian[:, 0, 0] = 1.0
            jacobian[:, 1, 1] = 1.0
            jacobian[:, 0, 2] = -rotated_inliers[:, 1]
            jacobian[:, 1, 2] = rotated_inliers[:, 0]
            data_hessian = np.einsum(
                "nai,n,naj->ij", jacobian, weights, jacobian
            ) / correspondence_count
            data_gradient = np.einsum(
                "nai,n,na->i", jacobian, weights, residual
            ) / correspondence_count

            error = prior_error(parameters)
            prior_jacobian = _forward_jacobian(
                prior_error, parameters, self.config.finite_difference_step
            )
            weighted_prior_jacobian = prior_weights[:, None] * prior_jacobian
            hessian = data_hessian + prior_jacobian.T @ weighted_prior_jacobian
            gradient = data_gradient + prior_jacobian.T @ (prior_weights * error)
            step = np.linalg.solve(hessian, -gradient)
            parameters += step
            if np.linalg.norm(step) < self.config.convergence_tolerance:
                converged = True
                break

        transform = candidate(parameters)
        error = prior_error(parameters)
        return EnuPriorFusionResult(
            map_T_source=transform,
            iterations=iteration,
            converged=converged,
            correspondence_count=correspondence_count,
            prior_bev_error_m=float(np.linalg.norm(error[:2])),
            prior_heading_error_deg=float(np.degrees(error[2] / self.config.heading_lever_arm_m)),
            weighting_mode=self.config.weighting_mode,
            position_mode=self.config.position_mode,
            estimated_scan_residual_std_m=scan_residual_std,
            effective_prior_translation_weight=float(prior_weights[0]),
            effective_prior_heading_weight=float(prior_weights[2]),
        )

    def _fuse_fixed_with_z(
        self,
        source: np.ndarray,
        target: np.ndarray,
        tree: cKDTree,
        scan: np.ndarray,
        prior: np.ndarray,
        source_pose: np.ndarray,
        roll_pitch: np.ndarray,
    ) -> EnuPriorFusionResult:
        scan_yaw = math.atan2(scan[1, 0], scan[0, 0])
        parameters = np.array([
            scan[0, 3] - prior[0, 3],
            scan[1, 3] - prior[1, 3],
            scan[2, 3] - prior[2, 3],
            scan_yaw,
        ])
        prior_pose = prior @ source_pose
        prior_weights = np.full(4, self.config.prior_weight, dtype=np.float64)
        correspondence_count = 0
        converged = False

        def candidate(values: np.ndarray) -> np.ndarray:
            result = np.eye(4, dtype=np.float64)
            result[:3, :3] = _yaw_rotation(values[3]) @ roll_pitch
            result[:3, 3] = prior[:3, 3] + values[:3]
            return result

        def prior_error(values: np.ndarray) -> np.ndarray:
            pose = candidate(values) @ source_pose
            heading_error = _wrap_angle(_heading(pose) - _heading(prior_pose))
            return np.concatenate((
                pose[:3, 3] - prior_pose[:3, 3],
                [self.config.heading_lever_arm_m * heading_error],
            ))

        for iteration in range(1, self.config.max_iterations + 1):
            transform = candidate(parameters)
            rotated = source @ transform[:3, :3].T
            transformed = rotated + transform[:3, 3]
            distances, indices = tree.query(transformed, workers=-1)
            keep = distances < self.config.max_correspondence_distance_m
            correspondence_count = int(np.count_nonzero(keep))
            if correspondence_count < 4:
                raise RuntimeError("3D ENU-prior fusion found fewer than four correspondences")
            residual = transformed[keep] - target[indices[keep]]
            weights = np.minimum(
                1.0,
                self.config.huber_scale_m / np.maximum(distances[keep], 1e-9),
            )
            rotated_inliers = rotated[keep]
            jacobian = np.zeros((correspondence_count, 3, 4), dtype=np.float64)
            jacobian[:, 0, 0] = 1.0
            jacobian[:, 1, 1] = 1.0
            jacobian[:, 2, 2] = 1.0
            jacobian[:, 0, 3] = -rotated_inliers[:, 1]
            jacobian[:, 1, 3] = rotated_inliers[:, 0]
            data_hessian = np.einsum(
                "nai,n,naj->ij", jacobian, weights, jacobian
            ) / correspondence_count
            data_gradient = np.einsum(
                "nai,n,na->i", jacobian, weights, residual
            ) / correspondence_count
            error = prior_error(parameters)
            prior_jacobian = _forward_jacobian(
                prior_error, parameters, self.config.finite_difference_step
            )
            weighted_prior_jacobian = prior_weights[:, None] * prior_jacobian
            hessian = data_hessian + prior_jacobian.T @ weighted_prior_jacobian
            gradient = data_gradient + prior_jacobian.T @ (prior_weights * error)
            step = np.linalg.solve(hessian, -gradient)
            parameters += step
            if np.linalg.norm(step) < self.config.convergence_tolerance:
                converged = True
                break

        transform = candidate(parameters)
        error = prior_error(parameters)
        return EnuPriorFusionResult(
            map_T_source=transform,
            iterations=iteration,
            converged=converged,
            correspondence_count=correspondence_count,
            prior_bev_error_m=float(np.linalg.norm(error[:2])),
            prior_vertical_error_m=float(abs(error[2])),
            prior_translation_error_m=float(np.linalg.norm(error[:3])),
            prior_heading_error_deg=float(
                np.degrees(error[3] / self.config.heading_lever_arm_m)
            ),
            weighting_mode="fixed",
            position_mode="xyz",
            effective_prior_translation_weight=float(prior_weights[0]),
            effective_prior_heading_weight=float(prior_weights[3]),
        )

    def _fuse_with_uncertainty(
        self,
        source: np.ndarray,
        tree: cKDTree,
        scan: np.ndarray,
        prior: np.ndarray,
        source_pose: np.ndarray,
        roll_pitch: np.ndarray,
    ) -> EnuPriorFusionResult:
        include_z = self.config.position_mode == "xyz"
        scan_yaw = math.atan2(scan[1, 0], scan[0, 0])
        scan_parameters = np.array(
            [scan[0, 3], scan[1, 3], scan[2, 3], scan_yaw]
            if include_z else [scan[0, 3], scan[1, 3], scan_yaw]
        )
        (
            scan_covariance,
            sandwich_covariance,
            dispersion_covariance,
            total_block_count,
            eligible_block_count,
            data_hessian_eigenvalues,
            data_hessian_condition_number,
            residual_std,
            correspondence_count,
        ) = self._estimate_scan_covariance(source, tree, scan, include_z=include_z)
        prior_heading_std_rad = math.radians(self.config.prior_heading_std_deg)
        prior_variances = [
            self.config.prior_translation_std_m**2,
            self.config.prior_translation_std_m**2,
        ]
        if include_z:
            prior_variances.append(self.config.prior_vertical_std_m**2)
        prior_variances.append(prior_heading_std_rad**2)
        prior_covariance = np.diag(prior_variances)
        scan_information = np.linalg.inv(scan_covariance)
        prior_information = np.linalg.inv(prior_covariance)
        prior_pose = prior @ source_pose

        def candidate(values: np.ndarray) -> np.ndarray:
            result = np.eye(4, dtype=np.float64)
            result[:3, :3] = _yaw_rotation(values[-1]) @ roll_pitch
            result[:3, 3] = (
                values[:3] if include_z else (values[0], values[1], scan[2, 3])
            )
            return result

        def prior_error(values: np.ndarray) -> np.ndarray:
            pose = candidate(values) @ source_pose
            position_error = pose[:3 if include_z else 2, 3] - prior_pose[
                :3 if include_z else 2, 3
            ]
            return np.concatenate((position_error, [
                _wrap_angle(_heading(pose) - _heading(prior_pose)),
            ]))

        initial_prior_error = prior_error(scan_parameters)
        initial_prior_jacobian = _forward_jacobian(
            prior_error, scan_parameters, self.config.finite_difference_step
        )
        innovation_covariance = (
            initial_prior_jacobian @ scan_covariance @ initial_prior_jacobian.T
            + prior_covariance
        )
        innovation_nis = float(
            initial_prior_error @ np.linalg.solve(innovation_covariance, initial_prior_error)
        )

        parameters = scan_parameters.copy()
        converged = False
        posterior_covariance = scan_covariance.copy()
        for iteration in range(1, self.config.max_iterations + 1):
            scan_error = parameters - scan_parameters
            scan_error[-1] = _wrap_angle(scan_error[-1])
            error = prior_error(parameters)
            prior_jacobian = _forward_jacobian(
                prior_error, parameters, self.config.finite_difference_step
            )
            hessian = scan_information + prior_jacobian.T @ prior_information @ prior_jacobian
            gradient = scan_information @ scan_error + prior_jacobian.T @ prior_information @ error
            step = np.linalg.solve(hessian, -gradient)
            parameters += step
            parameters[-1] = _wrap_angle(parameters[-1])
            posterior_covariance = np.linalg.inv(hessian)
            if np.linalg.norm(step) < self.config.convergence_tolerance:
                converged = True
                break

        transform = candidate(parameters)
        error = prior_error(parameters)
        return EnuPriorFusionResult(
            map_T_source=transform,
            iterations=iteration,
            converged=converged,
            correspondence_count=correspondence_count,
            prior_bev_error_m=float(np.linalg.norm(error[:2])),
            prior_vertical_error_m=(float(abs(error[2])) if include_z else None),
            prior_translation_error_m=(
                float(np.linalg.norm(error[:3])) if include_z else None
            ),
            prior_heading_error_deg=float(np.degrees(error[-1])),
            weighting_mode="uncertainty",
            position_mode=self.config.position_mode,
            estimated_scan_residual_std_m=residual_std,
            uncertainty_total_block_count=total_block_count,
            uncertainty_eligible_block_count=eligible_block_count,
            data_hessian_eigenvalues=data_hessian_eigenvalues,
            data_hessian_condition_number=data_hessian_condition_number,
            scan_transform_covariance=scan_covariance,
            scan_transform_sandwich_covariance=sandwich_covariance,
            scan_transform_block_heterogeneity_covariance=dispersion_covariance,
            prior_lidar_pose_covariance=prior_covariance,
            posterior_transform_covariance=posterior_covariance,
            innovation_nis=innovation_nis,
            innovation_exceeds_nominal_chi2_95pct=(
                innovation_nis > (
                    9.487729036781154 if include_z else 7.814727903251179
                )
            ),
        )

    def _estimate_scan_covariance(
        self,
        source: np.ndarray,
        tree: cKDTree,
        scan: np.ndarray,
        *,
        include_z: bool,
    ) -> tuple[
        np.ndarray, np.ndarray, np.ndarray, int, int,
        np.ndarray, float, float, int,
    ]:
        transformed = source @ scan[:3, :3].T + scan[:3, 3]
        distances, indices = tree.query(transformed, workers=-1)
        keep = distances < self.config.max_correspondence_distance_m
        if np.count_nonzero(keep) < 3:
            raise RuntimeError("cannot estimate scan uncertainty from fewer than three correspondences")
        correspondence_count = int(np.count_nonzero(keep))
        spatial_dimensions = 3 if include_z else 2
        parameter_count = 4 if include_z else 3
        residuals = (
            transformed[keep, :spatial_dimensions]
            - tree.data[indices[keep], :spatial_dimensions]
        )
        radial_median = float(np.median(np.linalg.norm(residuals, axis=1)))
        radial_median_factor = 1.5381722544550522 if include_z else math.sqrt(
            2.0 * math.log(2.0)
        )
        residual_std = radial_median / radial_median_factor
        rotated = source @ scan[:3, :3].T
        rotated_inliers = rotated[keep]
        jacobian = np.zeros(
            (correspondence_count, spatial_dimensions, parameter_count),
            dtype=np.float64,
        )
        jacobian[:, 0, 0] = 1.0
        jacobian[:, 1, 1] = 1.0
        if include_z:
            jacobian[:, 2, 2] = 1.0
        jacobian[:, 0, -1] = -rotated_inliers[:, 1]
        jacobian[:, 1, -1] = rotated_inliers[:, 0]
        weights = np.minimum(
            1.0,
            self.config.huber_scale_m / np.maximum(distances[keep], 1e-9),
        )
        blocks = np.floor(
            source[keep, :2] / self.config.uncertainty_block_size_m
        ).astype(np.int64)
        _, block_indices = np.unique(blocks, axis=0, return_inverse=True)
        total_block_count = int(block_indices.max()) + 1
        corrections = []
        data_hessian = np.einsum("nai,n,naj->ij", jacobian, weights, jacobian)
        point_scores = np.einsum(
            "nai,n,na->ni", jacobian, weights, residuals
        )
        block_scores = np.zeros(
            (total_block_count, parameter_count), dtype=np.float64
        )
        np.add.at(block_scores, block_indices, point_scores)
        for block_index in range(total_block_count):
            selected = block_indices == block_index
            if np.count_nonzero(selected) < self.config.uncertainty_min_block_points:
                continue
            block_jacobian = jacobian[selected]
            block_weights = weights[selected]
            block_residuals = residuals[selected]
            block_hessian = np.einsum(
                "nai,n,naj->ij", block_jacobian, block_weights, block_jacobian
            )
            condition_number = np.linalg.cond(block_hessian)
            if not np.isfinite(condition_number) or condition_number > 1e8:
                continue
            block_gradient = np.einsum(
                "nai,n,na->i", block_jacobian, block_weights, block_residuals
            )
            corrections.append(np.linalg.solve(block_hessian, -block_gradient))
        if len(corrections) < 6:
            raise RuntimeError("scan uncertainty requires at least six observable spatial blocks")

        scores = block_scores
        scores -= np.mean(scores, axis=0)
        data_hessian_inverse = np.linalg.pinv(data_hessian)
        sandwich_covariance = (
            data_hessian_inverse
            @ (scores.T @ scores * len(scores) / (len(scores) - 1))
            @ data_hessian_inverse
        )

        corrections_array = np.asarray(corrections)
        center = np.median(corrections_array, axis=0)
        robust_std = 1.4826 * np.median(
            np.abs(corrections_array - center), axis=0
        )
        floors = [
            self.config.scan_translation_std_floor_m,
            self.config.scan_translation_std_floor_m,
        ]
        if include_z:
            floors.append(self.config.scan_translation_std_floor_m)
        floors.append(math.radians(self.config.scan_heading_std_floor_deg))
        floors = np.asarray(floors)
        robust_std = np.maximum(robust_std, floors)
        clipped = np.clip(
            corrections_array,
            center - 3.0 * robust_std,
            center + 3.0 * robust_std,
        )
        dispersion_covariance = np.cov(clipped, rowvar=False)
        dispersion_covariance += np.diag(
            np.maximum(0.0, floors**2 - np.diag(dispersion_covariance))
        )
        scan_covariance = sandwich_covariance + dispersion_covariance
        scan_covariance = (scan_covariance + scan_covariance.T) / 2.0
        if np.linalg.eigvalsh(scan_covariance).min() <= 0:
            raise RuntimeError("estimated scan covariance is not positive definite")
        data_hessian_eigenvalues = np.linalg.eigvalsh(data_hessian)
        return (
            scan_covariance,
            sandwich_covariance,
            dispersion_covariance,
            total_block_count,
            len(corrections),
            data_hessian_eigenvalues,
            float(np.linalg.cond(data_hessian)),
            residual_std,
            correspondence_count,
        )


def _forward_jacobian(function, values: np.ndarray, step: float) -> np.ndarray:
    baseline = function(values)
    result = np.empty((len(baseline), len(values)), dtype=np.float64)
    for column in range(len(values)):
        shifted = values.copy()
        shifted[column] += step
        result[:, column] = (function(shifted) - baseline) / step
    return result


def _points(values: NDArray[np.floating], name: str) -> np.ndarray:
    points = np.asarray(values, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError(f"{name} must have shape (N, 3) with N > 0")
    if not np.isfinite(points).all():
        raise ValueError(f"{name} must contain only finite values")
    return points


def _transform(values: NDArray[np.floating], name: str) -> np.ndarray:
    transform = np.asarray(values, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0)):
        raise ValueError(f"{name} must be homogeneous")
    return transform


def _yaw_rotation(yaw: float) -> np.ndarray:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ])


def _heading(transform: np.ndarray) -> float:
    return math.atan2(transform[1, 0], transform[0, 0])


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
