"""LiDAR localization against static Gaussian-splat maps."""

from .registration import PointCloudRegistrar, RegistrationConfig, RegistrationResult

__all__ = ["PointCloudRegistrar", "RegistrationConfig", "RegistrationResult"]
