"""Boundary for the future DriveStudio-backed static scene renderer."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class DriveStudioConfig:
    """Paths needed to load a processed DriveStudio scene in a later stage."""

    repository: Path = Path("thirdparty/drivestudio")
    checkpoint: Path | None = None
    map_from_nuscenes_global: np.ndarray | None = None

    def validate_repository(self) -> None:
        """Fail early when the pinned DriveStudio submodule is unavailable."""

        if not (self.repository / "tools" / "train.py").is_file():
            raise FileNotFoundError(
                f"DriveStudio checkout is missing at {self.repository}. "
                "Run `git submodule update --init --recursive`."
            )


class StaticSceneRenderer(Protocol):
    """Renderer interface kept independent of DriveStudio's Python environment."""

    def load(self) -> None:
        """Load the configured static Gaussian scene."""

    def render(self, world_T_camera: np.ndarray) -> np.ndarray:
        """Render an RGB image from an absolute metric camera pose."""
