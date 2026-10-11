"""Past-only native LiDAR accumulation for explicit offline detector trials."""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from src.localization.transforms import _rigid_transform


@dataclass(frozen=True)
class _Sweep:
    points: np.ndarray
    world_T_lidar: np.ndarray
    timestamp_us: int


class PastSweepAssembler:
    """Compensate past scans into the current LiDAR frame using supplied poses.

    Features are x, y, z, intensity, positive capture age in seconds. Only ego
    motion is compensated; moving objects may leave trails. No GT is consumed.
    Native 10 Hz history differs from the checkpoint's nuScenes 20 Hz history.
    """

    def __init__(self, maximum_past_sweeps: int = 9, maximum_age_seconds: float = 1.0):
        if (isinstance(maximum_past_sweeps, bool)
                or not isinstance(maximum_past_sweeps, int)
                or not 0 <= maximum_past_sweeps <= 9):
            raise ValueError("maximum_past_sweeps must be an integer from 0 to 9")
        if not math.isfinite(maximum_age_seconds) or maximum_age_seconds <= 0:
            raise ValueError("maximum_age_seconds must be finite and positive")
        self.maximum_past_sweeps = maximum_past_sweeps
        self.maximum_age_us = maximum_age_seconds * 1_000_000
        self._history: deque[_Sweep] = deque(maxlen=maximum_past_sweeps)
        self._sequence_id: str | None = None
        self._timestamp_us: int | None = None
        self.last_past_timestamps_us: tuple[int, ...] = ()

    def reset(self) -> None:
        self._history.clear()
        self._sequence_id = self._timestamp_us = None
        self.last_past_timestamps_us = ()

    def add(self, points_xyzi: np.ndarray, world_T_lidar: np.ndarray,
            timestamp_us: int, sequence_id: str) -> np.ndarray:
        """Return current first, then newest past scans; keep source arrays fixed."""
        points = np.asarray(points_xyzi)
        if (points.ndim != 2 or points.shape[1] != 4
                or not np.issubdtype(points.dtype, np.number)
                or not np.isfinite(points).all()):
            raise ValueError("points_xyzi must be a finite numeric Nx4 array")
        pose = _rigid_transform(world_T_lidar, "world_T_lidar")
        if (isinstance(timestamp_us, bool) or not isinstance(timestamp_us, (int, np.integer))):
            raise ValueError("timestamp_us must be an integer capture time")
        if not isinstance(sequence_id, str) or not sequence_id:
            raise ValueError("sequence_id must be nonempty")
        if (sequence_id == self._sequence_id and self._timestamp_us is not None
                and timestamp_us <= self._timestamp_us):
            raise ValueError("capture timestamps must strictly increase within a sequence")
        if sequence_id != self._sequence_id:
            self.reset()
        timestamp_us = int(timestamp_us)
        while self._history and timestamp_us-self._history[0].timestamp_us > self.maximum_age_us:
            self._history.popleft()
        current = np.zeros((len(points), 5), dtype=np.float32)
        current[:, :4] = points
        scans = [current]
        current_T_world = np.linalg.inv(pose)
        selected = tuple(reversed(self._history))
        for sweep in selected:
            # Match upstream's square near-origin exclusion in the OLD sensor
            # frame. The current scan is never passed through this exclusion.
            old = sweep.points
            old = old[(abs(old[:, 0]) >= 1.0) | (abs(old[:, 1]) >= 1.0)]
            current_T_old = current_T_world @ sweep.world_T_lidar
            transformed = np.empty((len(old), 5), dtype=np.float32)
            transformed[:, :3] = old[:, :3] @ current_T_old[:3, :3].T + current_T_old[:3, 3]
            transformed[:, 3] = old[:, 3]
            transformed[:, 4] = (timestamp_us-sweep.timestamp_us)/1_000_000
            scans.append(transformed)
        result = np.ascontiguousarray(np.concatenate(scans))
        self._history.append(_Sweep(points.copy(), pose.copy(), timestamp_us))
        self._sequence_id, self._timestamp_us = sequence_id, timestamp_us
        self.last_past_timestamps_us = tuple(sweep.timestamp_us for sweep in selected)
        return result
