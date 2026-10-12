"""Offline camera and LiDAR BEV diagnostics for evaluated CooperScene boxes.

All poses and points must already share the declared evaluation world frame.
These overlays consume geometric assignments; GT never enters the tracker.
Camera imagery is OpenCV BGR. Rendering does not alter evaluation poses.
"""
from pathlib import Path
from collections.abc import Iterable, Mapping, Sequence

import cv2
import numpy as np


class TrackingOverlayRenderer:
    """Render a fixed-scale BEV beside the recorded camera with labelled IDs."""

    GT_COLOR = (80, 220, 80)
    PRED_COLOR = (255, 220, 40)
    MISS_COLOR = (0, 220, 255)
    FP_COLOR = (70, 70, 255)
    SWITCH_COLOR = (220, 60, 220)
    _SIGNS = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
        [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ])
    _EDGES = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6),
              (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7))

    def __init__(self, *, bev_size: int = 720, extent_m: float = 60.0,
                 compact_labels: bool = False):
        if bev_size < 100 or extent_m <= 0:
            raise ValueError("BEV size must be >=100 pixels and extent positive")
        self.bev_size = int(bev_size)
        self.extent_m = float(extent_m)
        self.compact_labels = compact_labels

    def _label(self, kind, box, score, suffix):
        identifier = str(box['track_id'])
        if self.compact_labels:
            parts = identifier.split(':')
            if (kind == 'GT' and len(parts) == 2 and parts[0].startswith('take')
                    and parts[0][4:].isdigit() and parts[1].isdigit()):
                return f'GT{parts[1]}{suffix}'
            if (kind == 'P' and len(parts) == 3 and parts[:2] == ['bevfusion', 'vehicle']
                    and parts[2].isdigit()):
                return f'P{parts[2]}{score}{suffix}'
            if (kind == 'P' and len(parts) == 3 and parts[0] == 'proposal'
                    and all(part.isdigit() for part in parts[1:])):
                return f'D{parts[1]}.{parts[2]}{score}{suffix}'
        return f"{kind}:{identifier} {box.get('label', '')}{score}{suffix}"

    @staticmethod
    def _pose(box):
        return np.asarray(box['world_T_object'], dtype=float).reshape(4, 4)

    def _corners(self, box):
        pose = self._pose(box)
        local = self._SIGNS * np.asarray(box['dimensions_lwh']) / 2
        return local @ pose[:3, :3].T + pose[:3, 3]

    @staticmethod
    def _pixel(point, intrinsic):
        projected = intrinsic @ point
        return np.clip(projected[:2] / projected[2], -1e6, 1e6).astype(int)

    def _camera_box(self, image, box, camera_T_world, intrinsic, color, text):
        corners = self._corners(box)
        camera = corners @ camera_T_world[:3, :3].T + camera_T_world[:3, 3]
        for a, b in self._EDGES:
            start, end = camera[a].copy(), camera[b].copy()
            if max(start[2], end[2]) <= .1:
                continue
            if min(start[2], end[2]) < .1:
                fraction = (.1 - start[2]) / (end[2] - start[2])
                intersection = start + fraction * (end - start)
                if start[2] < .1:
                    start = intersection
                else:
                    end = intersection
            self._line(image, self._pixel(start, intrinsic), self._pixel(end, intrinsic), color)
        center = camera.mean(axis=0)
        if center[2] > .1:
            x, y = self._pixel(center, intrinsic)
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                cv2.putText(image, text, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)

    @staticmethod
    def _line(image, start, end, color, thickness=1):
        inside, a, b = cv2.clipLine((0, 0, image.shape[1], image.shape[0]),
                                  tuple(map(int, start)), tuple(map(int, end)))
        if inside:
            cv2.line(image, a, b, color, thickness, cv2.LINE_AA)

    def _bev_pixels(self, points, center):
        offset = np.asarray(points)[..., :2] - center
        scale = (self.bev_size - 1) / (2 * self.extent_m)
        # East to the right, north upwards; axes are explicitly labelled.
        return np.clip(np.stack((offset[..., 0], -offset[..., 1]), axis=-1) * scale
                       + (self.bev_size - 1) / 2, -1e6, 1e6).astype(int)

    def render(self, camera_bgr: np.ndarray, *, world_T_camera, intrinsic,
               lidar_world: np.ndarray, predictions: Sequence[Mapping],
               gt: Sequence[Mapping], assignment: Mapping,
               title: str = '', prediction_kind: str = 'tracks',
               center_xy=None) -> np.ndarray:
        """Return BGR image; assignment uses matches/false_positive_indices/missed_gt_indices."""
        camera = np.asarray(camera_bgr, dtype=np.uint8).copy()
        world_T_camera = np.asarray(world_T_camera, dtype=float).reshape(4, 4)
        camera_T_world = np.linalg.inv(world_T_camera)
        intrinsic = np.asarray(intrinsic, dtype=float).reshape(3, 3)
        center = np.asarray(world_T_camera[:2, 3] if center_xy is None else center_xy)
        bev = np.full((self.bev_size, self.bev_size, 3), 15, dtype=np.uint8)
        for distance in np.arange(-self.extent_m, self.extent_m + .001, 10):
            for axis in range(2):
                ends = np.array([[-self.extent_m, distance], [self.extent_m, distance]])
                if axis:
                    ends = ends[:, ::-1]
                pixels = self._bev_pixels(ends + center, center)
                self._line(bev, pixels[0], pixels[1], (35, 35, 35))
        points = np.asarray(lidar_world, dtype=float)
        if points.size == 0:
            points = np.empty((0, 3))
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError('LiDAR world cloud must have shape Nx3 or Nx4')
        points = points[:, :3]
        points = points[np.isfinite(points).all(axis=1)]
        # Keep output cost bounded while retaining the spatial distribution.
        points = points[::max(1, len(points) // 80000)]
        pixels = self._bev_pixels(points, center)
        valid = ((pixels >= 0) & (pixels < self.bev_size)).all(axis=1)
        bev[pixels[valid, 1], pixels[valid, 0]] = (90, 90, 90)
        misses = set(assignment.get('missed_gt_indices', ()))
        false_positives = set(assignment.get('false_positive_indices', ()))
        switched = {str(row['pred_id']) for row in assignment.get('id_switches', ())}
        for kind, boxes in (('GT', gt), ('P', predictions)):
            for index, box in enumerate(boxes):
                identifier = str(box['track_id'])
                color = self.GT_COLOR if kind == 'GT' else self.PRED_COLOR
                suffix = ''
                if kind == 'GT' and index in misses:
                    color, suffix = self.MISS_COLOR, ' MISS'
                elif kind == 'P' and index in false_positives:
                    color, suffix = self.FP_COLOR, ' FP'
                if kind == 'P' and identifier in switched:
                    color, suffix = self.SWITCH_COLOR, suffix + ' IDSW'
                score = f" {float(box['score']):.2f}" if kind == 'P' and 'score' in box else ''
                text = self._label(kind, box, score, suffix)
                self._camera_box(camera, box, camera_T_world, intrinsic, color, text)
                corners = self._bev_pixels(self._corners(box)[:4], center)
                for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
                    self._line(bev, corners[a], corners[b], color, 2)
                pose = self._pose(box)
                start = self._bev_pixels(pose[:3, 3], center)
                end = self._bev_pixels(pose[:3, 3] + pose[:3, 0] * box['dimensions_lwh'][0] / 2, center)
                self._line(bev, start, end, color, 2)
                if ((start >= 0) & (start < self.bev_size)).all():
                    cv2.putText(bev, text, tuple(start), cv2.FONT_HERSHEY_SIMPLEX, .38, color, 1, cv2.LINE_AA)
        for match in assignment.get('matches', ()):
            a = self._pose(gt[match['gt_index']])[:3, 3]
            b = self._pose(predictions[match['pred_index']])[:3, 3]
            self._line(bev, self._bev_pixels(a, center), self._bev_pixels(b, center), (230, 230, 230))
            camera_points = np.array([a, b]) @ camera_T_world[:3, :3].T + camera_T_world[:3, 3]
            if (camera_points[:, 2] > .1).all():
                self._line(camera, self._pixel(camera_points[0], intrinsic),
                           self._pixel(camera_points[1], intrinsic), (230, 230, 230))
        cv2.putText(bev, 'ENU: East ->   North ^', (10, self.bev_size - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (200, 200, 200), 1)
        height = self.bev_size
        camera = cv2.resize(camera, (round(camera.shape[1] * height / camera.shape[0]), height))
        output = np.hstack((camera, bev))
        header = np.zeros((76, output.shape[1], 3), dtype=np.uint8)
        cv2.putText(header, f'{title} | {prediction_kind} | wireframe diagnostic (no depth occlusion)',
                    (10, 23), cv2.FONT_HERSHEY_SIMPLEX, .55, (240, 240, 240), 1)
        x = 10
        for label, color in [('GT', self.GT_COLOR), ('Predicted', self.PRED_COLOR),
                             ('MISS', self.MISS_COLOR), ('FP', self.FP_COLOR), ('IDSW', self.SWITCH_COLOR)]:
            cv2.putText(header, label, (x, 54), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1)
            x += 135
        return np.vstack((header, output))

    @staticmethod
    def save_png(path: str | Path, image: np.ndarray) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f'Failed to write {path}')

    @staticmethod
    def write_video(path: str | Path, frames: Iterable[np.ndarray], *, fps: float = 10) -> None:
        """Write identically sized BGR diagnostics and close the writer on error."""
        if fps <= 0:
            raise ValueError('Video fps must be positive')
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        writer = None
        try:
            for image in frames:
                if writer is None:
                    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), fps,
                                             (image.shape[1], image.shape[0]))
                    if not writer.isOpened():
                        raise RuntimeError(f'Failed to open video {path}')
                    expected_shape = image.shape
                if image.shape != expected_shape:
                    raise ValueError('Video frames must have the same shape')
                writer.write(image)
        finally:
            if writer is not None:
                writer.release()
