import cv2
import numpy as np

from src.realtime.composited_camera_process import CompositedFrameWriter


def test_composited_frame_writer_saves_numbered_png_and_video(tmp_path):
    writer = CompositedFrameWriter(tmp_path, 10.0)
    path = writer.write("frame-1", np.full((12, 20, 3), 0.5, dtype=np.float32))
    writer.close()

    assert path.name == "000000_frame-1.png"
    assert cv2.imread(str(path)).shape == (12, 20, 3)
    assert (tmp_path / "composited.mp4").stat().st_size > 0


def test_composited_frame_writer_rejects_resolution_changes(tmp_path):
    writer = CompositedFrameWriter(tmp_path, 10.0)
    try:
        writer.write("frame-1", np.zeros((12, 20, 3), dtype=np.float32))
        with np.testing.assert_raises_regex(ValueError, "frame size changed"):
            writer.write("frame-2", np.zeros((10, 20, 3), dtype=np.float32))
    finally:
        writer.close()
