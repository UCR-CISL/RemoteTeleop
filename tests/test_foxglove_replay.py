"""Offline exporter-to-adapter compatibility without ROS or network services."""
from types import SimpleNamespace

import numpy as np
import pytest

from src.data.processed_nuscenes_mcap import ProcessedNuScenesMcapConfig, ProcessedNuScenesMcapProcessor
from src.deployment.ros_frame_adapter import McapFrameReplay
from test_processed_nuscenes_mcap import _write_scene


def test_foxglove_replay_autodetects_and_joins_normalized_sensor_timestamps(tmp_path):
    _write_scene(tmp_path / 'input' / '152', 2)
    processor = ProcessedNuScenesMcapProcessor(ProcessedNuScenesMcapConfig(
        processed_root=tmp_path / 'input', output_root=tmp_path / 'output', expected_samples=2,
    ))
    processor.process()
    images, metadata, clouds, waits = [], [], [], []
    now = [0.0]
    def sleep(seconds):
        waits.append(seconds)
        now[0] += seconds
    adapter = SimpleNamespace(
        scene_generation='nuscenes:152',
        analysis_assembler=SimpleNamespace(receive_image=images.append, raise_if_failed=lambda: None),
        bevfusion=SimpleNamespace(receive_image=lambda image: None, receive_lidar=clouds.append, poll=lambda: None),
        receive_metadata=metadata.append,
    )
    McapFrameReplay(adapter, image_topic='/camera/image_raw', metadata_topic='/camera/frame_detections',
                    lidar_topic='/lidar/points', expected_samples=2,
                    monotonic=lambda: now[0], sleep=sleep).replay(processor.outputs['152'])
    assert len(images) == len(metadata) == len(clouds) == 2
    assert sum(waits) == pytest.approx(.1)
    assert max(waits) <= .01
    for index, (image, payload, cloud) in enumerate(zip(images, metadata, clouds)):
        assert payload['timestamp_us'] == index * 100_000
        assert image.header.stamp.nanosec == index * 100_000_000
        assert cloud.header.stamp == image.header.stamp
        assert payload['scene_generation'] == 'nuscenes:152'
        assert np.asarray(payload['world_T_camera']).size == 16
        assert cloud.points.shape[1] == 4
