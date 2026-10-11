"""Reader contracts using small, real protobuf MCAPs (no source data writes)."""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import pytest
from mcap_protobuf.writer import Writer
from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from foxglove_schemas_protobuf.SceneUpdate_pb2 import SceneUpdate
from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud

from src.data.foxglove_mcap import FoxgloveMcapReader


def _scene(frame: int, *, heading: bool = False, empty: bool = False, duplicate: bool = False) -> SceneUpdate:
    update = SceneUpdate()
    entity = update.entities.add(id="global_bboxes", frame_id="map")
    if not empty:
        for track_id in (["7", "7"] if duplicate else ["7"]):
            cube = entity.cubes.add()
            cube.pose.position.z = 10
            cube.pose.orientation.w = 1
            cube.size.x, cube.size.y, cube.size.z = 4, 2, 1.5
            entity.texts.add(text=track_id)
            entity.metadata.add(key=f"label:{track_id}", value="vehicle.bus.rigid")
            if heading:
                marker = entity.cubes.add()
                marker.CopyFrom(cube)
                marker.size.y = marker.size.z = 0.05
    update.entities.add(id=f"{frame}_frame_label", frame_id="map").texts.add(text=str(frame))
    return update


def _write(path: Path, *, frames: int = 2, heading: bool = False, empty: bool = False,
           duplicate: bool = False, missing_pose: bool = False, skew: bool = False,
           corrupt_image: bool = False, source_style: bool = False, lidar: bool = False,
           split_updates: bool = False) -> None:
    with path.open("wb") as output:
        writer = Writer(output)
        if not source_style:
            writer._writer.add_metadata("teleop", {
                "dataset": "nuscenes", "camera_intrinsic": json.dumps([100, 0, 50, 0, 100, 40, 0, 0, 1]),
                "frames_per_second": "10", "scene_generation": "fixture",
                "map_T_nuscenes_global": json.dumps(np.eye(4).reshape(-1).tolist()),
            })
        for index in range(frames):
            time = 1_000_000_000 + index * 100_000_000
            image = CompressedImage(frame_id="1_cam", format="png")
            image.data = b"bad" if corrupt_image else cv2.imencode('.png', np.zeros((80, 100, 3), dtype=np.uint8))[1].tobytes()
            writer.write_message('/1_Camera', image, log_time=time)
            for child in (['1_base_link'] if source_style else ['1_base_link', '1_cam']):
                if missing_pose and child == '1_base_link' and index == 0:
                    continue
                pose = FrameTransform(parent_frame_id='map', child_frame_id=child)
                pose.rotation.w = 1
                writer.write_message('/tf', pose, log_time=time + (250_000_000 if skew else 5_000_000))
            scene = _scene(index, heading=heading, empty=empty, duplicate=duplicate)
            if split_updates:
                for entity in scene.entities:
                    update = SceneUpdate()
                    update.entities.add().CopyFrom(entity)
                    writer.write_message('/bbox_global', update, log_time=time+10_000_000)
            else:
                writer.write_message('/bbox_global', scene, log_time=time+10_000_000)
            # Local duplicate annotations must not leak into normalized global tracks.
            writer.write_message('/1_Bbox', scene, log_time=time+10_000_000)
            if lidar:
                cloud = PointCloud(frame_id='1_base_link', point_stride=16)
                cloud.pose.orientation.w = 1
                for n, name in enumerate(('x', 'y', 'z', 'intensity')):
                    cloud.fields.add(name=name, offset=4*n, type=7)
                cloud.data = np.array([[1,2,3,4]],dtype='<f4').tobytes()
                writer.write_message('/1_LIDAR', cloud, log_time=time+5_000_000)
        writer.finish()


@pytest.mark.parametrize('heading', [False, True])
@pytest.mark.parametrize('split_updates', [False, True])
def test_join_and_preserve_tracks(tmp_path, heading, split_updates):
    path = tmp_path/'fixture.mcap'
    _write(path, heading=heading, split_updates=split_updates, lidar=True)
    frames = list(FoxgloveMcapReader(path, include_lidar=True))
    assert len(frames) == 2
    assert [frame.metadata['timestamp_us'] for frame in frames] == [0, 100000]
    assert [frame.metadata['frame_id'] for frame in frames] == ['0', '1']
    assert frames[0].timestamp_ns == 1_000_000_000
    assert frames[0].image_bgr.shape == (80,100,3)
    assert len(frames[0].vehicles) == len(frames[0].metadata['boxes']) == 1
    assert frames[0].vehicles[0].dimensions_lwh == (4,2,1.5)
    assert frames[0].metadata['boxes'][0]['text_label'] == 'vehicle.bus.rigid'
    assert frames[0].metadata['map_T_nuscenes_global'] == np.eye(4).reshape(-1).tolist()
    np.testing.assert_array_equal(frames[0].lidar_points, [[1,2,3,4]])
    assert frames[-1].metadata['end_of_scene']


def test_empty_frames_preserve_identity(tmp_path):
    path=tmp_path/'empty.mcap'; _write(path, empty=True)
    frames=list(FoxgloveMcapReader(path))
    assert [frame.metadata['frame_id'] for frame in frames] == ['0','1']
    assert all(not frame.vehicles for frame in frames)


def test_source_style_calibration_fallback(tmp_path):
    from src.localization.cooperscene_calibration import camera_intrinsic, camera_T_lidar
    path=tmp_path/'source.mcap'; _write(path, source_style=True, heading=True, split_updates=True)
    frame=next(iter(FoxgloveMcapReader(path, scene_generation='source-take')))
    assert frame.metadata['scene_generation']=='source-take'
    np.testing.assert_allclose(np.reshape(frame.metadata['camera_intrinsic'],(3,3)),camera_intrinsic('1'))
    np.testing.assert_allclose(np.reshape(frame.metadata['world_T_camera'],(4,4)),np.linalg.inv(camera_T_lidar('1')))


@pytest.mark.parametrize('option,match', [
    ('duplicate','unique'), ('missing_pose','incomplete|out of sync'),
    ('skew','out of sync'), ('corrupt_image','decode'),
])
def test_bad_recordings_fail(tmp_path, option, match):
    path=tmp_path/'bad.mcap';_write(path,**{option:True})
    with pytest.raises(ValueError,match=match):
        list(FoxgloveMcapReader(path))


def test_requested_missing_agent_or_lidar_fails(tmp_path):
    path=tmp_path/'missing.mcap';_write(path)
    with pytest.raises(ValueError,match='camera/global-box'):
        list(FoxgloveMcapReader(path,agent='2'))
    with pytest.raises(ValueError,match='LiDAR sample'):
        list(FoxgloveMcapReader(path,include_lidar=True))
