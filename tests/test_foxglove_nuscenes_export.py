"""Real MCAP checks: schema compatibility, preserved geometry and source pixels."""
import json
from dataclasses import replace

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

from src.data.processed_nuscenes import ProcessedNuScenesSequence, ProcessedNuScenesVehicle
from src.data.processed_nuscenes_mcap import ProcessedNuScenesMcapConfig, ProcessedNuScenesMcapProcessor
from src.data.foxglove_mcap_writer import NuScenesFoxgloveWriter
from test_processed_nuscenes_mcap import _write_scene


def test_default_export_matches_cooperscene_schema_and_preserves_pixels_and_geometry(tmp_path):
    root = tmp_path / "processed"
    _write_scene(root / "152", 2)
    source = ProcessedNuScenesSequence(root / "152", expected_frame_count=2)
    processor = ProcessedNuScenesMcapProcessor(ProcessedNuScenesMcapConfig(
        processed_root=root, output_root=tmp_path / "output", expected_samples=2,
    ))
    assert processor.process() == 2
    with processor.outputs['152'].open('rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        records = list(reader.iter_decoded_messages())
        metadata = {item.name: item.metadata for item in reader.iter_metadata()}['teleop']
    assert metadata['dataset'] == 'nuscenes'
    assert metadata['frames_per_second'] == '10.0'
    assert json.loads(metadata['camera_intrinsic']) == source.camera_intrinsic.reshape(-1).tolist()
    topic_schemas = {channel.topic: schema.name for schema, channel, _, _ in records}
    assert topic_schemas == {
        '/1_Camera': 'foxglove.CompressedImage', '/1_LIDAR': 'foxglove.PointCloud',
        '/1_Bbox': 'foxglove.SceneUpdate', '/bbox_global': 'foxglove.SceneUpdate',
        '/tf': 'foxglove.FrameTransform',
    }
    assert all(channel.message_encoding == 'protobuf' and schema.encoding == 'protobuf'
               for schema, channel, _, _ in records)
    for _, channel, message, decoded in records:
        frame = source[message.sequence]
        assert message.log_time == frame.timestamp_us * 1000
        if channel.topic == '/1_Camera':
            np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(decoded.data, np.uint8), cv2.IMREAD_COLOR), frame.image)
        elif channel.topic == '/1_LIDAR':
            np.testing.assert_array_equal(np.frombuffer(decoded.data, '<f4').reshape(-1, 4), frame.lidar_points)
            assert decoded.frame_id == '1_base_link'
            assert [(field.name, field.offset) for field in decoded.fields] == [('x', 0), ('y', 4), ('z', 8), ('intensity', 12)]
        elif channel.topic in ('/1_Bbox', '/bbox_global'):
            entity = decoded.entities[0]
            assert entity.frame_id == 'map'
            assert [text.text for text in entity.texts] == ['stable-car']
            cube = entity.cubes[0]
            assert (cube.size.x, cube.size.y, cube.size.z) == (4., 2., 2.)
            np.testing.assert_allclose([cube.pose.position.x, cube.pose.position.y, cube.pose.position.z], frame.vehicles[0].map_T_object[:3, 3])
        elif channel.topic == '/tf':
            pose = frame.map_T_camera if decoded.child_frame_id == '1_cam' else frame.map_T_lidar
            np.testing.assert_allclose([decoded.translation.x, decoded.translation.y, decoded.translation.z], pose[:3, 3])
    assert not list((tmp_path / 'output').glob('*.partial'))


def test_export_keeps_all_vehicle_types_and_offscreen_tracks(tmp_path):
    _write_scene(tmp_path / '152', 1)
    source = ProcessedNuScenesSequence(tmp_path / '152', expected_frame_count=1)
    frame = source[0]
    categories = ('vehicle.car', 'vehicle.truck', 'vehicle.bus.rigid', 'vehicle.bus.bendy',
                  'vehicle.trailer', 'vehicle.construction', 'vehicle.motorcycle', 'vehicle.bicycle',
                  'vehicle.emergency.ambulance', 'vehicle.emergency.police')
    pose = np.eye(4)
    pose[2, 3] = -100  # Behind camera; must still be archived.
    vehicles = tuple(ProcessedNuScenesVehicle(str(i), category, (4., 2., 2.), pose.copy())
                     for i, category in enumerate(categories))
    replacement = replace(frame, vehicles=vehicles)

    class Sequence:
        timestamp_step_us = source.timestamp_step_us
        map_T_global = source.map_T_global
        gaussian_T_nuscenes_global = source.gaussian_T_nuscenes_global
        def __getitem__(self, index):
            if index != 0:
                raise IndexError(index)
            return replacement

    output = tmp_path / 'vehicles.mcap'
    NuScenesFoxgloveWriter().write(output, Sequence(), scene_generation='test')
    with output.open('rb') as stream:
        records = list(make_reader(stream, decoder_factories=[DecoderFactory()]).iter_decoded_messages(topics=['/bbox_global']))
    entity = records[0].decoded_message.entities[0]
    assert len(entity.cubes) == 10
    assert [text.text for text in entity.texts] == [str(i) for i in range(10)]
    assert {item.key: item.value for item in entity.metadata} == {f'label:{i}': category for i, category in enumerate(categories)}


def test_export_roundtrips_through_playback_reader(tmp_path):
    from src.data.foxglove_mcap import FoxgloveMcapReader

    root = tmp_path / 'processed'
    _write_scene(root / '152', 2)
    source = ProcessedNuScenesSequence(root / '152', expected_frame_count=2)
    processor = ProcessedNuScenesMcapProcessor(ProcessedNuScenesMcapConfig(
        processed_root=root, output_root=tmp_path / 'output', expected_samples=2))
    processor.process()
    frames = list(FoxgloveMcapReader(processor.outputs['152'], include_lidar=True))
    assert len(frames) == 2
    for index, frame in enumerate(frames):
        expected = source[index]
        assert frame.metadata['frame_id'] == expected.frame_id
        assert frame.metadata['timestamp_us'] == expected.timestamp_us
        assert frame.metadata['scene_generation'] == 'nuscenes:152'
        np.testing.assert_array_equal(frame.image_bgr, expected.image)
        np.testing.assert_array_equal(frame.lidar_points, expected.lidar_points)
        np.testing.assert_allclose(np.asarray(frame.metadata['world_T_camera']).reshape(4, 4), expected.map_T_camera)
        np.testing.assert_allclose(np.asarray(frame.metadata['world_T_ego']).reshape(4, 4), expected.map_T_lidar)
        assert frame.vehicles[0].track_id == 'stable-car'
        assert frame.vehicles[0].text_label == 'vehicle.car'


def test_failed_export_does_not_replace_completed_file(tmp_path):
    import pytest

    _write_scene(tmp_path / '152', 1)
    source = ProcessedNuScenesSequence(tmp_path / '152', expected_frame_count=1)
    invalid = replace(source[0], lidar_points=np.zeros((3, 2)))

    class Sequence:
        timestamp_step_us = source.timestamp_step_us
        map_T_global = source.map_T_global
        gaussian_T_nuscenes_global = source.gaussian_T_nuscenes_global
        def __getitem__(self, index):
            if index != 0:
                raise IndexError(index)
            return invalid

    output = tmp_path / 'scene.mcap'
    output.write_bytes(b'existing-completed-scene')
    with pytest.raises(ValueError, match='LiDAR must have shape'):
        NuScenesFoxgloveWriter().write(output, Sequence(), scene_generation='test')
    assert output.read_bytes() == b'existing-completed-scene'
