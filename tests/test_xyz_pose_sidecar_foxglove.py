"""Exercise sidecar record branches and selected-frame playback timing."""

import copy
from types import SimpleNamespace

import pytest

from test_xyz_pose_sidecar import fixture
from src.data.xyz_pose_sidecar import VerifiedXYZPoseSidecar
from src.data.cooperscene_mcap import _serialize_image, _serialize_metadata
from src.deployment.ros_frame_adapter import (
    McapFrameReplay, RosAnalysisAssembler, RosFrameAdapter, RosFrameAdapterConfig,
)
from src.deployment.vehicle import VehicleAnalysisSpool, VehicleFrameSpool
from src.realtime.protocol import FrameDetections


@pytest.mark.parametrize("failure", [None, "missing_lidar", "wrong_extrinsic"])
def test_foxglove_sidecar_exact_range_and_contained_lidar(
    fixture, tmp_path, monkeypatch, failure,
):
    path, sidecar_path, _, metadata, image, points = fixture
    frames = []
    for index in (6, 7, 8, 9):
        value = copy.deepcopy(metadata)
        value.update(frame_id=str(100 + index), timestamp_us=index * 100_000)
        if failure == "wrong_extrinsic" and index == 7:
            value["world_T_camera"][3] += 0.1
        frames.append(SimpleNamespace(
            sequence=index, timestamp_ns=index * 100_000_000,
            metadata=value, image_bgr=image,
            lidar_points=None if failure == "missing_lidar" else points,
        ))

    def foxglove_reader(source, **options):
        assert source == path
        # Sidecars require the recording's cloud even when no detector is attached.
        assert options["include_lidar"] is True
        return iter(frames)

    monkeypatch.setattr("src.data.foxglove_mcap.FoxgloveMcapReader", foxglove_reader)

    class Reader:
        def get_summary(self):
            return SimpleNamespace(schemas={1: SimpleNamespace(name="foxglove.CompressedImage")})

    class Publisher:
        def send(self, *args, **kwargs):
            pass

    analysis = VehicleAnalysisSpool(tmp_path / "analysis")
    assembly = RosAnalysisAssembler(analysis)
    snapshots = VehicleFrameSpool(tmp_path / "snapshots")
    adapter = RosFrameAdapter(
        Publisher(), RosFrameAdapterConfig(),
        analysis_assembler=assembly, snapshot_sink=snapshots,
    )
    replay = McapFrameReplay(
        adapter, image_topic="/image", metadata_topic="/metadata",
        expected_samples=2, agent="2", start_sequence=7,
        xyz_pose_sidecar=VerifiedXYZPoseSidecar(sidecar_path),
        reader_factory=lambda stream: Reader(),
        sleep=lambda seconds: None, monotonic=lambda: 0.0,
    )
    try:
        if failure:
            with pytest.raises(ValueError, match="contained LiDAR|extrinsic mismatch"):
                replay.replay(path)
        else:
            replay.replay(path)
    finally:
        assembly.close()

    if failure is None:
        for ordinal, timestamp in enumerate((1_000_000, 1_100_003)):
            frame = analysis.get("scene", ordinal)
            assert frame.frame_id == str(107 + ordinal)
            assert frame.timestamp_us == timestamp
            assert frame.boxes == ()
            assert frame.end_of_scene == (ordinal == 1)
            assert snapshots.get("scene", ordinal).timestamp_us == timestamp
        assert assembly._written == 2


def test_ros_selected_clock_starts_after_reading_skipped_prefix(fixture, tmp_path):
    path, sidecar_path, _, metadata, image, _ = fixture

    class Clock:
        now = 0.0

        def sleep(self, seconds):
            self.now += seconds

    clock = Clock()
    deliveries = []

    class Reader:
        def get_summary(self):
            return None

        def iter_messages(self, **options):
            # Model a costly read of records before the qualified start.
            clock.now = 2.0
            for index in (6, 7, 8):
                timestamp = index * 100_000
                value = FrameDetections(
                    frame_id=str(100 + index), request_id=f"old:{index}",
                    scene_generation="scene", timestamp_us=timestamp,
                    image_jpeg=b"unused", camera_intrinsic=metadata["camera_intrinsic"],
                    world_T_camera=metadata["world_T_camera"],
                    world_T_ego=metadata["world_T_ego"], boxes=(),
                )
                records = (
                    ("/image", "sensor_msgs/msg/Image", _serialize_image(
                        timestamp_ns=timestamp * 1000, frame_id="camera", height=1,
                        width=1, data=image.tobytes(),
                    )),
                    ("/metadata", "std_msgs/msg/String", _serialize_metadata(value)),
                )
                for topic, schema, data in records:
                    yield (
                        SimpleNamespace(name=schema),
                        SimpleNamespace(topic=topic, message_encoding="cdr"),
                        SimpleNamespace(sequence=index, log_time=timestamp * 1000, data=data),
                    )

    class Publisher:
        def send(self, message, **options):
            deliveries.append(clock.now)

    analysis = VehicleAnalysisSpool(tmp_path / "analysis")
    assembly = RosAnalysisAssembler(analysis)
    adapter = RosFrameAdapter(Publisher(), RosFrameAdapterConfig(), analysis_assembler=assembly)
    try:
        McapFrameReplay(
            adapter, image_topic="/image", metadata_topic="/metadata",
            expected_samples=2, agent="2", start_sequence=7,
            xyz_pose_sidecar=VerifiedXYZPoseSidecar(sidecar_path),
            reader_factory=lambda stream: Reader(), sleep=clock.sleep,
            monotonic=lambda: clock.now,
        ).replay(path)
    finally:
        assembly.close()
    # Each capture emits a pose and object message. The 0.1s interval survives.
    assert min(deliveries) == 2.0
    assert max(deliveries) == pytest.approx(2.1)
