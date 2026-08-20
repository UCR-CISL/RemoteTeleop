import hashlib
from dataclasses import replace

import cv2
import numpy as np
import zmq

from src.deployment.remote import RemoteAssetCache, ResumableRemoteAssetCache
from src.deployment.vehicle import AssetStorePublisher, VehicleAnalysisSpool, VehicleAssetSession, VehicleAssetStore, VehicleFrameSession, VehicleFrameSpool, publish_mesh_asset
from src.realtime.mask_process import protocol_frame_to_prompt
from src.realtime.protocol import AssetChunk, AssetEvent, AssetFetch, AssetManifest, AssetState, AssetSync, BoxPrompt, FrameAck, FrameDetections, FrameEnd, FrameHello, FramePending, FrameSnapshot


def _frame(label: str = "car") -> FrameDetections:
    success, jpeg = cv2.imencode(".jpg", np.zeros((12, 16, 3), dtype=np.uint8))
    assert success
    return FrameDetections(
        frame_id="frame-1",
        request_id="request-1",
        scene_generation="take-1",
        timestamp_us=1,
        image_jpeg=jpeg.tobytes(),
        camera_intrinsic=(1, 0, 8, 0, 1, 6, 0, 0, 1),
        world_T_camera=(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
        world_T_ego=(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
        boxes=(BoxPrompt("car-1", (2, 2, 12, 10), (4, 2, 2), (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1), text_label=label),),
    )


def test_vehicle_prompt_preserves_projected_box_and_text_label():
    prompt = protocol_frame_to_prompt(_frame())

    assert prompt.boxes[0].xyxy == (2, 2, 12, 10)
    assert prompt.boxes[0].text_label == "car"


def test_remote_cache_materializes_verified_payload_with_original_suffix(tmp_path):
    payload = b"ply-data"
    event = AssetEvent(
        request_id="take-1:car-1",
        scene_generation="take-1",
        track_id="car-1",
        state=AssetState.READY,
        asset_id="take-1:car-1",
        content_sha256=hashlib.sha256(payload).hexdigest(),
        mesh_payload=payload,
        mesh_suffix=".ply",
    )

    local = RemoteAssetCache(tmp_path).receive(event)

    assert local.aligned_mesh_path is not None
    assert local.aligned_mesh_path.suffix == ".ply"
    assert local.aligned_mesh_path.read_bytes() == payload


def test_remote_cache_preserves_mesh_source_alignment_and_repairs_partial_file(tmp_path):
    payload = b"verified-mesh"
    event = AssetEvent(
        request_id="take-1:car-1", scene_generation="take-1", track_id="car-1",
        state=AssetState.READY, asset_id="take-1:car-1",
        content_sha256=hashlib.sha256(payload).hexdigest(), mesh_payload=payload,
        source_sequence=12, source_timestamp_us=1_200_000, source_frame_id="frame-12",
    )
    cache = RemoteAssetCache(tmp_path)
    first = cache.receive(event)
    assert first.aligned_mesh_path is not None
    first.aligned_mesh_path.write_bytes(b"truncated")

    repaired = cache.receive(event)

    assert repaired.aligned_mesh_path.read_bytes() == payload
    assert (repaired.source_sequence, repaired.source_timestamp_us, repaired.source_frame_id) == (
        12, 1_200_000, "frame-12",
    )


def test_mesh_publication_turns_vehicle_path_into_hashed_payload(tmp_path):
    mesh = tmp_path / "car.ply"
    mesh.write_bytes(b"mesh")
    sent = []

    class Publisher:
        def send(self, message, *, topic=None):
            sent.append(message)

    event = publish_mesh_asset(
        AssetEvent("take-1:car-1", "take-1", "car-1", AssetState.READY, mesh),
        Publisher(),
    )

    assert sent == [event]
    assert event.mesh_payload == b"mesh"
    assert event.mesh_suffix == ".ply"


def test_vehicle_spools_exact_snapshot_and_serves_bounded_mesh_chunks(tmp_path):
    snapshot = FrameSnapshot(
        sequence=7, timestamp_us=700, scene_generation="take-1", frame_id="frame-7",
        world_T_ego=(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
    )
    spool = VehicleFrameSpool(tmp_path / "frames")
    spool.append(snapshot)
    spool.append(snapshot)
    assert spool.get("take-1", 7) == snapshot

    mesh = tmp_path / "car.ply"
    mesh.write_bytes(b"abcdefgh")
    manifest = VehicleAssetStore(tmp_path / "assets", chunk_bytes=3).publish(AssetEvent(
        "take-1:car", "take-1", "car", AssetState.READY, mesh,
        source_sequence=7, source_timestamp_us=700, source_frame_id="frame-7",
    ))
    chunk = VehicleAssetStore(tmp_path / "assets", chunk_bytes=3).fetch(
        manifest, AssetFetch(manifest.asset_id, 1, manifest.content_sha256)
    )
    assert chunk is not None and chunk.payload == b"abc"


def test_vehicle_analysis_spool_is_image_bearing_local_and_strict_pull(tmp_path):
    frame = _frame()
    frame = replace(frame, source_sequence=0)
    spool = VehicleAnalysisSpool(tmp_path / "analysis")
    spool.append(frame)
    spool.append(frame)
    spool.finalize(FrameEnd("take-1", 0, 1))
    assert spool.get("take-1", 0) == frame

    class Router:
        def __init__(self): self.incoming = []; self.sent = []
        def receive(self): return self.incoming.pop(0)
        def send(self, identity, message, *, topic=None): self.sent.append((identity, message))

    router = Router()
    session = VehicleFrameSession(router, spool)
    router.incoming.append((b"sam3", "", FrameHello("take-1", "sam3", 0)))
    session.serve_once()
    assert router.sent == [(b"sam3", frame)]
    router.incoming.append((b"sam3", "", FrameAck("take-1", 0, frame.frame_id, frame.sha256)))
    session.serve_once()
    router.incoming.append((b"sam3", "", FrameHello("take-1", "sam3", 1)))
    session.serve_once()
    assert router.sent[-1][1] == FrameEnd("take-1", 0, 1)


def test_asset_store_publisher_durably_records_ready_event_only(tmp_path):
    mesh = tmp_path / "car.glb"
    mesh.write_bytes(b"mesh")
    store = VehicleAssetStore(tmp_path / "assets")
    publisher = AssetStorePublisher(store)
    ready = AssetEvent(
        "request", "take", "car", AssetState.READY, mesh,
        source_sequence=1, source_timestamp_us=1, source_frame_id="f1",
    )
    publisher.send(ready)
    publisher.send(AssetEvent("request", "take", "car", AssetState.RUNNING))

    assert len(store.manifests("take")) == 1
    assert publisher.lifecycle[-1].state is AssetState.RUNNING


def test_frame_session_is_strict_pull_and_recovers_after_lost_ack(tmp_path):
    class Router:
        def __init__(self): self.incoming = []; self.sent = []
        def receive(self): return self.incoming.pop(0)
        def send(self, identity, message, *, topic=None): self.sent.append((identity, message))

    spool = VehicleFrameSpool(tmp_path / "frames")
    identity = b"remote-a"
    snapshots = [FrameSnapshot(
        sequence=index, timestamp_us=index, scene_generation="take", frame_id=f"f{index}",
        world_T_ego=(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
    ) for index in range(2)]
    for snapshot in snapshots: spool.append(snapshot)
    router = Router()
    session = VehicleFrameSession(router, spool)

    router.incoming.append((identity, "", FrameHello("take", "remote-a", 0)))
    session.serve_once()
    assert router.sent == [(identity, snapshots[0])]
    router.incoming.append((identity, "", FrameAck("take", 0, "f0", snapshots[0].sha256)))
    session.serve_once()
    assert router.sent == [(identity, snapshots[0])]  # ACK never pushes frame 1.
    router.incoming.append((identity, "", FrameHello("take", "remote-a", 1)))
    session.serve_once()
    assert router.sent[-1] == (identity, snapshots[1])
    router.incoming.append((identity, "", FrameHello("take", "remote-a", 1)))
    session.serve_once()
    assert router.sent[-1] == (identity, snapshots[1])


def test_router_disconnect_after_request_is_resumable(tmp_path):
    class Router:
        def __init__(self): self.incoming = []; self.sent = []; self.disconnect_once = True
        def receive(self): return self.incoming.pop(0)
        def send(self, identity, message, *, topic=None):
            if self.disconnect_once:
                self.disconnect_once = False
                raise zmq.ZMQError(zmq.EHOSTUNREACH)
            self.sent.append((identity, message))

    snapshot = FrameSnapshot(
        0, 0, "take", "f0", (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
    )
    spool = VehicleFrameSpool(tmp_path / "frames")
    spool.append(snapshot)
    router = Router()
    session = VehicleFrameSession(router, spool)
    router.incoming.append((b"remote", "", FrameHello("take", "remote", 0)))
    session.serve_once()  # EHOSTUNREACH is a reconnect condition, not fatal.
    router.incoming.append((b"remote", "", FrameHello("take", "remote", 0)))
    session.serve_once()
    assert router.sent == [(b"remote", snapshot)]


def test_missing_snapshot_replies_pending_until_one_pull_after_spool(tmp_path):
    class Router:
        def __init__(self): self.incoming = []; self.sent = []
        def receive(self): return self.incoming.pop(0)
        def send(self, identity, message, *, topic=None): self.sent.append((identity, message))

    router = Router()
    spool = VehicleFrameSpool(tmp_path / "frames")
    session = VehicleFrameSession(router, spool)
    hello = FrameHello("take", "remote", 4)
    router.incoming.extend([(b"remote", "", hello)] * 3)
    for _ in range(3): session.serve_once()
    assert [message for _, message in router.sent] == [FramePending("take", 4)] * 3
    snapshot = FrameSnapshot(4, 4, "take", "f4", (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1))
    spool.append(snapshot)
    router.incoming.append((b"remote", "", hello))
    session.serve_once()
    assert [message for _, message in router.sent].count(snapshot) == 1


def test_spool_end_is_a_durable_response_after_final_snapshot(tmp_path):
    class Router:
        def __init__(self): self.incoming = []; self.sent = []
        def receive(self): return self.incoming.pop(0)
        def send(self, identity, message, *, topic=None): self.sent.append((identity, message))
    snapshot = FrameSnapshot(0, 9, "take", "f0", (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1))
    spool = VehicleFrameSpool(tmp_path / "frames")
    spool.append(snapshot)
    end = FrameEnd("take", 0, 9)
    spool.finalize(end)
    router = Router(); session = VehicleFrameSession(router, spool)
    router.incoming.append((b"remote", "", FrameHello("take", "remote", 1)))
    session.serve_once()
    assert router.sent == [(b"remote", end)]
def test_asset_sync_reannounces_manifest_for_cached_mesh_after_restart(tmp_path):
    class Router:
        def __init__(self): self.incoming = []; self.sent = []
        def receive(self): return self.incoming.pop(0)
        def send(self, identity, message, *, topic=None): self.sent.append((identity, message))

    mesh = tmp_path / "car.ply"
    mesh.write_bytes(b"cached")
    store = VehicleAssetStore(tmp_path / "assets")
    manifest = store.publish(AssetEvent(
        "request", "take", "car", AssetState.READY, mesh,
        source_sequence=2, source_timestamp_us=2, source_frame_id="f2",
    ))
    router = Router()
    session = VehicleAssetSession(router, store)  # Empty in-memory map: restart.
    router.incoming.append((b"remote", "", AssetSync("remote", "take", (manifest.content_sha256,))))
    session.serve_once()

    assert router.sent == [(b"remote", manifest)]
    assert (manifest.asset_id, manifest.asset_version, manifest.content_sha256) not in session.committed.get(b"remote", set())


def test_asset_sync_disconnect_does_not_lose_later_manifests(tmp_path):
    class Router:
        def __init__(self): self.incoming = []; self.sent = []; self.disconnect_once = True
        def receive(self): return self.incoming.pop(0)
        def send(self, identity, message, *, topic=None):
            if self.disconnect_once:
                self.disconnect_once = False
                raise zmq.ZMQError(zmq.EHOSTUNREACH)
            self.sent.append((identity, message))

    store = VehicleAssetStore(tmp_path / "assets")
    for track in ("a", "b"):
        mesh = tmp_path / f"{track}.glb"
        mesh.write_bytes(track.encode())
        store.publish(AssetEvent(
            f"request-{track}", "take", track, AssetState.READY, mesh,
            source_sequence=1, source_timestamp_us=1, source_frame_id="f1",
        ))
    router = Router()
    session = VehicleAssetSession(router, store)
    sync = AssetSync("remote", "take")
    router.incoming.append((b"remote", "", sync))
    session.serve_once()
    router.incoming.append((b"remote", "", sync))
    session.serve_once()
    assert {message.track_id for _, message in router.sent} == {"a", "b"}

def test_resumable_cache_commits_only_after_complete_verified_chunks(tmp_path):
    payload = b"0123456789"
    manifest = AssetManifest(
        scene_generation="take-1", track_id="car-1", request_id="request-1",
        source_sequence=3, source_timestamp_us=300_000, source_frame_id="frame-3",
        asset_id="take-1:car-1", asset_version=1, mesh_suffix=".glb",
        byte_length=len(payload), content_sha256=hashlib.sha256(payload).hexdigest(),
    )
    cache = ResumableRemoteAssetCache(tmp_path)
    assert cache.begin(manifest) == 0
    assert cache.append(AssetChunk(manifest.asset_id, 1, manifest.content_sha256, 0, payload[:4])) is None
    assert cache.begin(manifest) == 4

    local = cache.append(AssetChunk(manifest.asset_id, 1, manifest.content_sha256, 4, payload[4:]))

    assert local is not None
    assert local.aligned_mesh_path.read_bytes() == payload
    assert not local.aligned_mesh_path.with_suffix(".glb.partial").exists()


def test_resumable_cache_rejects_corrupt_completed_asset_without_activation(tmp_path):
    expected = b"expected"
    manifest = AssetManifest(
        scene_generation="take-1", track_id="car-1", request_id="request-1",
        source_sequence=3, source_timestamp_us=300_000, source_frame_id="frame-3",
        asset_id="take-1:car-1", asset_version=1, mesh_suffix=".glb",
        byte_length=len(expected), content_sha256=hashlib.sha256(expected).hexdigest(),
    )
    cache = ResumableRemoteAssetCache(tmp_path)
    cache.begin(manifest)

    with np.testing.assert_raises_regex(ValueError, "SHA-256 mismatch"):
        cache.append(AssetChunk(manifest.asset_id, 1, manifest.content_sha256, 0, b"corrupt!"))

    assert not list(tmp_path.rglob("*.glb"))


def test_resumable_cache_handles_many_chunks_and_discovers_committed_asset(tmp_path):
    payload = b"012345678"
    manifest = AssetManifest(
        scene_generation="take-1", track_id="car-1", request_id="request-1",
        source_sequence=3, source_timestamp_us=300_000, source_frame_id="frame-3",
        asset_id="take-1:car-1", asset_version=1, mesh_suffix=".glb",
        byte_length=len(payload), content_sha256=hashlib.sha256(payload).hexdigest(),
    )
    cache = ResumableRemoteAssetCache(tmp_path)
    assert cache.begin(manifest) == 0
    for offset in (0, 3, 6):
        result = cache.append(AssetChunk(manifest.asset_id, 1, manifest.content_sha256, offset, payload[offset:offset + 3]))
    assert result is not None

    restarted = ResumableRemoteAssetCache(tmp_path)
    assert restarted.known_hashes() == (manifest.content_sha256,)
    assert restarted.begin(manifest) == len(payload)
    assert restarted.committed(manifest) is not None
