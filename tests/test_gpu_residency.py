from __future__ import annotations

from src.realtime.gpu_residency import GpuResidencyLease


def test_residency_lease_excludes_peer_and_can_handoff(tmp_path):
    path = tmp_path / "cuda-model.lock"
    sam3 = GpuResidencyLease(path, owner="sam3-mask")
    sam3d = GpuResidencyLease(path, owner="sam3d-object")

    first = sam3.try_acquire()
    assert first is not None
    assert first.owner == "sam3-mask"
    assert sam3.held
    assert sam3d.try_acquire() is None

    sam3.release()
    second = sam3d.try_acquire()

    assert second is not None
    assert second.owner == "sam3d-object"
    assert second.wait_seconds >= 0.0
    assert path.read_text(encoding="utf-8").startswith("sam3d-object pid=")
    sam3d.close()


def test_residency_release_is_idempotent(tmp_path):
    lease = GpuResidencyLease(tmp_path / "cuda-model.lock", owner="sam3-mask")
    assert lease.try_acquire() is not None

    lease.release()
    lease.release()

    assert not lease.held
