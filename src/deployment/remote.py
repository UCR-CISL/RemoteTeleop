"""Remote-side mesh receipt/cache and GSplat composition adapter."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re

from src.realtime.protocol import AssetChunk, AssetEvent, AssetManifest, AssetState, FrameDetections
from src.viz.composited_camera import CompositedCameraBackend, CompositedCameraFrame


_SAFE_COMPONENT = re.compile(r"[^A-Za-z0-9_.-]+")


class RemoteAssetCache:
    """Materialize verified vehicle mesh payloads into renderer-local paths."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._assets: dict[tuple[str, str, int], Path] = {}

    def receive(self, event: AssetEvent) -> AssetEvent:
        if event.state is not AssetState.READY or event.mesh_payload is None:
            return event
        assert event.content_sha256 is not None
        if hashlib.sha256(event.mesh_payload).hexdigest() != event.content_sha256:
            return AssetEvent(
                request_id=event.request_id,
                scene_generation=event.scene_generation,
                track_id=event.track_id,
                state=AssetState.FAILED,
                asset_id=event.asset_id,
                asset_version=event.asset_version,
                error="mesh payload SHA-256 mismatch",
            )
        asset_id = event.asset_id or f"{event.scene_generation}:{event.track_id}"
        key = (event.scene_generation, asset_id, event.asset_version)
        path = self.root / _safe(event.scene_generation) / (
            f"{_safe(asset_id)}-v{event.asset_version}-{event.content_sha256}{event.mesh_suffix}"
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.is_file() or _file_sha256(path) != event.content_sha256:
            temporary = path.with_suffix(path.suffix + ".partial")
            with temporary.open("wb") as stream:
                stream.write(event.mesh_payload)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            _fsync_directory(path.parent)
        self._assets[key] = path
        return AssetEvent(
            request_id=event.request_id,
            scene_generation=event.scene_generation,
            track_id=event.track_id,
            state=AssetState.READY,
            aligned_mesh_path=path,
            asset_id=asset_id,
            asset_version=event.asset_version,
            content_sha256=event.content_sha256,
            mesh_suffix=event.mesh_suffix,
            metrics=event.metrics,
            source_sequence=event.source_sequence,
            source_timestamp_us=event.source_timestamp_us,
            source_frame_id=event.source_frame_id,
        )


class ResumableRemoteAssetCache:
    """Durably assemble chunked mesh assets before exposing renderer paths."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        self._manifests: dict[tuple[str, int, str], AssetManifest] = {}

    def begin(self, manifest: AssetManifest) -> int:
        key = (manifest.asset_id, manifest.asset_version, manifest.content_sha256)
        self._manifests[key] = manifest
        final = self._path(manifest)
        if final.is_file() and final.stat().st_size == manifest.byte_length and _file_sha256(final) == manifest.content_sha256:
            return manifest.byte_length
        partial = self._partial_path(manifest)
        partial.parent.mkdir(parents=True, exist_ok=True)
        size = partial.stat().st_size if partial.is_file() else 0
        if size > manifest.byte_length:
            partial.unlink()
            return 0
        return size

    def known_hashes(self) -> tuple[str, ...]:
        known: set[str] = set()
        for path in self.root.rglob("*") if self.root.is_dir() else ():
            if not path.is_file() or ".partial" in path.name:
                continue
            match = re.search(r"-([0-9a-f]{64})\.[A-Za-z0-9]{1,15}$", path.name)
            if match and _file_sha256(path) == match.group(1):
                known.add(match.group(1))
        return tuple(sorted(known))

    def committed(self, manifest: AssetManifest) -> AssetEvent | None:
        path = self._path(manifest)
        if not path.is_file() or path.stat().st_size != manifest.byte_length:
            return None
        if _file_sha256(path) != manifest.content_sha256:
            return None
        return self._event(manifest, path)

    def append(self, chunk: AssetChunk) -> AssetEvent | None:
        key = (chunk.asset_id, chunk.asset_version, chunk.content_sha256)
        manifest = self._manifests.get(key)
        if manifest is None:
            raise ValueError("asset chunk arrived without a manifest")
        partial = self._partial_path(manifest)
        current = partial.stat().st_size if partial.is_file() else 0
        if chunk.offset != current:
            raise ValueError(f"asset chunk offset {chunk.offset} does not match local offset {current}")
        if current + len(chunk.payload) > manifest.byte_length:
            raise ValueError("asset chunk exceeds manifest byte length")
        with partial.open("ab") as stream:
            stream.write(chunk.payload)
            stream.flush()
            os.fsync(stream.fileno())
        if current + len(chunk.payload) != manifest.byte_length:
            return None
        payload_digest = _file_sha256(partial)
        if payload_digest != manifest.content_sha256:
            partial.unlink()
            raise ValueError("assembled mesh SHA-256 mismatch")
        path = self._path(manifest)
        os.replace(partial, path)
        _fsync_directory(path.parent)
        return self._event(manifest, path)

    @staticmethod
    def _event(manifest: AssetManifest, path: Path) -> AssetEvent:
        return AssetEvent(
            request_id=manifest.request_id,
            scene_generation=manifest.scene_generation,
            track_id=manifest.track_id,
            state=AssetState.READY,
            aligned_mesh_path=path,
            asset_id=manifest.asset_id,
            asset_version=manifest.asset_version,
            content_sha256=manifest.content_sha256,
            mesh_suffix=manifest.mesh_suffix,
            source_sequence=manifest.source_sequence,
            source_timestamp_us=manifest.source_timestamp_us,
            source_frame_id=manifest.source_frame_id,
        )

    def next_offset(self, asset_id: str, asset_version: int, content_sha256: str) -> int:
        manifest = self._manifests.get((asset_id, asset_version, content_sha256))
        if manifest is None:
            raise ValueError("asset offset requested without a manifest")
        partial = self._partial_path(manifest)
        return partial.stat().st_size if partial.is_file() else manifest.byte_length

    def _path(self, manifest: AssetManifest) -> Path:
        return self.root / _safe(manifest.scene_generation) / (
            f"{_safe(manifest.asset_id)}-v{manifest.asset_version}-{manifest.content_sha256}{manifest.mesh_suffix}"
        )

    def _partial_path(self, manifest: AssetManifest) -> Path:
        return self._path(manifest).with_suffix(manifest.mesh_suffix + ".partial")


class RemoteCompositionRuntime:
    """Compose each received frame immediately; meshes are an optional cache hit."""

    def __init__(self, backend: CompositedCameraBackend, cache: RemoteAssetCache) -> None:
        self.backend = backend
        self.cache = cache

    def handle_asset(self, event: AssetEvent) -> None:
        self.backend.handle_asset_event(self.cache.receive(event))

    def render(self, frame: FrameDetections) -> CompositedCameraFrame:
        return self.backend.render_frame(frame)


def _safe(value: str) -> str:
    return _SAFE_COMPONENT.sub("_", value).strip("._")[:120] or "asset"


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
