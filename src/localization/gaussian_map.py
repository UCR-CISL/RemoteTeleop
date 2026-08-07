"""Read the legacy gzip-compressed SPZ v3 Gaussian-splat format.

SPZ stores its native values in the RUB coordinate system.  This module leaves
those coordinates unchanged; choosing and applying a map-frame conversion is a
caller responsibility.
"""

from __future__ import annotations

from dataclasses import dataclass
import gzip
from pathlib import Path
import struct

import numpy as np
from numpy.typing import NDArray


_MAGIC = b"NGSP"
_VERSION = 3
_HEADER_SIZE = 16
_COLOR_SCALE = np.float32(0.28209479177387814)
_SQRT_HALF = np.float32(0.7071067811865476)


@dataclass(frozen=True)
class GaussianMap:
    """Gaussian attributes in forms directly consumable by ``gsplat.rasterization``.

    ``quats`` uses gsplat's ``wxyz`` convention; SPZ stores quaternions as
    ``xyzw``.  ``scales`` are linear standard deviations, ``opacities`` are in
    ``[0, 1]``, and ``sh_coeffs`` has shape ``(N, (degree + 1) ** 2, 3)`` with
    the decoded DC coefficient at index zero.
    """

    means: NDArray[np.float32]
    quats: NDArray[np.float32]
    scales: NDArray[np.float32]
    opacities: NDArray[np.float32]
    sh_coeffs: NDArray[np.float32]
    sh_degree: int
    antialiased: bool


def load_spz_v3(path: str | Path) -> GaussianMap:
    """Load a legacy gzip-compressed SPZ version 3 file.

    Version 4 SPZ uses independently compressed Zstandard streams and is
    deliberately rejected here rather than being decoded incorrectly.
    """

    with gzip.open(path, "rb") as stream:
        payload = stream.read()
    return _decode_spz_v3(payload)


def _decode_spz_v3(payload: bytes) -> GaussianMap:
    if len(payload) < _HEADER_SIZE:
        raise ValueError("SPZ v3 payload is shorter than its 16-byte header")

    magic, version, num_points, sh_degree, fractional_bits, flags, reserved = struct.unpack_from(
        "<4sIIBBBB", payload
    )
    if magic != _MAGIC:
        raise ValueError("SPZ payload does not start with the NGSP magic")
    if version != _VERSION:
        raise ValueError(f"expected SPZ version 3, got version {version}")
    if sh_degree > 3:
        raise ValueError(f"SPZ v3 supports SH degrees 0 through 3, got {sh_degree}")
    if fractional_bits > 23:
        raise ValueError(f"invalid SPZ fractional-bit count: {fractional_bits}")
    if flags & ~0x01:
        raise ValueError(f"unsupported SPZ v3 flags: 0x{flags:02x}")
    if reserved:
        raise ValueError("SPZ v3 reserved header byte must be zero")

    sh_count = (sh_degree + 1) ** 2 - 1
    stream_sizes = (
        num_points * 9,  # signed fixed-point xyz, 3 bytes each
        num_points,  # alpha
        num_points * 3,  # SH DC / color
        num_points * 3,  # log-scale xyz
        num_points * 4,  # v3 smallest-three quaternion
        num_points * sh_count * 3,
    )
    expected_size = _HEADER_SIZE + sum(stream_sizes)
    if len(payload) != expected_size:
        raise ValueError(
            f"SPZ v3 payload size mismatch: expected {expected_size} bytes, got {len(payload)}"
        )

    offset = _HEADER_SIZE
    positions = _take(payload, offset, stream_sizes[0]).reshape(num_points, 3, 3)
    offset += stream_sizes[0]
    alpha = _take(payload, offset, stream_sizes[1])
    offset += stream_sizes[1]
    colors = _take(payload, offset, stream_sizes[2]).reshape(num_points, 3)
    offset += stream_sizes[2]
    packed_scales = _take(payload, offset, stream_sizes[3]).reshape(num_points, 3)
    offset += stream_sizes[3]
    packed_rotations = _take(payload, offset, stream_sizes[4]).reshape(num_points, 4)
    offset += stream_sizes[4]
    packed_sh = _take(payload, offset, stream_sizes[5]).reshape(num_points, sh_count, 3)

    means = _unpack_positions(positions, fractional_bits)
    rotations_xyzw = _unpack_smallest_three(packed_rotations)
    quats = rotations_xyzw[:, [3, 0, 1, 2]]
    scales = np.exp(packed_scales.astype(np.float32) / 16.0 - 10.0).astype(np.float32)
    opacities = alpha.astype(np.float32) / 255.0

    sh_coeffs = np.empty((num_points, sh_count + 1, 3), dtype=np.float32)
    sh_coeffs[:, 0, :] = (colors.astype(np.float32) / 255.0 - 0.5) / _COLOR_SCALE
    if sh_count:
        sh_coeffs[:, 1:, :] = (packed_sh.astype(np.float32) - 128.0) / 128.0

    return GaussianMap(
        means=means,
        quats=quats,
        scales=scales,
        opacities=opacities,
        sh_coeffs=sh_coeffs,
        sh_degree=sh_degree,
        antialiased=bool(flags & 0x01),
    )


def _take(payload: bytes, offset: int, size: int) -> NDArray[np.uint8]:
    return np.frombuffer(payload, dtype=np.uint8, count=size, offset=offset)


def _unpack_positions(packed: NDArray[np.uint8], fractional_bits: int) -> NDArray[np.float32]:
    fixed = (
        packed[..., 0].astype(np.int32)
        | (packed[..., 1].astype(np.int32) << 8)
        | (packed[..., 2].astype(np.int32) << 16)
    )
    fixed = np.where(fixed & 0x800000, fixed - (1 << 24), fixed)
    return (fixed.astype(np.float32) / (1 << fractional_bits)).astype(np.float32)


def _unpack_smallest_three(packed: NDArray[np.uint8]) -> NDArray[np.float32]:
    """Decode SPZ v3's 2-bit index plus three signed 10-bit components."""

    encoded = (
        packed[:, 0].astype(np.uint32)
        | (packed[:, 1].astype(np.uint32) << 8)
        | (packed[:, 2].astype(np.uint32) << 16)
        | (packed[:, 3].astype(np.uint32) << 24)
    )
    largest = encoded >> 30
    result = np.zeros((len(packed), 4), dtype=np.float32)
    component_index = np.zeros(len(packed), dtype=np.uint32)
    for shift in (20, 10, 0):
        code = (encoded >> shift) & 0x3FF
        values = (code & 0x1FF).astype(np.float32) / 511.0 * _SQRT_HALF
        values = np.where(code & 0x200, -values, values)
        insert = component_index + (component_index >= largest)
        result[np.arange(len(packed)), insert] = values
        component_index += 1
    remainder = np.maximum(0.0, 1.0 - np.sum(result * result, axis=1))
    result[np.arange(len(packed)), largest] = np.sqrt(remainder)
    return result
