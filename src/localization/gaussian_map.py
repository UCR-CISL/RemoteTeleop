"""Read SPZ v3 and standard binary little-endian 3DGS Gaussian maps.

SPZ stores its native values in the RUB coordinate system.  This module leaves
those coordinates unchanged; choosing and applying a map-frame conversion is a
caller responsibility.
"""

from __future__ import annotations

from dataclasses import dataclass
import gzip
from pathlib import Path
import struct
from typing import BinaryIO

import numpy as np
from numpy.typing import NDArray


_MAGIC = b"NGSP"
_VERSION = 3
_HEADER_SIZE = 16
_COLOR_SCALE = np.float32(0.28209479177387814)
_SQRT_HALF = np.float32(0.7071067811865476)
_PLY_HEADER_LIMIT = 1 << 20
_PLY_TYPES = {
    "char": "i1",
    "uchar": "u1",
    "short": "<i2",
    "ushort": "<u2",
    "int": "<i4",
    "uint": "<u4",
    "float": "<f4",
    "double": "<f8",
    "int8": "i1",
    "uint8": "u1",
    "int16": "<i2",
    "uint16": "<u2",
    "int32": "<i4",
    "uint32": "<u4",
    "float32": "<f4",
    "float64": "<f8",
}


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


def load_gaussian_map(path: str | Path) -> GaussianMap:
    """Load a Gaussian map based on its filename extension."""

    path = Path(path)
    suffix = path.suffix.lower()
    if suffix == ".spz":
        return load_spz_v3(path)
    if suffix == ".ply":
        return load_ply_3dgs(path)
    raise ValueError(f"unsupported Gaussian-map extension: {path.suffix or '<none>'}")


def load_ply_3dgs(path: str | Path) -> GaussianMap:
    """Load a binary little-endian PLY using the standard 3DGS schema.

    The ``f_rest_*`` coefficients are expected in the channel-major order used
    by the original 3D Gaussian Splatting PLY exporter.
    """

    path = Path(path)
    with path.open("rb") as stream:
        properties, vertex_count, data_offset = _read_ply_header(stream)

    required = [
        "x",
        "y",
        "z",
        "f_dc_0",
        "f_dc_1",
        "f_dc_2",
        *(f"f_rest_{index}" for index in range(45)),
        "opacity",
        "scale_0",
        "scale_1",
        "scale_2",
        "rot_0",
        "rot_1",
        "rot_2",
        "rot_3",
    ]
    property_types = {name: type_name for type_name, name in properties}
    missing = [name for name in required if name not in property_types]
    if missing:
        raise ValueError(f"PLY is missing required 3DGS properties: {', '.join(missing)}")
    invalid_types = [
        name for name in required if property_types[name] not in ("float", "float32")
    ]
    if invalid_types:
        raise ValueError(
            "3DGS PLY properties must be float32; invalid properties: "
            + ", ".join(invalid_types)
        )

    dtype = np.dtype([(name, _PLY_TYPES[type_name]) for type_name, name in properties])
    expected_size = data_offset + vertex_count * dtype.itemsize
    actual_size = path.stat().st_size
    if actual_size != expected_size:
        raise ValueError(
            f"PLY payload size mismatch: expected {expected_size} bytes, got {actual_size}"
        )

    vertices = np.fromfile(path, dtype=dtype, count=vertex_count, offset=data_offset)
    if len(vertices) != vertex_count:
        raise ValueError(f"could not read all {vertex_count} PLY vertices")

    means = _stack_ply_properties(vertices, ("x", "y", "z"))
    dc = _stack_ply_properties(vertices, ("f_dc_0", "f_dc_1", "f_dc_2"))[:, None, :]
    rest = _stack_ply_properties(vertices, tuple(f"f_rest_{index}" for index in range(45)))
    rest = rest.reshape(vertex_count, 3, 15).transpose(0, 2, 1)
    sh_coeffs = np.concatenate((dc, rest), axis=1).astype(np.float32, copy=False)

    opacity_logits = vertices["opacity"].astype(np.float32)
    opacities = np.empty_like(opacity_logits)
    positive = opacity_logits >= 0
    opacities[positive] = 1.0 / (1.0 + np.exp(-opacity_logits[positive]))
    exponent = np.exp(opacity_logits[~positive])
    opacities[~positive] = exponent / (1.0 + exponent)

    log_scales = _stack_ply_properties(vertices, ("scale_0", "scale_1", "scale_2"))
    scales = np.exp(log_scales).astype(np.float32)
    quats = _stack_ply_properties(vertices, ("rot_0", "rot_1", "rot_2", "rot_3"))
    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    if np.any(~np.isfinite(norms)) or np.any(norms == 0):
        raise ValueError("PLY contains a non-finite or zero-length quaternion")
    quats = (quats / norms).astype(np.float32)

    arrays = (means, scales, opacities, sh_coeffs)
    if any(np.any(~np.isfinite(array)) for array in arrays):
        raise ValueError("PLY contains non-finite Gaussian attributes")

    return GaussianMap(
        means=means,
        quats=quats,
        scales=scales,
        opacities=opacities,
        sh_coeffs=sh_coeffs,
        sh_degree=3,
        antialiased=False,
    )


def _read_ply_header(stream: BinaryIO) -> tuple[list[tuple[str, str]], int, int]:
    first_line = stream.readline()
    if first_line != b"ply\n" and first_line != b"ply\r\n":
        raise ValueError("PLY file does not start with the ply magic line")

    properties: list[tuple[str, str]] = []
    vertex_count: int | None = None
    current_element: str | None = None
    saw_format = False
    while stream.tell() <= _PLY_HEADER_LIMIT:
        raw_line = stream.readline()
        if not raw_line:
            raise ValueError("PLY header is missing end_header")
        try:
            tokens = raw_line.decode("ascii").strip().split()
        except UnicodeDecodeError as error:
            raise ValueError("PLY header is not ASCII") from error
        if not tokens or tokens[0] in ("comment", "obj_info"):
            continue
        if tokens == ["end_header"]:
            break
        if tokens[0] == "format":
            if tokens != ["format", "binary_little_endian", "1.0"] or saw_format:
                raise ValueError("PLY must use binary_little_endian format 1.0 exactly once")
            saw_format = True
        elif tokens[0] == "element":
            if len(tokens) != 3:
                raise ValueError("malformed PLY element declaration")
            if tokens[1] != "vertex" or vertex_count is not None:
                raise ValueError("3DGS PLY must contain exactly one vertex element")
            try:
                vertex_count = int(tokens[2])
            except ValueError as error:
                raise ValueError("PLY vertex count is not an integer") from error
            if vertex_count < 0:
                raise ValueError("PLY vertex count cannot be negative")
            current_element = "vertex"
        elif tokens[0] == "property":
            if current_element != "vertex" or len(tokens) != 3:
                raise ValueError("3DGS PLY supports only scalar vertex properties")
            type_name, name = tokens[1:]
            if type_name not in _PLY_TYPES:
                raise ValueError(f"unsupported PLY property type: {type_name}")
            if any(existing_name == name for _, existing_name in properties):
                raise ValueError(f"duplicate PLY property: {name}")
            properties.append((type_name, name))
        else:
            raise ValueError(f"unsupported PLY header directive: {tokens[0]}")
    else:
        raise ValueError(f"PLY header exceeds {_PLY_HEADER_LIMIT} bytes")

    if not saw_format:
        raise ValueError("PLY header is missing its format declaration")
    if vertex_count is None:
        raise ValueError("PLY header is missing its vertex element")
    if not properties:
        raise ValueError("PLY vertex element has no properties")
    return properties, vertex_count, stream.tell()


def _stack_ply_properties(
    vertices: NDArray, names: tuple[str, ...]
) -> NDArray[np.float32]:
    return np.stack([vertices[name] for name in names], axis=1).astype(np.float32)


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
