import gzip
import struct

import numpy as np
import pytest

from src.localization.gaussian_map import (
    _decode_spz_v3,
    load_gaussian_map,
    load_ply_3dgs,
    load_spz_v3,
)


def _pack_position(value: float, fractional_bits: int) -> bytes:
    fixed = int(round(value * (1 << fractional_bits))) & 0xFFFFFF
    return fixed.to_bytes(3, "little")


def _pack_quat_xyzw(quat: np.ndarray) -> bytes:
    quat = np.asarray(quat, dtype=np.float64)
    quat /= np.linalg.norm(quat)
    largest = int(np.argmax(np.abs(quat)))
    negate = quat[largest] < 0
    encoded = largest
    for index, value in enumerate(quat):
        if index == largest:
            continue
        magnitude = round(511 * abs(value) / np.sqrt(0.5))
        code = (int((value < 0) ^ negate) << 9) | int(magnitude)
        encoded = (encoded << 10) | code
    return encoded.to_bytes(4, "little")


def _fixture_payload() -> bytes:
    n, degree, fractional_bits = 2, 1, 4
    positions = b"".join(
        _pack_position(value, fractional_bits)
        for point in ((1.5, -2.0, 0.25), (-3.5, 0.0, 4.0))
        for value in point
    )
    alphas = bytes((0, 255))
    colors = bytes((128, 255, 0, 64, 128, 192))
    scales = bytes((160, 144, 128, 80, 96, 112))
    rotations = _pack_quat_xyzw(np.array((0.0, 0.0, 0.0, 1.0))) + _pack_quat_xyzw(
        np.array((0.2, -0.3, 0.1, 0.92736185))
    )
    sh = bytes(range(128, 128 + n * 3 * (((degree + 1) ** 2) - 1)))
    header = struct.pack("<4sIIBBBB", b"NGSP", 3, n, degree, fractional_bits, 1, 0)
    return header + positions + alphas + colors + scales + rotations + sh


def test_load_spz_v3_decodes_all_gsplat_attributes(tmp_path):
    path = tmp_path / "tiny.spz"
    path.write_bytes(gzip.compress(_fixture_payload()))

    splats = load_spz_v3(path)

    np.testing.assert_allclose(splats.means, [[1.5, -2.0, 0.25], [-3.5, 0.0, 4.0]])
    np.testing.assert_allclose(splats.quats[0], [1.0, 0.0, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(splats.quats, axis=1), 1.0, atol=2e-3)
    np.testing.assert_allclose(splats.scales, np.exp(np.array([[0.0, -1.0, -2.0], [-5.0, -4.0, -3.0]])))
    np.testing.assert_allclose(splats.opacities, [0.0, 1.0])
    assert splats.sh_coeffs.shape == (2, 4, 3)
    np.testing.assert_allclose(
        splats.sh_coeffs[0, 0],
        (np.array([128.0, 255.0, 0.0]) / 255.0 - 0.5) / 0.28209479177387814,
        atol=1e-6,
    )
    np.testing.assert_allclose(splats.sh_coeffs[0, 1], [0.0, 1.0 / 128.0, 2.0 / 128.0])
    assert splats.sh_degree == 1
    assert splats.antialiased


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        (b"short", "shorter"),
        (_fixture_payload().replace(b"NGSP", b"NOPE", 1), "magic"),
        (_fixture_payload()[:4] + struct.pack("<I", 2) + _fixture_payload()[8:], "expected SPZ version 3"),
        (_fixture_payload()[:-1], "size mismatch"),
    ],
)
def test_spz_v3_validation(payload, message):
    with pytest.raises(ValueError, match=message):
        _decode_spz_v3(payload)


def _ply_fixture(*, property_type="float", quaternion=(2.0, 0.0, 0.0, 0.0)) -> bytes:
    names = [
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
    header = "\n".join(
        [
            "ply",
            "format binary_little_endian 1.0",
            "comment test fixture",
            "element vertex 2",
            *(f"property {property_type} {name}" for name in names),
            "end_header",
            "",
        ]
    ).encode("ascii")
    dtype = "<f4" if property_type == "float" else "<f8"
    values = np.zeros((2, len(names)), dtype=dtype)
    columns = {name: index for index, name in enumerate(names)}
    values[:, [columns[axis] for axis in ("x", "y", "z")]] = [
        [1.0, -2.0, 3.0],
        [4.0, 5.0, -6.0],
    ]
    values[:, [columns[f"f_dc_{index}"] for index in range(3)]] = [
        [0.1, 0.2, 0.3],
        [0.4, 0.5, 0.6],
    ]
    values[0, [columns[f"f_rest_{index}"] for index in range(45)]] = np.arange(45)
    values[1, [columns[f"f_rest_{index}"] for index in range(45)]] = np.arange(100, 145)
    values[:, columns["opacity"]] = [0.0, np.log(3.0)]
    values[:, [columns[f"scale_{index}"] for index in range(3)]] = [
        [0.0, np.log(2.0), np.log(3.0)],
        [np.log(4.0), np.log(5.0), np.log(6.0)],
    ]
    values[:, [columns[f"rot_{index}"] for index in range(4)]] = [
        quaternion,
        [1.0, 2.0, 2.0, 0.0],
    ]
    return header + values.tobytes()


def test_load_ply_3dgs_decodes_transforms_and_channel_major_sh(tmp_path):
    path = tmp_path / "background.ply"
    path.write_bytes(_ply_fixture())

    splats = load_gaussian_map(path)

    np.testing.assert_allclose(splats.means, [[1.0, -2.0, 3.0], [4.0, 5.0, -6.0]])
    np.testing.assert_allclose(
        splats.quats, [[1.0, 0.0, 0.0, 0.0], [1 / 3, 2 / 3, 2 / 3, 0]]
    )
    np.testing.assert_allclose(splats.scales, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    np.testing.assert_allclose(splats.opacities, [0.5, 0.75])
    assert splats.sh_coeffs.shape == (2, 16, 3)
    np.testing.assert_allclose(splats.sh_coeffs[0, 0], [0.1, 0.2, 0.3])
    np.testing.assert_allclose(splats.sh_coeffs[0, 1], [0.0, 15.0, 30.0])
    np.testing.assert_allclose(splats.sh_coeffs[0, 15], [14.0, 29.0, 44.0])
    assert splats.sh_degree == 3
    assert not splats.antialiased


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (
            lambda payload: payload.replace(
                b"binary_little_endian", b"ascii               ", 1
            ),
            "binary_little_endian",
        ),
        (lambda payload: payload.replace(b"property float x\n", b"", 1), "missing required"),
        (lambda payload: payload[:-4], "payload size mismatch"),
        (lambda payload: _ply_fixture(property_type="double"), "must be float32"),
        (
            lambda payload: _ply_fixture(quaternion=(0.0, 0.0, 0.0, 0.0)),
            "zero-length quaternion",
        ),
    ],
)
def test_ply_3dgs_validation(tmp_path, mutate, message):
    path = tmp_path / "invalid.ply"
    path.write_bytes(mutate(_ply_fixture()))

    with pytest.raises(ValueError, match=message):
        load_ply_3dgs(path)


def test_load_gaussian_map_rejects_unknown_extension(tmp_path):
    with pytest.raises(ValueError, match="unsupported Gaussian-map extension"):
        load_gaussian_map(tmp_path / "map.bin")
