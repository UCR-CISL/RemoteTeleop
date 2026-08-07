import gzip
import struct

import numpy as np
import pytest

from src.localization.gaussian_map import _decode_spz_v3, load_spz_v3


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
