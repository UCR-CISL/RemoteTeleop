from __future__ import annotations

import pytest
import torch
from types import SimpleNamespace

from src.common.nf4 import quantize_dense_linear_nf4
from src.reconstruction.sam3d_upstream import UpstreamSAM3DBackend, _sam3d_nf4_roots
from src.segmentation import SAM3MaskBackendConfig
from src.segmentation.backends import (
    _OfficialSAM3ImagePredictor,
    _exclude_sam3_nf4_linear,
)


class _SparseLikeLinear(torch.nn.Linear):
    """Stand-in for SAM3D SparseLinear's non-Tensor forward contract."""


class _FakeNF4(torch.nn.Module):
    def __init__(self, source: torch.nn.Linear, device: str) -> None:
        super().__init__()
        self.in_features = source.in_features
        self.out_features = source.out_features
        self.requested_device = device

    def forward(self, value: torch.Tensor) -> torch.Tensor:
        return value


def test_nf4_quantizes_only_exact_dense_linear_modules():
    model = torch.nn.Sequential(
        torch.nn.Linear(4, 8),
        torch.nn.Sequential(torch.nn.Linear(8, 2), _SparseLikeLinear(2, 1)),
        torch.nn.Conv2d(1, 1, 1),
    )
    total_parameters = sum(parameter.numel() for parameter in model.parameters())

    metrics = quantize_dense_linear_nf4(
        (model,),
        device="cuda:7",
        linear_factory=lambda module, device: _FakeNF4(module, device),
    )

    assert isinstance(model[0], _FakeNF4)
    assert isinstance(model[1][0], _FakeNF4)
    assert type(model[1][1]) is _SparseLikeLinear
    assert type(model[2]) is torch.nn.Conv2d
    assert model[0].requested_device == "cuda:7"
    assert metrics.total_parameters == total_parameters
    assert metrics.quantized_parameters == 4 * 8 + 8 * 2
    assert metrics.total_linear_modules == 2
    assert metrics.quantized_linear_modules == 2
    assert metrics.total_modules == 6
    assert metrics.as_dict()["quantized_parameter_fraction"] == pytest.approx(
        metrics.quantized_parameters / total_parameters
    )


def test_sam3_nf4_excludes_fused_vitdet_fc1_but_keeps_safe_linears():
    class Mlp(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.fc1 = torch.nn.Linear(4, 8)
            self.fc2 = torch.nn.Linear(8, 4)

    Mlp.__module__ = "sam3.model.vitdet"
    model = torch.nn.Sequential(Mlp(), torch.nn.Linear(4, 2))

    metrics = quantize_dense_linear_nf4(
        (model,),
        device="cuda:0",
        linear_factory=lambda module, device: _FakeNF4(module, device),
        exclude=_exclude_sam3_nf4_linear,
    )

    assert type(model[0].fc1) is torch.nn.Linear
    assert isinstance(model[0].fc2, _FakeNF4)
    assert isinstance(model[1], _FakeNF4)
    assert metrics.total_linear_modules == 3
    assert metrics.quantized_linear_modules == 2
    assert 0.0 < metrics.as_dict()["quantized_parameter_fraction"] < 1.0


def test_sam3_cpu_autocast_bridges_bf16_activation_to_fp32_linear():
    predictor = object.__new__(_OfficialSAM3ImagePredictor)
    predictor._device = "cpu"
    predictor._precision = "default"
    activation = torch.randn(2, 4, dtype=torch.bfloat16)
    linear = torch.nn.Linear(4, 3)

    with predictor._autocast():
        output = linear(activation)

    assert output.dtype is torch.bfloat16


def test_nf4_configuration_requires_cuda_and_disables_compile(tmp_path):
    with pytest.raises(ValueError, match="CUDA"):
        SAM3MaskBackendConfig(device="cpu", precision="nf4")
    with pytest.raises(ValueError, match="compile"):
        SAM3MaskBackendConfig(device="cuda", precision="nf4", compile=True)
    with pytest.raises(ValueError, match="default BF16"):
        SAM3MaskBackendConfig(device="cpu", precision="fp16")
    with pytest.raises(ValueError, match="compilation"):
        UpstreamSAM3DBackend(
            repository=tmp_path,
            config_path="pipeline.yaml",
            precision="nf4",
            compile_model=True,
        )


def test_sam3d_nf4_roots_flattens_mappings_and_unwraps_depth_model():
    model = torch.nn.Linear(2, 2)
    conditioner = torch.nn.Linear(2, 2)
    depth = torch.nn.Linear(2, 2)
    inference = SimpleNamespace(
        models={"generator": model},
        condition_embedders={"image": conditioner},
        depth_model=SimpleNamespace(model=depth),
        pose_decoder=lambda value: value,
    )

    assert _sam3d_nf4_roots(inference) == (model, conditioner, depth)


@pytest.mark.parametrize("precision", ("default", "fp16", "nf4"))
def test_sam3d_accepts_supported_precisions(tmp_path, precision):
    backend = UpstreamSAM3DBackend(
        repository=tmp_path,
        config_path="pipeline.yaml",
        precision=precision,
    )
    assert backend.precision == precision
