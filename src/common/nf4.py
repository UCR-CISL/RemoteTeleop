"""Selective bitsandbytes NF4 conversion for dense inference layers."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import asdict, dataclass
from typing import Any

NF4ExcludePredicate = Callable[[str, Any, str, Any], bool]


@dataclass(frozen=True)
class NF4QuantizationMetrics:
    total_parameters: int
    quantized_parameters: int
    total_modules: int
    total_linear_modules: int
    quantized_linear_modules: int

    def as_dict(self) -> dict[str, int | float]:
        values: dict[str, int | float] = asdict(self)
        values["quantized_parameter_fraction"] = (
            self.quantized_parameters / self.total_parameters
            if self.total_parameters
            else 0.0
        )
        return values


def quantize_dense_linear_nf4(
    roots: Iterable[Any],
    *,
    device: str,
    linear_factory: Callable[[Any, str], Any] | None = None,
    exclude: NF4ExcludePredicate | None = None,
) -> NF4QuantizationMetrics:
    """Replace only exact ``torch.nn.Linear`` children with NF4 modules.

    Exact-type matching is intentional. In particular, SAM3D's SparseLinear
    subclasses nn.Linear but consumes and returns SparseTensor wrappers; a
    generic Linear4bit replacement would break that contract. Convolutions,
    MultiheadAttention's specialized output projection, and spconv modules are
    likewise left untouched.
    """

    import torch

    materialized = tuple(root for root in roots if root is not None)
    total_parameters = sum(
        parameter.numel()
        for root in materialized
        for parameter in root.parameters()
    )
    total_linear_modules = sum(
        1
        for root in materialized
        for module in root.modules()
        if type(module) is torch.nn.Linear
    )
    total_modules = sum(1 for root in materialized for _module in root.modules())
    factory = linear_factory or _bitsandbytes_nf4_linear
    quantized_parameters = 0
    quantized_modules = 0

    def convert(parent: Any, path: str) -> None:
        nonlocal quantized_parameters, quantized_modules
        for name, child in tuple(parent.named_children()):
            child_path = f"{path}.{name}" if path else name
            if type(child) is torch.nn.Linear and not (
                exclude is not None and exclude(child_path, parent, name, child)
            ):
                quantized_parameters += child.weight.numel()
                quantized_modules += 1
                setattr(parent, name, factory(child, device))
            else:
                convert(child, child_path)

    for root in materialized:
        convert(root, "")
    return NF4QuantizationMetrics(
        total_parameters=total_parameters,
        quantized_parameters=quantized_parameters,
        total_modules=total_modules,
        total_linear_modules=total_linear_modules,
        quantized_linear_modules=quantized_modules,
    )


def _bitsandbytes_nf4_linear(linear: Any, device: str) -> Any:
    import bitsandbytes as bnb
    import torch

    replacement = bnb.nn.Linear4bit(
        linear.in_features,
        linear.out_features,
        bias=linear.bias is not None,
        compute_dtype=torch.float16,
        compress_statistics=True,
        quant_type="nf4",
        device="cpu",
    )
    replacement.weight = bnb.nn.Params4bit(
        linear.weight.detach().cpu(),
        requires_grad=False,
        compress_statistics=True,
        quant_type="nf4",
        module=replacement,
    )
    if linear.bias is not None:
        replacement.bias.data.copy_(linear.bias.detach().cpu())
        replacement.bias.requires_grad_(False)
    replacement.eval()
    # Params4bit performs the actual packing when moved to CUDA.
    return replacement.to(device)
