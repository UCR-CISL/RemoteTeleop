from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys


def _module():
    path = Path(__file__).parents[1] / "scripts/run_cooperscene_runtime_matrix.py"
    spec = spec_from_file_location("runtime_matrix", path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_matrix_contains_requested_tradeoff_variants(tmp_path):
    module = _module()
    variants = {variant.name: variant.arguments for variant in module.VARIANTS}

    assert "take_turns_fp16" in variants
    assert "sam3_cpu_sam3d_gpu" in variants
    assert "sam3_gpu_sam3d_cpu" in variants
    assert variants["both_gpu_nf4"][-2:] == ("--sam3d-precision", "nf4")
    assert variants["sam3_fp16_sam3d_nf4"][-2:] == ("--sam3d-precision", "nf4")

    command = module._base_command(Path("python"), tmp_path)
    assert "--stop-at-overlap" not in command
    assert command[command.index("--agent") + 1] == "1"
