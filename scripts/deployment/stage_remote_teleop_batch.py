#!/usr/bin/env python3
"""Stage proxy replay code in isolated host directories without starting services."""
from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import subprocess
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.config import RemoteTeleopConfig


class BatchCodeStager:
    def __init__(self, config: Path, original_config: Path, source: Path):
        self.config = RemoteTeleopConfig.from_yaml(config)
        self.original = RemoteTeleopConfig.from_yaml(original_config)
        self.source = source.resolve()
        for role in ("vehicle", "remote"):
            target, original = getattr(self.config, role), getattr(self.original, role)
            if (target.address, target.user) != (original.address, original.user):
                raise ValueError("staging and original configurations must identify the same hosts")
            if target.repo_path == original.repo_path:
                raise ValueError("staging requires separate repository paths")

    def commands(self):
        for role in ("vehicle", "remote"):
            target, original = getattr(self.config, role), getattr(self.original, role)
            host = f"{target.user}@{target.address}"
            root = str(target.repo_path)
            interpreter = str(original.repo_path / ".venv/bin/python")
            ssh = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15", host]
            yield ssh + ["mkdir -p " + shlex.quote(root)]
            sources = [str(self.source / name) for name in ("src", "scripts", "cfg", "config", "docker", "pyproject.toml", "uv.lock")]
            yield ["rsync", "-az", "--partial", "--protect-args", "--exclude=__pycache__", "-e", "ssh -o BatchMode=yes -o ConnectTimeout=15", *sources, f"{host}:{root}/"]
            # Keep the existing CUDA environment; only the Foxglove decoder's
            # lightweight dependency lives in an isolated supplemental directory.
            if role == "vehicle":
                yield ssh + ['export PATH="$HOME/.local/bin:$PATH"; ' + shlex.join(["uv", "pip", "install", "--python", interpreter,
                                        "--target", root + "/.batch-site", "protobuf>=4.25", "foxglove-schemas-protobuf>=0.4.0"])]
            wrapper = "#!/bin/sh\nexport PYTHONPATH=" + shlex.quote(root + "/.batch-site:" + root) + '\nexec ' + shlex.quote(interpreter) + ' "$@"\n'
            wrapper_path = root + "/batch-python"
            yield ssh + ["printf %s " + shlex.quote(wrapper) + " > " + shlex.quote(wrapper_path)
                         + " && chmod +x " + shlex.quote(wrapper_path)]
            check = ("import cv2, numpy, scipy, zmq, mcap; from google.protobuf.message_factory import GetMessageClass; import src.data.foxglove_mcap"
                     if role == "vehicle" else
                     "import torch, trimesh; from gsplat import rasterization; from pytorch3d.renderer import MeshRasterizer; import src.realtime.composited_camera_process; assert torch.cuda.is_available()")
            yield ssh + ["cd " + shlex.quote(root) + " && " + shlex.join([wrapper_path, "-c", check])]

    def run(self, *, dry_run=False):
        for command in self.commands():
            print(shlex.join(command), flush=True)
            if not dry_run:
                subprocess.run(command, check=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("cfg/alien3_lambda_proxy_batch.yaml"))
    parser.add_argument("--original-config", type=Path, default=Path("cfg/alien3_lambda.yaml"))
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    BatchCodeStager(args.config, args.original_config, args.source).run(dry_run=args.dry_run)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
