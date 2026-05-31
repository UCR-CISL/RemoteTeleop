from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CloudXRStreamerPaths:
    repo_root: Path
    config_path: Path
    camera_viz_dir: Path

    @classmethod
    def from_repo(cls, config_path: Path | None = None) -> "CloudXRStreamerPaths":
        repo_root = Path(__file__).resolve().parents[2]
        return cls(
            repo_root=repo_root,
            config_path=(
                config_path
                if config_path is not None
                else repo_root / "config" / "lucid_cloudxr_streamer.yaml"
            ),
            camera_viz_dir=repo_root
            / "thirdparty"
            / "IsaacTeleop"
            / "examples"
            / "camera_viz",
        )


class CloudXRStreamer:
    """Launch IsaacTeleop camera_viz for the Lucid RTP stream.

    The Lucid process still owns camera capture and RTP/H.264 encoding.
    camera_viz owns the receiver/decode/render side, and CloudXR/OpenXR owns
    delivery of the rendered XR view to the headset or client.
    """

    def __init__(self, paths: CloudXRStreamerPaths) -> None:
        self.paths = paths

    @property
    def camera_viz_script(self) -> Path:
        return self.paths.camera_viz_dir / "camera_viz.sh"

    @property
    def camera_viz_python(self) -> Path:
        return self.paths.camera_viz_dir / ".venv" / "bin" / "python"

    def build_command(self) -> list[str]:
        return [
            "bash",
            str(self.camera_viz_script),
            "run",
            str(self.paths.config_path),
        ]

    def validate(self, require_setup: bool = True) -> None:
        missing: list[str] = []
        if not self.camera_viz_script.is_file():
            missing.append(f"missing IsaacTeleop camera_viz script: {self.camera_viz_script}")
        if not self.paths.config_path.is_file():
            missing.append(f"missing CloudXR streamer config: {self.paths.config_path}")
        if require_setup and not self.camera_viz_python.is_file():
            missing.append(
                "IsaacTeleop camera_viz environment is not set up. Run: "
                f"cd {self.paths.camera_viz_dir} && ./camera_viz.sh setup"
            )
        if missing:
            raise FileNotFoundError("\n".join(missing))

    def warn_if_xr_env_missing(self) -> None:
        if not self._config_requests_xr():
            return
        if os.environ.get("XR_RUNTIME_JSON") or os.environ.get("ISAAC_TELEOP_SKIP_XR_ENV_WARNING"):
            return
        print(
            "cloudxr_streamer: XR_RUNTIME_JSON is not set. If CloudXR is your OpenXR "
            "runtime, source the CloudXR environment before launching.",
            file=sys.stderr,
        )

    def run(self, dry_run: bool = False, require_setup: bool = True) -> int:
        self.validate(require_setup=require_setup)
        self.warn_if_xr_env_missing()
        command = self.build_command()
        if dry_run:
            print(" ".join(command))
            return 0
        os.execvp(command[0], command)
        return 0

    def _config_requests_xr(self) -> bool:
        try:
            lines = self.paths.config_path.read_text(encoding="utf-8").splitlines()
        except OSError:
            return False
        return any(line.strip().lower() == "mode: xr" for line in lines)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch IsaacTeleop camera_viz as the Lucid CloudXR streamer."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help="camera_viz YAML config. Defaults to config/lucid_cloudxr_streamer.yaml.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the camera_viz command without launching it.",
    )
    parser.add_argument(
        "--skip-setup-check",
        action="store_true",
        help="Do not require examples/camera_viz/.venv to exist before launch.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    paths = CloudXRStreamerPaths.from_repo(args.config.resolve() if args.config else None)
    streamer = CloudXRStreamer(paths)
    return streamer.run(
        dry_run=args.dry_run,
        require_setup=not args.skip_setup_check,
    )


if __name__ == "__main__":
    raise SystemExit(main())
