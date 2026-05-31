from pathlib import Path

import pytest

from src.streaming.cloudxr_streamer import CloudXRStreamer, CloudXRStreamerPaths


def test_build_command_uses_camera_viz_run(tmp_path: Path) -> None:
    paths = CloudXRStreamerPaths(
        repo_root=tmp_path,
        config_path=tmp_path / "config.yaml",
        camera_viz_dir=tmp_path / "camera_viz",
    )

    assert CloudXRStreamer(paths).build_command() == [
        "bash",
        str(tmp_path / "camera_viz" / "camera_viz.sh"),
        "run",
        str(tmp_path / "config.yaml"),
    ]


def test_validate_reports_missing_setup(tmp_path: Path) -> None:
    camera_viz_dir = tmp_path / "camera_viz"
    camera_viz_dir.mkdir()
    (camera_viz_dir / "camera_viz.sh").write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    config_path = tmp_path / "config.yaml"
    config_path.write_text("source: rtp\n", encoding="utf-8")
    streamer = CloudXRStreamer(
        CloudXRStreamerPaths(
            repo_root=tmp_path,
            config_path=config_path,
            camera_viz_dir=camera_viz_dir,
        )
    )

    with pytest.raises(FileNotFoundError, match="environment is not set up"):
        streamer.validate()


def test_validate_can_skip_setup_check(tmp_path: Path) -> None:
    camera_viz_dir = tmp_path / "camera_viz"
    camera_viz_dir.mkdir()
    (camera_viz_dir / "camera_viz.sh").write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    config_path = tmp_path / "config.yaml"
    config_path.write_text("source: rtp\n", encoding="utf-8")
    streamer = CloudXRStreamer(
        CloudXRStreamerPaths(
            repo_root=tmp_path,
            config_path=config_path,
            camera_viz_dir=camera_viz_dir,
        )
    )

    streamer.validate(require_setup=False)
