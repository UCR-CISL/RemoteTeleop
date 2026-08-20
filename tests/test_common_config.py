from pathlib import Path

import pytest

from src.common.config import ConfigError, RemoteTeleopConfig


def test_loads_two_hosts_and_selects_remote_local_endpoint() -> None:
    config = RemoteTeleopConfig.from_yaml(
        Path("cfg/alien3_lambda.yaml"), local_role="remote-ops"
    )

    assert config.local.name == "alien3"
    assert config.local.role == "remote"
    assert config.peer.name == "lambda"
    assert config.vehicle.name == "lambda"
    assert config.remote.name == "alien3"
    assert config.vehicle.endpoint_uri == "tcp://100.97.168.98:8768"
    assert config.vehicle.asset_endpoint_uri == "tcp://100.97.168.98:8769"
    assert config.remote.ssh_uri == "ssh://coop3r-slam@100.109.210.56"
    assert config.vehicle.repo_path == Path("/home/justin/Documents/CISL-Projects/RemoteTeleop")


def test_loads_without_a_local_host_and_can_bind_later() -> None:
    config = RemoteTeleopConfig.from_yaml(Path("cfg/alien3_lambda.yaml"))

    with pytest.raises(ConfigError, match="No local host selected"):
        _ = config.local

    vehicle_view = config.for_host("lambda")
    assert vehicle_view.local is config.vehicle
    assert vehicle_view.peer is config.remote


def test_alien4_config_marks_the_vehicle_as_local() -> None:
    config = RemoteTeleopConfig.from_yaml(Path("cfg/alien4_alien3.yaml"))

    assert config.vehicle.name == "alien4"
    assert config.vehicle.is_local is True
    assert config.remote.is_local is False


@pytest.mark.parametrize(
    ("yaml_text", "message"),
    [
        (
            """remote:\n  role: remote\n  host_address: bad address\n  port: 8767\n  user: remote\n  repo_path: /srv/teleop\nvehicle:\n  role: vehicle\n  host_address: 10.0.0.2\n  port: 8768\n  user: vehicle\n  repo_path: /srv/teleop\n""",
            "invalid host_address",
        ),
        (
            """remote:\n  role: remote\n  host_address: 10.0.0.1\n  port: 70000\n  user: remote\n  repo_path: /srv/teleop\nvehicle:\n  role: vehicle\n  host_address: 10.0.0.2\n  port: 8768\n  user: vehicle\n  repo_path: /srv/teleop\n""",
            "port must be an integer",
        ),
        (
            """remote:\n  role: remote\n  host_address: 10.0.0.1\n  port: 8767\n  user: remote\n  repo_path: relative/path\nvehicle:\n  role: vehicle\n  host_address: 10.0.0.2\n  port: 8768\n  user: vehicle\n  repo_path: /srv/teleop\n""",
            "repo_path",
        ),
    ],
)
def test_rejects_invalid_endpoint_fields(tmp_path: Path, yaml_text: str, message: str) -> None:
    config_path = tmp_path / "hosts.yaml"
    config_path.write_text(yaml_text, encoding="utf-8")

    with pytest.raises(ConfigError, match=message):
        RemoteTeleopConfig.from_yaml(config_path)


def test_requires_exactly_one_vehicle_and_remote_role(tmp_path: Path) -> None:
    config_path = tmp_path / "hosts.yaml"
    config_path.write_text(
        """one:\n  role: vehicle\n  host_address: 10.0.0.1\n  port: 8767\n  user: one\n  repo_path: /srv/one\ntwo:\n  role: vehicle\n  host_address: 10.0.0.2\n  port: 8768\n  user: two\n  repo_path: /srv/two\n""",
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="one vehicle host and one remote host"):
        RemoteTeleopConfig.from_yaml(config_path)
