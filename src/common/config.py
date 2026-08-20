"""Configuration for the two hosts participating in remote teleoperation.

This module describes where a process runs and where it should delegate its
counterpart.  It deliberately does not start processes or open SSH sessions.
"""

from __future__ import annotations

from dataclasses import dataclass
from ipaddress import ip_address
from pathlib import Path, PurePosixPath
import re
from typing import Any, Mapping

import yaml


class ConfigError(ValueError):
    """Raised when a remote-teleoperation host configuration is invalid."""


_ROLE_ALIASES = {
    "vehicle": "vehicle",
    "remote": "remote",
    "remote-ops": "remote",
    "remote_ops": "remote",
}
_HOSTNAME_RE = re.compile(r"(?=.{1,253}\Z)(?:[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?\.)*[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?\Z")
_USER_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_-]*\Z")


@dataclass(frozen=True)
class EndpointConfig:
    """Connection and launch information for one teleoperation host."""

    name: str
    role: str
    address: str
    port: int
    asset_port: int | None
    analysis_port: int | None
    user: str
    repo_path: PurePosixPath
    is_local: bool = False

    @property
    def endpoint_uri(self) -> str:
        """The TCP endpoint used by the process running on this host."""
        address = f"[{self.address}]" if ":" in self.address else self.address
        return f"tcp://{address}:{self.port}"

    @property
    def asset_endpoint_uri(self) -> str:
        """The dedicated reliable mesh endpoint hosted by this machine."""
        if self.asset_port is None:
            raise ConfigError(f"Host {self.name!r} does not define asset_port")
        address = f"[{self.address}]" if ":" in self.address else self.address
        return f"tcp://{address}:{self.asset_port}"

    @property
    def analysis_endpoint_uri(self) -> str:
        """The vehicle-local durable image analysis endpoint."""
        if self.analysis_port is None:
            raise ConfigError(f"Host {self.name!r} does not define analysis_port")
        address = f"[{self.address}]" if ":" in self.address else self.address
        return f"tcp://{address}:{self.analysis_port}"

    @property
    def ssh_uri(self) -> str:
        """An SSH URI identifying the host on which delegated work belongs."""
        address = f"[{self.address}]" if ":" in self.address else self.address
        return f"ssh://{self.user}@{address}"


@dataclass(frozen=True)
class RemoteTeleopConfig:
    """A validated two-host configuration, optionally bound to one local host."""

    endpoints: Mapping[str, EndpointConfig]
    local_name: str | None = None

    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
        *,
        local_host: str | None = None,
        local_role: str | None = None,
    ) -> "RemoteTeleopConfig":
        """Load a two-host YAML file and optionally select the local endpoint.

        ``local_host`` is the top-level YAML key (for example ``alien3``), while
        ``local_role`` accepts ``vehicle``, ``remote``, or ``remote-ops``.
        Exactly one selector may be provided.
        """
        if local_host is not None and local_role is not None:
            raise ConfigError("Specify either local_host or local_role, not both")

        config_path = Path(path)
        try:
            with config_path.open(encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
        except OSError as exc:
            raise ConfigError(f"Could not read configuration {config_path}: {exc}") from exc
        except yaml.YAMLError as exc:
            raise ConfigError(f"Invalid YAML in {config_path}: {exc}") from exc

        if not isinstance(document, Mapping) or not document:
            raise ConfigError("Configuration must be a non-empty mapping of host names")

        endpoints = {
            name: _parse_endpoint(name, values)
            for name, values in document.items()
        }
        if len(endpoints) != 2:
            raise ConfigError("Configuration must define exactly two hosts")
        roles = {endpoint.role for endpoint in endpoints.values()}
        if roles != {"vehicle", "remote"}:
            raise ConfigError("Configuration must define one vehicle host and one remote host")
        if sum(endpoint.is_local for endpoint in endpoints.values()) > 1:
            raise ConfigError("Configuration can define at most one local host")

        local_name = _select_local_name(endpoints, local_host, local_role)
        return cls(endpoints=endpoints, local_name=local_name)

    def for_host(self, host_name: str) -> "RemoteTeleopConfig":
        """Return this configuration as viewed from ``host_name``."""
        if host_name not in self.endpoints:
            raise ConfigError(f"Unknown host {host_name!r}")
        return RemoteTeleopConfig(endpoints=self.endpoints, local_name=host_name)

    def for_role(self, role: str) -> "RemoteTeleopConfig":
        """Return this configuration as viewed from the host with ``role``."""
        normalized_role = _normalize_role(role)
        return self.for_host(self.endpoint_for_role(normalized_role).name)

    @property
    def local(self) -> EndpointConfig:
        """The endpoint on which the caller's process runs."""
        if self.local_name is None:
            raise ConfigError("No local host selected; use for_host() or for_role()")
        return self.endpoints[self.local_name]

    @property
    def peer(self) -> EndpointConfig:
        """The endpoint to which the local process delegates its counterpart."""
        return next(endpoint for name, endpoint in self.endpoints.items() if name != self.local.name)

    @property
    def vehicle(self) -> EndpointConfig:
        """The endpoint assigned to the vehicle-side process."""
        return self.endpoint_for_role("vehicle")

    @property
    def remote(self) -> EndpointConfig:
        """The endpoint assigned to the remote-operations process."""
        return self.endpoint_for_role("remote")

    def endpoint_for_role(self, role: str) -> EndpointConfig:
        """Return the endpoint responsible for a validated role."""
        normalized_role = _normalize_role(role)
        return next(endpoint for endpoint in self.endpoints.values() if endpoint.role == normalized_role)


def _parse_endpoint(name: Any, values: Any) -> EndpointConfig:
    if not isinstance(name, str) or not name.strip():
        raise ConfigError("Host names must be non-empty strings")
    if not isinstance(values, Mapping):
        raise ConfigError(f"Host {name!r} must be a mapping")

    required = {"role", "host_address", "port", "user", "repo_path"}
    optional = {"asset_port", "analysis_port", "local"}
    missing = required - values.keys()
    unknown = values.keys() - required - optional
    if missing:
        raise ConfigError(f"Host {name!r} is missing: {', '.join(sorted(missing))}")
    if unknown:
        raise ConfigError(f"Host {name!r} has unknown fields: {', '.join(sorted(unknown))}")

    role = _normalize_role(values["role"])
    address = _validate_address(values["host_address"], name)
    port = _validate_port(values["port"], name)
    asset_port = (
        _validate_port(values["asset_port"], name)
        if "asset_port" in values
        else None
    )
    analysis_port = (
        _validate_port(values["analysis_port"], name)
        if "analysis_port" in values
        else None
    )
    configured_ports = [value for value in (port, asset_port, analysis_port) if value is not None]
    if len(configured_ports) != len(set(configured_ports)):
        raise ConfigError(f"Host {name!r} frame, asset, and analysis ports must differ")
    user = _validate_user(values["user"], name)
    repo_path = _validate_repo_path(values["repo_path"], name)
    is_local = values.get("local", False)
    if not isinstance(is_local, bool):
        raise ConfigError(f"Host {name!r} local must be a boolean")
    return EndpointConfig(
        name=name,
        role=role,
        address=address,
        port=port,
        asset_port=asset_port,
        analysis_port=analysis_port,
        user=user,
        repo_path=repo_path,
        is_local=is_local,
    )


def _normalize_role(value: Any) -> str:
    if not isinstance(value, str) or value not in _ROLE_ALIASES:
        choices = ", ".join(sorted(_ROLE_ALIASES))
        raise ConfigError(f"Role must be one of: {choices}")
    return _ROLE_ALIASES[value]


def _validate_address(value: Any, host_name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f"Host {host_name!r} has an invalid host_address")
    address = value.strip()
    try:
        ip_address(address)
    except ValueError:
        if not _HOSTNAME_RE.fullmatch(address):
            raise ConfigError(f"Host {host_name!r} has an invalid host_address {value!r}") from None
    return address


def _validate_port(value: Any, host_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
        raise ConfigError(f"Host {host_name!r} port must be an integer from 1 to 65535")
    return value


def _validate_user(value: Any, host_name: str) -> str:
    if not isinstance(value, str) or not _USER_RE.fullmatch(value):
        raise ConfigError(f"Host {host_name!r} has an invalid user")
    return value


def _validate_repo_path(value: Any, host_name: str) -> PurePosixPath:
    if not isinstance(value, str):
        raise ConfigError(f"Host {host_name!r} repo_path must be an absolute path")
    path = PurePosixPath(value)
    if not path.is_absolute() or path == PurePosixPath("/") or ".." in path.parts:
        raise ConfigError(f"Host {host_name!r} repo_path must be a non-root absolute path")
    return path


def _select_local_name(
    endpoints: Mapping[str, EndpointConfig], local_host: str | None, local_role: str | None
) -> str | None:
    if local_host is not None:
        if local_host not in endpoints:
            raise ConfigError(f"Unknown local_host {local_host!r}")
        return local_host
    if local_role is not None:
        normalized_role = _normalize_role(local_role)
        return next(endpoint.name for endpoint in endpoints.values() if endpoint.role == normalized_role)
    return None
