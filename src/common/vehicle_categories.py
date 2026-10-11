"""Shared nuScenes and detector labels for supported vehicle proxy families."""

_VEHICLE_PROXY_KINDS = {
    # Generic vehicle predictions use a car-shaped primitive scaled to their
    # measured dimensions. This rendering fallback does not infer a subtype.
    "vehicle": "car",
    "car": "car",
    "truck": "truck",
    "bus": "bus",
    "articulated_bus": "articulated_bus",
    "trailer": "trailer",
    "construction_vehicle": "construction_vehicle",
    "motorcycle": "motorcycle",
    "bicycle": "bicycle",
    "ambulance": "ambulance",
    "police": "police",
    "vehicle.car": "car",
    "vehicle.truck": "truck",
    "vehicle.bus.rigid": "bus",
    "vehicle.bus.bendy": "articulated_bus",
    "vehicle.trailer": "trailer",
    "vehicle.construction": "construction_vehicle",
    "vehicle.motorcycle": "motorcycle",
    "vehicle.bicycle": "bicycle",
    "vehicle.emergency.ambulance": "ambulance",
    "vehicle.emergency.police": "police",
}


def vehicle_proxy_kind(label: str) -> str | None:
    """Return a canonical vehicle family; unsupported labels remain excluded."""
    return _VEHICLE_PROXY_KINDS.get(label)
