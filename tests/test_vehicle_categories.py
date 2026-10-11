import pytest

from src.common.vehicle_categories import vehicle_proxy_kind


@pytest.mark.parametrize("label,kind", [
    ("vehicle", "car"),
    ("vehicle.car", "car"), ("vehicle.truck", "truck"),
    ("vehicle.bus.rigid", "bus"), ("vehicle.bus.bendy", "articulated_bus"),
    ("vehicle.trailer", "trailer"), ("vehicle.construction", "construction_vehicle"),
    ("vehicle.motorcycle", "motorcycle"), ("vehicle.bicycle", "bicycle"),
    ("vehicle.emergency.ambulance", "ambulance"), ("vehicle.emergency.police", "police"),
])
def test_vehicle_labels_and_shorthand_map_to_proxy_family(label, kind):
    assert vehicle_proxy_kind(label) == kind
    assert vehicle_proxy_kind(kind) == kind


@pytest.mark.parametrize("label", ["human.pedestrian.adult", "pedestrian", "animal", "movable_object.barrier", "traffic_cone", "vehicle.unknown", ""])
def test_nonvehicle_and_unknown_labels_have_no_proxy(label):
    assert vehicle_proxy_kind(label) is None
