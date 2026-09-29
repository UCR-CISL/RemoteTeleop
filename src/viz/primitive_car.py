"""Compatibility import for the car-specific procedural asset."""
from src.viz.primitive_vehicle import PrimitiveVehicleAsset


class PrimitiveCarAsset(PrimitiveVehicleAsset):
    """A metric car template with bbox adaptation and fixed circular wheels."""

    def __init__(self) -> None:
        super().__init__("car")
