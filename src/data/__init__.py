"""Offline datasets used to exercise remote-teleoperation pipelines."""

from src.data.cooperscene_mcap import CooperSceneMcapConfig, CooperSceneMcapProcessor
from src.data.data_processor import DataProcessor
from src.data.nuscenes_loader import NuScenesFrame, NuScenesSequence, NuScenesSequenceDataset, VehicleBox
from src.data.processed_nuscenes import (
    ProcessedNuScenesFrame,
    ProcessedNuScenesSequence,
    ProcessedNuScenesVehicle,
)

__all__ = [
    "CooperSceneMcapConfig",
    "CooperSceneMcapProcessor",
    "DataProcessor",
    "NuScenesFrame",
    "NuScenesSequence",
    "NuScenesSequenceDataset",
    "VehicleBox",
    "ProcessedNuScenesFrame",
    "ProcessedNuScenesSequence",
    "ProcessedNuScenesVehicle",
    "ProcessedNuScenesMcapConfig",
    "ProcessedNuScenesMcapProcessor",
]


def __getattr__(name: str):
    if name in {"ProcessedNuScenesMcapConfig", "ProcessedNuScenesMcapProcessor"}:
        from src.data import processed_nuscenes_mcap

        value = getattr(processed_nuscenes_mcap, name)
        globals()[name] = value
        return value
    raise AttributeError(name)
