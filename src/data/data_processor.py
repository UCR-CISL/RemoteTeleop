"""Base contract for offline dataset conversions."""

from __future__ import annotations

from abc import ABC, abstractmethod


class DataProcessor(ABC):
    """A stateful conversion from one dataset representation to another."""

    @abstractmethod
    def process(self) -> int:
        """Run the conversion and return the number of emitted records."""
