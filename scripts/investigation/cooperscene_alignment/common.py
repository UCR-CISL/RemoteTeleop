"""Shared provenance helpers for the offline CooperScene frame search."""
import hashlib
from pathlib import Path

from src.localization.cooperscene_geodetic import DEFAULT_RECIPE, FrozenProfile

REPOSITORY = Path(__file__).resolve().parents[3]
DEFAULT_PRIOR = DEFAULT_RECIPE


def file_digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()
