"""CLI wrapper for offline CooperScene Gaussian-splat localization."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.localization.cooperscene_localization import main


if __name__ == "__main__":
    main()
