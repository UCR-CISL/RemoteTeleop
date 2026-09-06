"""Command-line entry point for offline NuScenes-to-Gaussian localization."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.localization.nuscenes_localization import main


if __name__ == "__main__":
    main()
