#!/usr/bin/env python3
"""Write a comparable summary for one realtime experiment directory."""

from __future__ import annotations

import argparse
from pathlib import Path

from src.realtime.benchmark_summary import write_summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_root", type=Path)
    parser.add_argument("--variant", required=True)
    args = parser.parse_args()
    print(write_summary(args.output_root, variant=args.variant))


if __name__ == "__main__":
    main()
