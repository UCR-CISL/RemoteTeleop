#!/usr/bin/env python3
"""Plot latency metrics from localhost_times.csv."""

import argparse
import csv
from pathlib import Path


def load_column(csv_path: Path, column: str):
    x_vals = []
    y_vals = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row.")

        fallback_columns = {
            "observed_capture_to_send_ns": ["capture_to_send_ns"],
            "observed_send_to_recv_ns": ["send_to_recv_ns"],
            "observed_recv_to_display_ns": ["recv_to_display_ns"],
            "observed_send_to_display_ns": ["send_to_display_cross_host_ns"],
            "send_to_display_cross_host_ns": ["observed_send_to_display_ns"],
        }
        resolved_column = column
        if resolved_column not in reader.fieldnames:
            for fallback in fallback_columns.get(column, []):
                if fallback in reader.fieldnames:
                    resolved_column = fallback
                    break
        if resolved_column not in reader.fieldnames:
            raise ValueError(
                f"Column '{column}' not found. Available: {reader.fieldnames}"
            )

        for i, row in enumerate(reader):
            raw = row.get(resolved_column, "")
            if raw is None or raw == "":
                continue
            try:
                value_ns = float(raw)
            except ValueError:
                continue
            x_vals.append(i)
            y_vals.append(value_ns / 1_000_000.0)  # ns -> ms

    if not x_vals:
        raise ValueError(f"No numeric values found in column '{column}'.")

    return x_vals, y_vals


def moving_average(values, window):
    if window <= 1:
        return values

    out = []
    running_sum = 0.0
    queue = []
    for v in values:
        queue.append(v)
        running_sum += v
        if len(queue) > window:
            running_sum -= queue.pop(0)
        out.append(running_sum / len(queue))
    return out


def main():
    parser = argparse.ArgumentParser(
        description="Plot latency columns from localhost_times.csv"
    )
    parser.add_argument(
        "--csv",
        default="localhost_times.csv",
        help="Path to CSV file (default: localhost_times.csv)",
    )
    parser.add_argument(
        "--column",
        default="observed_send_to_display_ns",
        help="CSV column to plot (default: observed_send_to_display_ns)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=100,
        help="Moving average window in frames (default: 100)",
    )
    parser.add_argument(
        "--out",
        default="latency_plot.png",
        help="Output image path (default: latency_plot.png)",
    )
    parser.add_argument(
        "--title",
        default=None,
        help="Custom chart title",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    x_vals, y_vals = load_column(csv_path, args.column)
    avg_vals = moving_average(y_vals, args.window)

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required. Install with: pip install matplotlib"
        ) from exc

    plt.figure(figsize=(12, 6))
    plt.plot(x_vals, y_vals, linewidth=0.8, alpha=0.45, label=f"{args.column} (raw)")
    plt.plot(x_vals, avg_vals, linewidth=2.0, label=f"{args.column} ({args.window}-frame avg)")

    title = args.title or f"Latency over time: {args.column}"
    plt.title(title)
    plt.xlabel("Frame index")
    plt.ylabel("Latency (ms)")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out, dpi=160)

    print(f"Saved plot to {args.out}")
    print(f"Plotted {len(y_vals)} points from {csv_path} ({args.column}).")


if __name__ == "__main__":
    main()
