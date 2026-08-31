#!/usr/bin/env python3
"""Create comparable tracking plots from fixed_path_benchmark CSV output."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_rows(path: Path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def series(rows, name):
    return [float(row[name]) for row in rows]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path, help="CSV written by fixed_path_benchmark_node")
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()
    rows = load_rows(args.csv)
    if not rows:
        raise SystemExit("CSV has no samples")
    output = args.output_dir or args.csv.parent
    output.mkdir(parents=True, exist_ok=True)
    stem = args.csv.stem
    time = series(rows, "time_s")

    plt.figure(figsize=(7, 6))
    plt.plot(series(rows, "reference_x"), series(rows, "reference_y"), "k--", label="reference")
    plt.plot(series(rows, "actual_x"), series(rows, "actual_y"), "b", label="actual")
    plt.axis("equal")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Reference and actual XY trajectory")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output / f"{stem}_xy.png", dpi=160)
    plt.close()

    plt.figure(figsize=(9, 4))
    plt.plot(time, series(rows, "xy_error_m"), label="XY")
    plt.plot(time, series(rows, "error_3d_m"), label="3D")
    plt.plot(time, series(rows, "lateral_error_m"), label="lateral")
    plt.xlabel("time (s)")
    plt.ylabel("error (m)")
    plt.title("Tracking error")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output / f"{stem}_error.png", dpi=160)
    plt.close()

    fig, (yaw_axis, command_axis) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    yaw_axis.plot(time, series(rows, "yaw_error_rad"), label="yaw error")
    yaw_axis.set_ylabel("rad")
    yaw_axis.grid(True)
    yaw_axis.legend()
    command_axis.plot(time, series(rows, "cmd_vx"), label="vx")
    command_axis.plot(time, series(rows, "cmd_vy"), label="vy")
    command_axis.plot(time, series(rows, "cmd_wz"), label="wz")
    command_axis.set_xlabel("time (s)")
    command_axis.set_ylabel("command")
    command_axis.grid(True)
    command_axis.legend()
    fig.suptitle("Yaw error and command")
    fig.tight_layout()
    fig.savefig(output / f"{stem}_yaw_command.png", dpi=160)
    print(f"plots written to {output}")


if __name__ == "__main__":
    main()
