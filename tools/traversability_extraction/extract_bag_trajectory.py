#!/usr/bin/env python3
"""Extract the mapping trajectory and ground-contact seed points from a ROS 2 bag."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--topic", default="/lio_odom")
    parser.add_argument("--body-height", type=float, default=0.40)
    parser.add_argument("--min-spacing", type=float, default=0.04)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    poses: list[tuple[int, float, float, float, float, float, float, float]] = []
    last_xyz: tuple[float, float, float] | None = None
    frame_id = ""
    child_frame_id = ""

    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(payload, Odometry)
        point = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        xyz = (point.x, point.y, point.z)
        if last_xyz is not None:
            distance = math.dist(xyz, last_xyz)
            if distance < args.min_spacing:
                continue
        poses.append(
            (
                timestamp,
                point.x,
                point.y,
                point.z,
                quat.x,
                quat.y,
                quat.z,
                quat.w,
            )
        )
        last_xyz = xyz
        frame_id = msg.header.frame_id
        child_frame_id = msg.child_frame_id

    if not poses:
        raise RuntimeError(f"no messages found on {args.topic}")

    csv_path = args.output_dir / "trajectory.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(poses)

    pcd_path = args.output_dir / "trajectory_ground.pcd"
    with pcd_path.open("w", encoding="ascii") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write("FIELDS x y z intensity\n")
        stream.write("SIZE 4 4 4 4\n")
        stream.write("TYPE F F F F\n")
        stream.write("COUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(poses)}\nHEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(poses)}\nDATA ascii\n")
        for index, (_, x, y, z, *_quat) in enumerate(poses):
            stream.write(f"{x:.9f} {y:.9f} {z - args.body_height:.9f} {index:.1f}\n")

    xyz = [(row[1], row[2], row[3]) for row in poses]
    summary = {
        "bag": str(args.bag),
        "topic": args.topic,
        "frame_id": frame_id,
        "child_frame_id": child_frame_id,
        "body_height_m": args.body_height,
        "pose_count": len(poses),
        "bounds_body": {
            "min": [min(p[i] for p in xyz) for i in range(3)],
            "max": [max(p[i] for p in xyz) for i in range(3)],
        },
    }
    (args.output_dir / "trajectory_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
