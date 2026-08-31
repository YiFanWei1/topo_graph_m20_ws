#!/usr/bin/env python3
"""Measure map/path rates and processing latency reported by the demo nodes."""

import argparse
import math
import statistics
import time
from collections import Counter, defaultdict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node


class Metrics(Node):
    def __init__(self):
        super().__init__("efficient_3d_local_planner_bag_metrics")
        self.maps = 0
        self.paths = 0
        self.nonempty_paths = 0
        self.map_ms = []
        self.plan_ms = []
        self.reasons = Counter()
        self.reason_metrics = defaultdict(lambda: defaultdict(list))
        self.first_path_invalidations = None
        self.last_path_invalidations = None
        self.first_revision = None
        self.last_revision = None
        self.first_map_stamp = None
        self.last_map_stamp = None
        self.commands = 0
        self.nonzero_commands = 0
        self.max_abs_vx = 0.0
        self.max_abs_vy = 0.0
        self.max_abs_wz = 0.0
        self.controller_states = Counter()
        self.controller_diagnostics = 0
        self.large_yaw_samples = 0
        self.max_linear_at_large_yaw = 0.0
        self.max_abs_yaw_error = 0.0
        self.yaw_sign_changes = 0
        self.rapid_yaw_sign_changes = 0
        self.previous_yaw_sign = 0
        self.previous_yaw_sign_time = None
        self.create_subscription(Path, "/local_planner/local_path", self.on_path, 10)
        self.create_subscription(
            DiagnosticArray, "/local_voxel_map/diagnostics", self.on_diagnostics, 10)
        self.create_subscription(
            DiagnosticArray, "/local_planner/diagnostics", self.on_diagnostics, 10)
        self.create_subscription(
            DiagnosticArray, "/local_planner/controller_diagnostics",
            self.on_diagnostics, 20)
        self.create_subscription(
            Twist, "/cmd_vel_smoothed", self.on_command, 20)

    def on_command(self, message):
        self.commands += 1
        moving = (abs(message.linear.x) > 1e-4 or
                  abs(message.linear.y) > 1e-4 or
                  abs(message.angular.z) > 1e-4)
        self.nonzero_commands += int(moving)
        self.max_abs_vx = max(self.max_abs_vx, abs(message.linear.x))
        self.max_abs_vy = max(self.max_abs_vy, abs(message.linear.y))
        self.max_abs_wz = max(self.max_abs_wz, abs(message.angular.z))

    def on_path(self, message):
        self.paths += 1
        self.nonempty_paths += int(bool(message.poses))

    def on_diagnostics(self, message):
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if "last_update_ms" in values:
                self.map_ms.append(float(values["last_update_ms"]))
                self.maps += 1
                revision = int(values["revision"])
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                if self.first_revision is None:
                    self.first_revision = revision
                    self.first_map_stamp = stamp
                self.last_revision = revision
                self.last_map_stamp = stamp
            if "planning_ms" in values:
                self.plan_ms.append(float(values["planning_ms"]))
                self.reasons[status.message] += 1
                for key in (
                    "expansions", "closest_goal_distance", "furthest_guide_arc",
                    "rejected_outside_map", "rejected_corridor_xy",
                    "rejected_corridor_z", "rejected_diagonal_corner",
                    "rejected_hard_collision", "rejected_soft_layer",
                    "start_front_hard", "start_rear_hard",
                ):
                    if key in values:
                        self.reason_metrics[status.message][key].append(float(values[key]))
                if "path_invalidations_total" in values:
                    invalidations = int(values["path_invalidations_total"])
                    if self.first_path_invalidations is None:
                        self.first_path_invalidations = invalidations
                    self.last_path_invalidations = invalidations
            if status.name.endswith("/local_path_follower"):
                self.controller_states[status.message] += 1
                self.controller_diagnostics += 1
                yaw_error = float(values.get("yaw_error", 0.0))
                vx = float(values.get("cmd_vx", 0.0))
                vy = float(values.get("cmd_vy", 0.0))
                wz = float(values.get("cmd_wz", 0.0))
                self.max_abs_yaw_error = max(self.max_abs_yaw_error, abs(yaw_error))
                if abs(yaw_error) >= math.radians(40.0):
                    self.large_yaw_samples += 1
                    self.max_linear_at_large_yaw = max(
                        self.max_linear_at_large_yaw, (vx * vx + vy * vy) ** 0.5)
                if abs(wz) >= 0.05:
                    sign = 1 if wz > 0.0 else -1
                    stamp = (message.header.stamp.sec +
                             message.header.stamp.nanosec * 1e-9)
                    if self.previous_yaw_sign and sign != self.previous_yaw_sign:
                        self.yaw_sign_changes += 1
                        if (self.previous_yaw_sign_time is not None and
                                stamp - self.previous_yaw_sign_time <= 0.5):
                            self.rapid_yaw_sign_changes += 1
                    self.previous_yaw_sign = sign
                    self.previous_yaw_sign_time = stamp


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    rclpy.init()
    node = Metrics()
    begin = time.monotonic()
    while rclpy.ok() and time.monotonic() - begin < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = max(1e-6, time.monotonic() - begin)
    print(f"map_rate_hz={node.maps / elapsed:.3f}")
    if node.last_revision is not None and node.last_map_stamp > node.first_map_stamp:
        revision_rate = ((node.last_revision - node.first_revision) /
                         (node.last_map_stamp - node.first_map_stamp))
        print(f"map_revision_rate_hz={revision_rate:.3f}")
        print(f"map_revision_span={node.first_revision}:{node.last_revision}")
    print(f"path_rate_hz={node.paths / elapsed:.3f}")
    print(f"nonempty_path_ratio={node.nonempty_paths / max(1, node.paths):.3f}")
    print(f"map_ms_p95={percentile(node.map_ms, 0.95):.3f}")
    print(f"plan_ms_p95={percentile(node.plan_ms, 0.95):.3f}")
    if node.map_ms:
        print(f"map_ms_mean={statistics.fmean(node.map_ms):.3f}")
    if node.plan_ms:
        print(f"plan_ms_mean={statistics.fmean(node.plan_ms):.3f}")
    print("planning_reasons=" + ",".join(
        f"{name}:{count}" for name, count in node.reasons.most_common()))
    print(f"cmd_rate_hz={node.commands / elapsed:.3f}")
    print(f"cmd_nonzero_ratio={node.nonzero_commands / max(1, node.commands):.3f}")
    print(f"cmd_max_abs=[{node.max_abs_vx:.3f},{node.max_abs_vy:.3f},{node.max_abs_wz:.3f}]")
    print("controller_states=" + ",".join(
        f"{name}:{count}" for name, count in node.controller_states.most_common()))
    print(f"controller_diagnostics={node.controller_diagnostics}")
    print(f"controller_max_abs_yaw_error={node.max_abs_yaw_error:.3f}")
    print(f"controller_large_yaw_samples={node.large_yaw_samples}")
    print(f"controller_max_linear_at_large_yaw={node.max_linear_at_large_yaw:.6f}")
    print(f"controller_yaw_sign_changes={node.yaw_sign_changes}")
    print(f"controller_rapid_yaw_sign_changes={node.rapid_yaw_sign_changes}")
    if node.first_path_invalidations is not None:
        print("path_invalidations=" + str(
            node.last_path_invalidations - node.first_path_invalidations))
    for reason, metrics in node.reason_metrics.items():
        if reason.endswith("success"):
            continue
        fields = []
        for key, values in metrics.items():
            finite = [value for value in values if value != float("inf")]
            if finite:
                fields.append(f"{key}_mean={statistics.fmean(finite):.3f}")
        print(f"failure_detail[{reason}]=" + ",".join(fields))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
