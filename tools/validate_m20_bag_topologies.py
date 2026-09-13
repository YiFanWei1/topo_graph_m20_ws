#!/usr/bin/env python3
"""Validate regenerated bag topologies through the production Dijkstra node."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray, String


BAG_NAMES = ('regu', 'xili', 'full_nav_replay_with_plan')


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class PlannerProbe(Node):
    def __init__(self) -> None:
        super().__init__('m20_bag_topology_validation_probe')
        self.statuses: list[dict] = []
        self.request = self.create_publisher(
            Int32MultiArray, '/route3d_dijkstra/plan_request', 10)
        self.create_subscription(
            String, '/route3d_dijkstra/status', self._on_status, latched_qos())

    def _on_status(self, message: String) -> None:
        self.statuses.append(json.loads(message.data))

    def plan(self, start: int, goal: int) -> dict:
        message = Int32MultiArray()
        message.data = [start, goal]
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.request.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            for status in reversed(self.statuses):
                if status.get('start_id') == start and status.get('goal_id') == goal:
                    if not status.get('success'):
                        raise AssertionError(
                            f'Dijkstra {start}->{goal} failed: {status.get("error")}')
                    return status
        raise RuntimeError(f'timed out waiting for Dijkstra {start}->{goal}')


def start_planner(graph: Path) -> subprocess.Popen:
    return subprocess.Popen(
        [
            'ros2', 'run', 'route3d_dijkstra_planner', 'dijkstra_planner_node',
            '--ros-args', '-p', f'graph_file:={graph}', '-p', 'strict_schema_v2:=true',
            '-p', 'request.odometry_body_height_m:=0.57',
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
        env=os.environ.copy(),
    )


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3.0)


def validate_graph(probe: PlannerProbe, path: Path) -> dict:
    document = json.loads(path.read_text(encoding='utf-8'))
    vertices = document['vertices']
    edges = document['edges']
    vertex_ids = sorted(int(value) for value in vertices)
    if len(vertex_ids) < 2 or not edges:
        raise AssertionError(f'{path}: topology is too small')
    obstacle_modes = sorted({edge['meta']['obstacleMode'] for edge in edges.values()})
    align_values = sorted({vertex['alignFinalYaw'] for vertex in vertices.values()})
    if obstacle_modes != [1]:
        raise AssertionError(f'{path}: obstacle modes are {obstacle_modes}, expected [1]')
    if align_values != [False]:
        raise AssertionError(f'{path}: alignFinalYaw values are {align_values}')
    if 'locomotionMode' in json.dumps(document, separators=(',', ':')):
        raise AssertionError(f'{path}: locomotionMode remains in regenerated graph')

    probe.statuses.clear()
    process = start_planner(path.resolve())
    try:
        forward = probe.plan(vertex_ids[0], vertex_ids[-1])
        reverse = probe.plan(vertex_ids[-1], vertex_ids[0])
    finally:
        stop_process(process)
    return {
        'vertices': len(vertices),
        'edges': len(edges),
        'obstacle_modes': obstacle_modes,
        'align_final_yaw_values': align_values,
        'locomotion_mode_fields': 0,
        'forward': {
            'start': forward['start_id'],
            'goal': forward['goal_id'],
            'path_vertices': len(forward['vertex_ids']),
            'path_edges': len(forward['edge_ids']),
            'cost': forward['total_cost'],
            'planning_time_ms': forward['planning_time_ms'],
        },
        'reverse': {
            'start': reverse['start_id'],
            'goal': reverse['goal_id'],
            'path_vertices': len(reverse['vertex_ids']),
            'path_edges': len(reverse['edge_ids']),
            'cost': reverse['total_cost'],
            'planning_time_ms': reverse['planning_time_ms'],
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default='data/m20_validation')
    parser.add_argument('--output', default='validation/m20_bag_topology_plans.json')
    args = parser.parse_args()

    rclpy.init()
    probe = PlannerProbe()
    try:
        reports = {
            name: validate_graph(
                probe, Path(args.root) / name / 'topoGraph_data.json')
            for name in BAG_NAMES
        }
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    report = {
        'recording_platform': 'Go2',
        'recording_body_height_m': 0.40,
        'target_platform': 'M20',
        'm20_body_height_m': 0.57,
        'bags': reports,
        'passed': True,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
