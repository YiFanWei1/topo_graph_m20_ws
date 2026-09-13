#!/usr/bin/env python3
"""Replay all validation bags through the M20 stack with motion disabled."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


BAGS = {
    'regu': Path('/home/wei/bag/regu'),
    'xili': Path('/home/wei/bag/xili/test_0911'),
    'full_nav_replay_with_plan': Path('/home/wei/bag/full_nav_replay_with_plan'),
}


class ReplayProbe(Node):
    def __init__(self) -> None:
        super().__init__('m20_bag_runtime_replay_probe')
        self.clouds = 0
        self.odometry = 0
        self.commands = 0
        self.maximum_command = 0.0
        self.disabled_statuses = 0
        self.create_subscription(
            PointCloud2, '/cloud_registered_body', self._cloud, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/lio_odom', self._odometry, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self._command, 20)
        self.create_subscription(
            String, '/route3d_m20_adapter/status', self._status, 10)

    def reset(self) -> None:
        self.clouds = 0
        self.odometry = 0
        self.commands = 0
        self.maximum_command = 0.0
        self.disabled_statuses = 0

    def _cloud(self, _message: PointCloud2) -> None:
        self.clouds += 1

    def _odometry(self, _message: Odometry) -> None:
        self.odometry += 1

    def _command(self, message: Twist) -> None:
        self.commands += 1
        self.maximum_command = max(
            self.maximum_command,
            abs(message.linear.x), abs(message.linear.y), abs(message.linear.z),
            abs(message.angular.x), abs(message.angular.y), abs(message.angular.z))

    def _status(self, message: String) -> None:
        status = json.loads(message.data)
        if status.get('block_reason') == 'motion_disabled':
            self.disabled_statuses += 1


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def start(command: list[str]) -> subprocess.Popen:
    return subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
        env=os.environ.copy(),
    )


def wait_for_discovery(probe: ReplayProbe, timeout: float = 12.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.count_publishers('/cmd_vel_smoothed') > 0 and all(
                probe.count_publishers(topic) > 0 for topic in (
                    '/route3d_pid_controller/swept_volume',
                    '/route3d_pid_controller/swept_collision_points',
                    '/route3d_pid_controller/nearest_hit_distance')):
            return
    raise RuntimeError('M20 launch did not expose required command/debug publishers')


def replay_one(
        probe: ReplayProbe, name: str, bag: Path, graph: Path, rate: float) -> dict:
    probe.reset()
    launch = start([
        'ros2', 'launch', 'route3d_m20_adapter', 'm20_pid_route.launch.xml',
        f'graph_file:={graph.resolve()}', 'enable_motion:=false',
        'start_m20_bridge:=false', 'launch_rviz:=false',
        'use_sim_time:=true',
    ])
    try:
        wait_for_discovery(probe)
        player = start([
            'ros2', 'bag', 'play', str(bag), '--clock', '--rate', str(rate),
        ])
        deadline = time.monotonic() + 90.0
        while player.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.03)
            if launch.poll() is not None:
                raise RuntimeError(f'M20 launch exited during {name} replay')
        if player.poll() is None:
            stop_process(player)
            raise RuntimeError(f'{name} replay exceeded timeout')
        if player.returncode != 0:
            raise RuntimeError(f'{name} ros2 bag play returned {player.returncode}')
        final_spin = time.monotonic() + 0.5
        while time.monotonic() < final_spin:
            rclpy.spin_once(probe, timeout_sec=0.03)
        if probe.clouds == 0 or probe.odometry == 0 or probe.commands == 0:
            raise AssertionError(
                f'{name}: missing traffic clouds={probe.clouds} '
                f'odometry={probe.odometry} commands={probe.commands}')
        if probe.maximum_command > 1.0e-12:
            raise AssertionError(
                f'{name}: enable_motion=false emitted {probe.maximum_command}')
        if probe.disabled_statuses == 0:
            raise AssertionError(f'{name}: adapter did not report motion_disabled')
        return {
            'bag': str(bag),
            'graph': str(graph),
            'playback_rate': rate,
            'cloud_messages_observed': probe.clouds,
            'odometry_messages_observed': probe.odometry,
            'zero_commands_observed': probe.commands,
            'maximum_absolute_command': probe.maximum_command,
            'motion_disabled_statuses': probe.disabled_statuses,
            'debug_publishers': {
                'swept_volume': probe.count_publishers(
                    '/route3d_pid_controller/swept_volume'),
                'collision_cloud': probe.count_publishers(
                    '/route3d_pid_controller/swept_collision_points'),
                'nearest_hit_distance': probe.count_publishers(
                    '/route3d_pid_controller/nearest_hit_distance'),
            },
            'stack_alive_after_replay': launch.poll() is None,
        }
    finally:
        stop_process(launch)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--graph-root', default='data/m20_validation')
    parser.add_argument('--rate', type=float, default=20.0)
    parser.add_argument('--output', default='validation/m20_bag_runtime_replay.json')
    args = parser.parse_args()
    if args.rate <= 0.0:
        raise ValueError('--rate must be positive')

    rclpy.init()
    probe = ReplayProbe()
    try:
        reports = {}
        for name, bag in BAGS.items():
            graph = Path(args.graph_root) / name / 'topoGraph_data.json'
            reports[name] = replay_one(probe, name, bag, graph, args.rate)
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    report = {
        'use_sim_time': True,
        'enable_motion': False,
        'recording_platform': 'Go2',
        'recording_body_height_m': 0.40,
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
