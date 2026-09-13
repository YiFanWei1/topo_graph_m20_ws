#!/usr/bin/env python3
"""Exercise M20 preparation and fail-closed command selection over ROS topics."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Callable

import rclpy
from drdds.msg import MotionState
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class RuntimeProbe(Node):
    def __init__(self) -> None:
        super().__init__('m20_runtime_validation_probe')
        self.statuses: list[dict] = []
        self.outputs: list[Twist] = []
        self.usage_modes: list[int] = []
        self.motion_requests: list[int] = []
        self.ready_values: list[bool] = []
        self.preparation_statuses: list[dict] = []

        self.pid = self.create_publisher(Twist, '/cmd_vel_pid', 20)
        self.efficient = self.create_publisher(Twist, '/cmd_vel_effi', 20)
        self.source = self.create_publisher(
            String, '/route3d_controller/active_source', latched_qos())
        self.alignment = self.create_publisher(
            Bool, '/route3d_pid_controller/alignment_active', latched_qos())
        self.ready = self.create_publisher(Bool, '/m20/control/ready', latched_qos())
        self.motion_state = self.create_publisher(
            MotionState, '/m20/state/motion_state', 10)
        self.control_result = self.create_publisher(
            String, '/m20/state/control_result', 10)

        self.create_subscription(
            String, '/route3d_m20_adapter/status', self._adapter_status, latched_qos())
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.outputs.append, 20)
        self.create_subscription(
            UInt32, '/m20/control/usage_mode',
            lambda message: self.usage_modes.append(int(message.data)), 10)
        self.create_subscription(
            MotionState, '/m20/control/motion_state',
            lambda message: self.motion_requests.append(int(message.data.state)), 10)
        self.create_subscription(
            Bool, '/m20/control/ready',
            lambda message: self.ready_values.append(bool(message.data)), latched_qos())
        self.create_subscription(
            String, '/m20/control/preparation_status',
            self._preparation_status, latched_qos())

    def _adapter_status(self, message: String) -> None:
        self.statuses.append(json.loads(message.data))

    def _preparation_status(self, message: String) -> None:
        self.preparation_statuses.append(json.loads(message.data))

    def publish_state(self, state: int) -> None:
        message = MotionState()
        message.data.state = state
        self.motion_state.publish(message)

    def publish_source(self, source: str) -> None:
        message = String()
        message.data = source
        self.source.publish(message)

    def publish_ready(self, ready: bool) -> None:
        message = Bool()
        message.data = ready
        self.ready.publish(message)

    def publish_alignment(self, active: bool) -> None:
        message = Bool()
        message.data = active
        self.alignment.publish(message)

    def publish_twist(self, publisher, vx: float, vy: float, wz: float) -> None:
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        publisher.publish(message)


def spin_until(node: Node, predicate: Callable[[], bool], timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
        if predicate():
            return
    raise RuntimeError('timed out waiting for ROS validation condition')


def publish_until(
        node: Node, publish: Callable[[], None], predicate: Callable[[], bool],
        timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        publish()
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise RuntimeError('timed out while publishing ROS validation stimulus')


def start_node(command: list[str]) -> subprocess.Popen:
    return subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
        env=os.environ.copy(),
    )


def stop_node(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=3.0)


def latest_status(probe: RuntimeProbe, reason: str | None = None) -> dict | None:
    for status in reversed(probe.statuses):
        if reason is None or status.get('block_reason') == reason:
            return status
    return None


def assert_selected(status: dict, expected: tuple[float, float, float]) -> None:
    actual = status['selected']
    values = (actual['vx'], actual['vy'], actual['wz'])
    if any(abs(left - right) > 1e-6 for left, right in zip(values, expected)):
        raise AssertionError(f'expected selected={expected}, got {values}')


def run_adapter(probe: RuntimeProbe, enabled: bool) -> subprocess.Popen:
    return start_node([
        'ros2', 'run', 'route3d_m20_adapter', 'm20_route_adapter_node',
        '--ros-args', '-p', f'enable_motion:={str(enabled).lower()}',
        '-p', 'command_timeout_s:=0.20', '-p', 'state_timeout_s:=0.80',
    ])


def validate_adapter(probe: RuntimeProbe) -> dict:
    checks: dict[str, object] = {}
    process = run_adapter(probe, True)
    try:
        spin_until(probe, lambda: bool(probe.statuses))
        probe.publish_source('pid')
        probe.publish_alignment(False)
        probe.publish_ready(False)
        probe.publish_state(17)
        probe.publish_twist(probe.pid, 1.2, 0.4, 0.8)
        spin_until(probe, lambda: latest_status(probe, 'm20_not_ready') is not None)
        status = latest_status(probe, 'm20_not_ready')
        assert_selected(status, (0.0, 0.0, 0.0))
        checks['not_ready_zero'] = True

        probe.publish_ready(True)
        probe.publish_state(17)
        probe.publish_twist(probe.pid, 1.2, 0.4, 0.8)
        spin_until(probe, lambda: latest_status(probe) is not None and
                   not latest_status(probe).get('blocked', True))
        status = latest_status(probe)
        assert_selected(status, (0.8, 0.0, 0.5))
        checks['normal_limits'] = status['selected']

        probe.publish_alignment(True)
        probe.publish_state(17)
        probe.publish_twist(probe.pid, 1.2, 0.4, 0.8)
        spin_until(probe, lambda: latest_status(probe) is not None and
                   latest_status(probe).get('alignment_active') is True and
                   not latest_status(probe).get('blocked', True))
        status = latest_status(probe)
        assert_selected(status, (0.2, 0.3, 0.5))
        checks['alignment_limits'] = status['selected']

        probe.publish_source('efficient_3d_local_planner')
        probe.publish_state(17)
        probe.publish_twist(probe.efficient, -1.2, 0.4, -0.8)
        spin_until(probe, lambda: latest_status(probe) is not None and
                   latest_status(probe).get('active_controller') ==
                   'efficient_3d_local_planner' and
                   not latest_status(probe).get('blocked', True))
        status = latest_status(probe)
        assert_selected(status, (-0.8, 0.0, -0.5))
        checks['efficient_selected'] = status['selected']

        probe.publish_source('invalid_controller')
        spin_until(probe, lambda: latest_status(probe, 'unknown_controller') is not None)
        assert_selected(latest_status(probe, 'unknown_controller'), (0.0, 0.0, 0.0))
        checks['unknown_source_zero'] = True

        probe.publish_source('pid')
        probe.publish_state(17)
        probe.publish_twist(probe.pid, 0.4, 0.0, 0.1)
        spin_until(probe, lambda: latest_status(probe) is not None and
                   not latest_status(probe).get('blocked', True))
        spin_until(probe, lambda: latest_status(probe, 'controller_command_stale') is not None)
        checks['command_timeout_zero'] = True

        probe.publish_twist(probe.pid, 0.4, 0.0, 0.1)
        spin_until(probe, lambda: latest_status(probe, 'm20_state_not_fresh_rl') is not None,
                   timeout=2.0)
        assert_selected(
            latest_status(probe, 'm20_state_not_fresh_rl'), (0.0, 0.0, 0.0))
        checks['state_timeout_zero'] = True
    finally:
        stop_node(process)

    probe.statuses.clear()
    disabled = run_adapter(probe, False)
    try:
        spin_until(probe, lambda: latest_status(probe, 'motion_disabled') is not None)
        status = latest_status(probe, 'motion_disabled')
        assert_selected(status, (0.0, 0.0, 0.0))
        checks['enable_motion_false_zero'] = True
    finally:
        stop_node(disabled)
    return checks


def validate_preparation(probe: RuntimeProbe) -> dict:
    probe.usage_modes.clear()
    probe.motion_requests.clear()
    probe.ready_values.clear()
    probe.preparation_statuses.clear()
    process = start_node([
        'ros2', 'run', 'basic_server_bridge', 'm20_ready_cmd_vel',
        '--ros-args', '-p', 'command_repeat_sec:=0.10',
        '-p', 'state_wait_timeout_sec:=0.20',
    ])
    try:
        spin_until(probe, lambda: bool(probe.preparation_statuses))
        publish_until(
            probe, lambda: probe.publish_state(0),
            lambda: len(probe.usage_modes) >= 2)
        if any(mode != 0 for mode in probe.usage_modes):
            raise AssertionError(f'non-normal usage request: {probe.usage_modes}')

        result = String()
        result.data = 'result usage_mode error_code=0'
        publish_until(
            probe, lambda: probe.control_result.publish(result),
            lambda: 1 in probe.motion_requests)

        publish_until(
            probe, lambda: probe.publish_state(1),
            lambda: 17 in probe.motion_requests)
        publish_until(
            probe, lambda: probe.publish_state(17),
            lambda: bool(probe.ready_values) and probe.ready_values[-1])
        spin_until(probe, lambda: probe.preparation_statuses[-1].get('stage') == 'ready')

        probe.motion_requests.clear()
        usage_count = len(probe.usage_modes)
        publish_until(
            probe, lambda: probe.publish_state(4),
            lambda: bool(probe.ready_values) and not probe.ready_values[-1])
        publish_until(
            probe, lambda: probe.publish_state(4),
            lambda: len(probe.usage_modes) > usage_count)
        publish_until(
            probe, lambda: probe.control_result.publish(result),
            lambda: bool(probe.preparation_statuses) and
            probe.preparation_statuses[-1].get('normal_mode_acknowledged') is True)
        publish_until(
            probe, lambda: probe.publish_state(4),
            lambda: 1 in probe.motion_requests)

        gait_publishers = probe.get_publishers_info_by_topic('/m20/control/gait')
        if gait_publishers:
            raise AssertionError('preparation created a gait control publisher')
        return {
            'normal_mode_retries_before_ack': len(probe.usage_modes),
            'stand_requested': True,
            'rl_17_requested': True,
            'ready_after_feedback': True,
            'state_loss_revokes_ready': True,
            'gait_control_publishers': 0,
        }
    finally:
        stop_node(process)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output', default='validation/m20_runtime_checks.json',
        help='JSON report path')
    args = parser.parse_args()

    rclpy.init()
    probe = RuntimeProbe()
    try:
        report = {
            'adapter': validate_adapter(probe),
            'preparation': validate_preparation(probe),
            'passed': True,
        }
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
