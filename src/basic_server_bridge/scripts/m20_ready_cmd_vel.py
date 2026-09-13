#!/usr/bin/env python3
"""Prepare M20 motion control without changing or waiting for a gait."""

from __future__ import annotations

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, UInt32

from drdds.msg import MotionState


MOTION_STAND = 1
MOTION_RL = 17
MODE_NORMAL = 0


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class M20ReadyCmdVel(Node):
    """Confirm normal usage mode, then put M20 into RL motion state."""

    def __init__(self) -> None:
        super().__init__("m20_ready_cmd_vel")
        self.declare_parameter("motion_state_topic", "/m20/state/motion_state")
        self.declare_parameter("usage_mode_topic", "/m20/control/usage_mode")
        self.declare_parameter("motion_state_control_topic", "/m20/control/motion_state")
        self.declare_parameter("control_result_topic", "/m20/state/control_result")
        self.declare_parameter("ready_topic", "/m20/control/ready")
        self.declare_parameter("status_topic", "/m20/control/preparation_status")
        self.declare_parameter("state_wait_timeout_sec", 10.0)
        self.declare_parameter("command_repeat_sec", 1.0)

        self._state_wait_timeout = float(self.get_parameter("state_wait_timeout_sec").value)
        self._command_repeat = float(self.get_parameter("command_repeat_sec").value)
        self._motion_state: Optional[int] = None
        self._started_at = time.monotonic()
        self._last_command_at = 0.0
        self._mode_acknowledged = False
        self._stage = "waiting_state"
        self._retry_count = 0
        self._last_result = ""
        self._ready = False

        self._mode_publisher = self.create_publisher(
            UInt32, self.get_parameter("usage_mode_topic").value, 10)
        self._motion_state_publisher = self.create_publisher(
            MotionState, self.get_parameter("motion_state_control_topic").value, 10)
        self._ready_publisher = self.create_publisher(
            Bool, self.get_parameter("ready_topic").value, latched_qos())
        self._status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, latched_qos())
        self.create_subscription(
            MotionState, self.get_parameter("motion_state_topic").value,
            self._on_motion_state, 10)
        self.create_subscription(
            String, self.get_parameter("control_result_topic").value,
            self._on_control_result, 10)
        self.create_timer(0.1, self._advance_preparation)

        self._publish_ready(False)
        self._publish_status()
        self.get_logger().info(
            "Waiting for M20 status; preparation changes usage/motion state only. "
            "No gait command is created by this node.")

    def _on_motion_state(self, message: MotionState) -> None:
        self._motion_state = int(message.data.state)
        if self._ready and self._motion_state != MOTION_RL:
            self._mode_acknowledged = False
            self._stage = "normal_mode"
            self._publish_ready(False)

    def _on_control_result(self, message: String) -> None:
        self._last_result = message.data
        if "result usage_mode" not in message.data:
            return
        if "error_code=" in message.data and "error_code=0" not in message.data:
            self.get_logger().error(f"Normal usage mode rejected: {message.data}")
            self._publish_status(error="usage_mode_rejected")
            return
        self._mode_acknowledged = True
        self.get_logger().info(f"Normal usage mode acknowledged: {message.data}")

    def _publish_ready(self, ready: bool) -> None:
        self._ready = ready
        message = Bool()
        message.data = ready
        self._ready_publisher.publish(message)

    def _publish_status(self, error: str = "") -> None:
        message = String()
        message.data = json.dumps({
            "ready": self._ready,
            "stage": self._stage,
            "motion_state": self._motion_state,
            "normal_mode_acknowledged": self._mode_acknowledged,
            "retry_count": self._retry_count,
            "last_result": self._last_result,
            "error": error,
        }, separators=(",", ":"))
        self._status_publisher.publish(message)

    def _publish_mode_normal(self) -> None:
        message = UInt32()
        message.data = MODE_NORMAL
        self._mode_publisher.publish(message)
        self.get_logger().info("Requested normal usage mode")

    def _publish_motion_state(self, state: int) -> None:
        message = MotionState()
        message.data.state = state
        self._motion_state_publisher.publish(message)
        self.get_logger().info(f"Requested motion state {state}")

    def _advance_preparation(self) -> None:
        now = time.monotonic()
        if self._motion_state is None:
            self._stage = "waiting_state"
            if now - self._started_at >= self._state_wait_timeout:
                self._publish_status(error="motion_state_timeout")
            return

        if not self._mode_acknowledged:
            self._stage = "normal_mode"
            if now - self._last_command_at >= self._command_repeat:
                self._last_command_at = now
                self._retry_count += 1
                self._publish_mode_normal()
                self._publish_status()
            return

        if self._motion_state == MOTION_RL:
            self._stage = "ready"
            if not self._ready:
                self._publish_ready(True)
                self.get_logger().info("M20 ready: normal usage mode and RL state confirmed")
            self._publish_status()
            return

        self._publish_ready(False)
        if now - self._last_command_at < self._command_repeat:
            return
        self._last_command_at = now
        self._retry_count += 1
        if self._motion_state != MOTION_STAND:
            self._stage = "stand"
            self._publish_motion_state(MOTION_STAND)
        else:
            self._stage = "rl"
            self._publish_motion_state(MOTION_RL)
        self._publish_status()


def main() -> None:
    rclpy.init()
    node = M20ReadyCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_ready(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
