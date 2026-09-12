from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Request localization initialization from a Route3D vertex id.')
    parser.add_argument('vertex_id', type=int, help='Route3D vertex id, e.g. 57')
    parser.add_argument(
        '--topic', default='/route3d_initial_pose/vertex_id',
        help='request topic used by route3d_vertex_initializer')
    parser.add_argument(
        '--wait', type=float, default=2.0,
        help='seconds to wait for the initializer subscriber')
    parsed = parser.parse_args(args=args)

    rclpy.init()
    node = Node('route3d_vertex_initializer_cli')
    publisher = node.create_publisher(Int32, parsed.topic, 10)

    deadline = time.monotonic() + max(0.0, parsed.wait)
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    message = Int32()
    message.data = parsed.vertex_id
    publisher.publish(message)
    # Allow DDS time to transmit before shutting down this short-lived process.
    end = time.monotonic() + 0.25
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f'已请求使用 Route3D 点位 {parsed.vertex_id} 发布初始化位姿')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
