"""Publish raw pose trajectory and attributed topology markers for RViz."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from .topology_builder import load_pose_file
from .visualization_markers import build_result_markers


class FileResultVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('route3d_odom_waypoint_visualizer')
        pose_file = Path(str(self.declare_parameter('pose_file', '').value)).expanduser()
        topology_file = Path(str(
            self.declare_parameter('topology_file', '').value)).expanduser()
        self.body_height = float(self.declare_parameter('body_height', 0.57).value)
        requested_frame = str(self.declare_parameter('frame_id', '').value).strip()
        self.show_vertex_labels = bool(self.declare_parameter(
            'show_vertex_labels', True).value)
        self.show_edge_labels = bool(self.declare_parameter(
            'show_edge_labels', True).value)
        self.vertex_scale = float(self.declare_parameter('vertex_scale', 0.20).value)
        self.label_scale = float(self.declare_parameter('label_scale', 0.16).value)

        if not pose_file.is_file():
            raise ValueError(f'pose file does not exist: {pose_file}')
        if not topology_file.is_file():
            raise ValueError(f'topology file does not exist: {topology_file}')
        if not all(math.isfinite(value) and value >= 0.0 for value in (
                self.body_height, self.vertex_scale, self.label_scale)):
            raise ValueError('visualization sizes and body_height must be finite and non-negative')

        self.samples = load_pose_file(pose_file)
        self.document = json.loads(topology_file.read_text(encoding='utf-8'))
        if not isinstance(self.document.get('vertices'), dict) or \
                not isinstance(self.document.get('edges'), dict):
            raise ValueError('topology JSON must contain vertices and edges objects')
        self.frame_id = requested_frame or str(
            self.document.get('frame_id', 'camera_init'))
        self.markers = self.build_markers()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            MarkerArray, '/route3d_odom_waypoint/visualization', qos)
        self.timer = self.create_timer(1.0, self.publish)
        self.publish()
        self.get_logger().info(
            f'visualizing raw poses={len(self.samples)}, '
            f'vertices={len(self.document["vertices"])}, '
            f'edges={len(self.document["edges"])}, frame={self.frame_id}')

    def build_markers(self) -> MarkerArray:
        return build_result_markers(
            self.samples, self.document, self.frame_id, self.body_height,
            show_vertex_labels=self.show_vertex_labels,
            show_edge_labels=self.show_edge_labels,
            vertex_scale=self.vertex_scale,
            label_scale=self.label_scale,
        )

    def publish(self) -> None:
        now = self.get_clock().now().to_msg()
        for marker in self.markers.markers:
            marker.header.stamp = now
        self.publisher.publish(self.markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FileResultVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
