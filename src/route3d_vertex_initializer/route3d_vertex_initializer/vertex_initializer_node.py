from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

from .graph_loader import GraphSnapshot, load_graph, quaternion_from_rpy


class VertexInitializerNode(Node):
    """Publish /initialpose from a Route3D vertex.

    Graph source priority:
      1. graph_file_topic published by the web console (exact topology selected in browser)
      2. current planner graph_file parameter
      3. fallback_graph_file from YAML
    """

    def __init__(self) -> None:
        super().__init__('route3d_vertex_initializer')

        self.planner_node = self.declare_parameter(
            'planner_node', '/route3d_dijkstra_planner').value
        self.planner_graph_parameter = self.declare_parameter(
            'planner_graph_parameter', 'graph_file').value
        self.fallback_graph_file = self.declare_parameter(
            'fallback_graph_file', '').value
        self.graph_file_topic = self.declare_parameter(
            'graph_file_topic', '/route3d_initial_pose/graph_file').value
        self.request_topic = self.declare_parameter(
            'request_topic', '/route3d_initial_pose/vertex_id').value
        self.initial_pose_topic = self.declare_parameter(
            'initial_pose_topic', '/initialpose').value
        self.selected_pose_topic = self.declare_parameter(
            'selected_pose_topic', '/route3d_initial_pose/selected_pose').value
        self.status_topic = self.declare_parameter(
            'status_topic', '/route3d_initial_pose/status').value
        self.refresh_period_s = float(self.declare_parameter(
            'graph_refresh_period_s', 1.0).value)
        self.prefer_graph_file_topic = bool(self.declare_parameter(
            'prefer_graph_file_topic', True).value)

        self.publish_frame_id = str(self.declare_parameter(
            'initial_pose.frame_id', 'map').value)
        self.use_graph_frame_when_empty = bool(self.declare_parameter(
            'initial_pose.use_graph_frame_when_empty', True).value)
        self.use_vertex_z = bool(self.declare_parameter(
            'initial_pose.use_vertex_z', True).value)
        self.use_full_rpy = bool(self.declare_parameter(
            'initial_pose.use_full_rpy', True).value)
        self.z_override = float(self.declare_parameter(
            'initial_pose.z_override', 0.0).value)

        self.cov_x = float(self.declare_parameter('covariance.x', 0.25).value)
        self.cov_y = float(self.declare_parameter('covariance.y', 0.25).value)
        self.cov_z = float(self.declare_parameter('covariance.z', 0.25).value)
        self.cov_roll = float(self.declare_parameter('covariance.roll', 0.0685).value)
        self.cov_pitch = float(self.declare_parameter('covariance.pitch', 0.0685).value)
        self.cov_yaw = float(self.declare_parameter('covariance.yaw', 0.0685).value)

        self.graph: Optional[GraphSnapshot] = None
        self.graph_mtime_ns: Optional[int] = None
        self.pending_parameter_request = False
        self.last_remote_graph_file = ''
        self.web_graph_file = ''
        self.graph_source = ''

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initial_pose_topic, 10)
        self.selected_pose_pub = self.create_publisher(
            PoseStamped, self.selected_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.request_sub = self.create_subscription(
            Int32, self.request_topic, self._vertex_request_callback, 10)

        # Transient-local makes the most recently selected browser topology available
        # even if this node starts after route3d_web_console.
        graph_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.graph_file_sub = self.create_subscription(
            String, self.graph_file_topic, self._graph_file_callback, graph_qos)

        service_name = self.planner_node.rstrip('/') + '/get_parameters'
        self.parameter_client = self.create_client(GetParameters, service_name)

        if self.fallback_graph_file:
            self._load_graph(self.fallback_graph_file, source='fallback')

        self.refresh_timer = self.create_timer(
            max(0.2, self.refresh_period_s), self._refresh_graph)

        self.get_logger().info(
            f'Route3D vertex initializer ready: request={self.request_topic} '
            f'graph_topic={self.graph_file_topic} initial_pose={self.initial_pose_topic}')
        self._publish_status('waiting_for_graph' if self.graph is None else 'ready')

    def _publish_status(self, state: str, **extra) -> None:
        payload = {
            'state': state,
            'graph_file': str(self.graph.path) if self.graph else '',
            'graph_frame': self.graph.frame_id if self.graph else '',
            'graph_source': self.graph_source,
            'vertex_count': len(self.graph.vertices) if self.graph else 0,
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(message)

    def _graph_file_callback(self, message: String) -> None:
        graph_file = message.data.strip()
        if not graph_file:
            self.web_graph_file = ''
            self.get_logger().info('Web topology override cleared; planner parameter is active again')
            return
        self.web_graph_file = graph_file
        # Always reload: Web Console republishes the same path after saving edits in place.
        self._load_graph(graph_file, source='web_console')

    def _refresh_graph(self) -> None:
        # Reload the current file if edited in place by the topology editor.
        if self.graph is not None:
            try:
                mtime_ns = os.stat(self.graph.path).st_mtime_ns
                if self.graph_mtime_ns is not None and mtime_ns != self.graph_mtime_ns:
                    self._load_graph(str(self.graph.path), source=f'{self.graph_source}_file_changed')
            except OSError:
                pass

        if self.prefer_graph_file_topic and self.web_graph_file:
            return
        self._refresh_graph_from_planner()

    def _refresh_graph_from_planner(self) -> None:
        if self.pending_parameter_request or not self.parameter_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = [str(self.planner_graph_parameter)]
        self.pending_parameter_request = True
        future = self.parameter_client.call_async(request)
        future.add_done_callback(self._planner_parameter_response)

    def _planner_parameter_response(self, future) -> None:
        self.pending_parameter_request = False
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed reading planner graph_file: {exc}')
            return
        if not response.values:
            return
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            self.get_logger().warn(
                f'{self.planner_node}.{self.planner_graph_parameter} is not a string parameter')
            return
        graph_file = value.string_value.strip()
        if not graph_file:
            return
        if graph_file != self.last_remote_graph_file or self.graph is None:
            self.last_remote_graph_file = graph_file
            self._load_graph(graph_file, source='planner_parameter')

    def _load_graph(self, graph_file: str, source: str) -> None:
        try:
            snapshot = load_graph(graph_file)
            mtime_ns = os.stat(snapshot.path).st_mtime_ns
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Cannot load topology graph {graph_file}: {exc}')
            self._publish_status('graph_load_error', error=str(exc), source=source)
            return

        changed = self.graph is None or self.graph.path != snapshot.path
        self.graph = snapshot
        self.graph_mtime_ns = mtime_ns
        self.graph_source = source
        if changed:
            self.get_logger().info(
                f'Loaded topology: {snapshot.path} frame={snapshot.frame_id} '
                f'vertices={len(snapshot.vertices)} source={source}')
            if self.publish_frame_id and self.publish_frame_id != snapshot.frame_id:
                self.get_logger().warn(
                    f'Publishing initial pose in frame "{self.publish_frame_id}" while graph frame '
                    f'is "{snapshot.frame_id}". Pose values are copied directly; no TF transform is applied.')
        self._publish_status('ready', source=source)

    def _vertex_request_callback(self, message: Int32) -> None:
        vertex_id = int(message.data)
        if self.graph is None:
            self.get_logger().error(
                f'Cannot initialize from vertex {vertex_id}: topology graph is not loaded yet')
            self._publish_status('request_rejected', vertex_id=vertex_id, reason='graph_not_loaded')
            return

        vertex = self.graph.vertices.get(vertex_id)
        if vertex is None:
            available = sorted(self.graph.vertices)
            self.get_logger().error(
                f'Vertex {vertex_id} does not exist; valid range/list begins with {available[:20]}')
            self._publish_status('request_rejected', vertex_id=vertex_id, reason='vertex_not_found')
            return

        x, y, z = vertex.position
        roll, pitch, yaw = vertex.rpy
        if not self.use_vertex_z:
            z = self.z_override
        if not self.use_full_rpy:
            roll = 0.0
            pitch = 0.0

        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        frame_id = self.publish_frame_id.strip()
        if not frame_id and self.use_graph_frame_when_empty:
            frame_id = self.graph.frame_id
        if not frame_id:
            frame_id = 'map'

        initial = PoseWithCovarianceStamped()
        initial.header.stamp = self.get_clock().now().to_msg()
        initial.header.frame_id = frame_id
        initial.pose.pose.position.x = x
        initial.pose.pose.position.y = y
        initial.pose.pose.position.z = z
        initial.pose.pose.orientation.x = qx
        initial.pose.pose.orientation.y = qy
        initial.pose.pose.orientation.z = qz
        initial.pose.pose.orientation.w = qw
        initial.pose.covariance[0] = self.cov_x
        initial.pose.covariance[7] = self.cov_y
        initial.pose.covariance[14] = self.cov_z
        initial.pose.covariance[21] = self.cov_roll
        initial.pose.covariance[28] = self.cov_pitch
        initial.pose.covariance[35] = self.cov_yaw
        self.initial_pose_pub.publish(initial)

        selected = PoseStamped()
        selected.header = initial.header
        selected.pose = initial.pose.pose
        self.selected_pose_pub.publish(selected)

        self.get_logger().info(
            'Published vertex initial pose: id=%d frame=%s pos=(%.3f, %.3f, %.3f) '
            'rpy=(%.3f, %.3f, %.3f)',
            vertex_id, frame_id, x, y, z, roll, pitch, yaw)
        self._publish_status(
            'published', vertex_id=vertex_id, frame_id=frame_id,
            position=[x, y, z], rpy=[roll, pitch, yaw])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VertexInitializerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
