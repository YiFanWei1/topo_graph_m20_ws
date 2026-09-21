"""Record /lio_odom directly into a Route3D topology without point clouds."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from route3d_topology_core import PoseSample
from route_slope_annotator.slope_analysis import SlopeConfig
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from .topology_builder import BuilderSettings, OdomTopologyBuilder, atomic_write_json
from .visualization_markers import build_result_markers


def transient_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class OdomWaypointNode(Node):
    def __init__(self) -> None:
        super().__init__('route3d_odom_waypoint')
        self.odom_topic = str(self.declare_parameter('odom_topic', '/lio_odom').value)
        self.output_file = Path(str(self.declare_parameter(
            'output_file', 'topoGraph_odom.json').value)).expanduser().resolve()
        self.expected_frame = str(self.declare_parameter('expected_frame', '').value)
        self.expected_child_frame = str(self.declare_parameter(
            'expected_child_frame', '').value)
        self.auto_start = bool(self.declare_parameter('auto_start', True).value)
        snapshot_interval = float(self.declare_parameter(
            'snapshot_interval_s', 1.0).value)
        if not math.isfinite(snapshot_interval) or snapshot_interval <= 0.0:
            raise ValueError('snapshot_interval_s must be positive and finite')

        self.settings = BuilderSettings(
            target_spacing=float(self.declare_parameter('target_spacing', 1.0).value),
            dedup_distance=float(self.declare_parameter('dedup_distance', 0.05).value),
            body_height=float(self.declare_parameter('body_height', 0.57).value),
            relocation_distance=float(self.declare_parameter(
                'relocation_distance', 0.50).value),
            retrace_enabled=bool(self.declare_parameter('retrace_enabled', True).value),
            geometric_loop_closure_enabled=bool(self.declare_parameter(
                'geometric_loop_closure_enabled', True).value),
            loop_corridor_xy_tolerance=float(self.declare_parameter(
                'loop_closure.corridor_xy_tolerance', 0.40).value),
            loop_corridor_exit_xy_tolerance=float(self.declare_parameter(
                'loop_closure.corridor_exit_xy_tolerance', 0.55).value),
            loop_z_tolerance=float(self.declare_parameter(
                'loop_closure.z_tolerance', 0.20).value),
            loop_heading_tolerance_degrees=float(self.declare_parameter(
                'loop_closure.heading_tolerance_degrees', 45.0).value),
            loop_confirmation_distance=float(self.declare_parameter(
                'loop_closure.confirmation_distance', 0.80).value),
            loop_exit_confirmation_distance=float(self.declare_parameter(
                'loop_closure.exit_confirmation_distance', 0.80).value),
            loop_minimum_graph_separation=float(self.declare_parameter(
                'loop_closure.minimum_graph_separation', 3.00).value),
            loop_minimum_time_separation=float(self.declare_parameter(
                'loop_closure.minimum_time_separation', 5.00).value),
            loop_vertex_snap_distance=float(self.declare_parameter(
                'loop_closure.vertex_snap_distance', 0.45).value),
            loop_rejection_cooldown_distance=float(self.declare_parameter(
                'loop_closure.rejection_cooldown_distance', 1.00).value),
            corner_xy_radius=float(self.declare_parameter(
                'corner_xy_radius', 0.50).value),
            corner_yaw_degrees=float(self.declare_parameter(
                'corner_yaw_degrees', 45.0).value),
            corner_min_duration=float(self.declare_parameter(
                'corner_min_duration', 0.50).value),
            corner_max_z_range=float(self.declare_parameter(
                'corner_max_z_range', 0.10).value),
            corner_merge_distance=float(self.declare_parameter(
                'corner_merge_distance', 0.35).value),
            slope_enabled=bool(self.declare_parameter('slope_enabled', True).value),
            slope_config=SlopeConfig(
                fit_radius=float(self.declare_parameter('slope.fit_radius', 2.0).value),
                grade_threshold=float(self.declare_parameter(
                    'slope.grade_threshold', 0.12).value),
                minimum_core_length=float(self.declare_parameter(
                    'slope.minimum_core_length', 2.0).value),
                minimum_height_change=float(self.declare_parameter(
                    'slope.minimum_height_change', 0.20).value),
                maximum_core_gap=float(self.declare_parameter(
                    'slope.maximum_core_gap', 1.0).value),
                buffer_distance=float(self.declare_parameter(
                    'slope.buffer_distance', 2.0).value),
            ),
            obstacle_mode=int(self.declare_parameter('obstacle_mode', 0).value),
        )
        self.settings.validate()
        self.topology = OdomTopologyBuilder(self.settings)
        self.recording = self.auto_start
        self.dirty = False
        self.last_stamp: Optional[float] = None
        self.frame_id = ''
        self.child_frame_id = ''
        self.raw_path = PathMessage()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Odometry, self.odom_topic, self.on_odometry, sensor_qos)
        self.path_publisher = self.create_publisher(
            PathMessage, '~/path', transient_qos())
        self.visualization_publisher = self.create_publisher(
            MarkerArray, '~/visualization', transient_qos())
        self.status_publisher = self.create_publisher(String, '~/status', transient_qos())
        self.start_service = self.create_service(Trigger, '~/start', self.on_start)
        self.save_service = self.create_service(Trigger, '~/stop_and_save', self.on_save)
        self.reset_service = self.create_service(Trigger, '~/reset', self.on_reset)
        self.snapshot_timer = self.create_timer(snapshot_interval, self.write_snapshot)
        self.publish_status('recording' if self.recording else 'idle')
        self.get_logger().info(
            f'odom-only waypoint recorder: topic={self.odom_topic} '
            f'output={self.output_file} recording={self.recording}')

    def _new_session(self) -> None:
        self.topology = OdomTopologyBuilder(self.settings)
        self.dirty = False
        self.last_stamp = None
        self.frame_id = ''
        self.child_frame_id = ''
        self.raw_path = PathMessage()

    def on_odometry(self, message: Odometry) -> None:
        if not self.recording or self.topology.finalized:
            return
        frame = message.header.frame_id
        child = message.child_frame_id
        if self.expected_frame and frame != self.expected_frame:
            self.get_logger().error(
                f'忽略里程计：frame_id={frame}，期望 {self.expected_frame}')
            return
        if self.expected_child_frame and child != self.expected_child_frame:
            self.get_logger().error(
                f'忽略里程计：child_frame_id={child}，期望 {self.expected_child_frame}')
            return
        if not self.frame_id:
            self.frame_id = frame or self.expected_frame or 'camera_init'
            self.child_frame_id = child
            self.raw_path.header.frame_id = self.frame_id
        elif frame != self.frame_id or child != self.child_frame_id:
            self.get_logger().error('忽略坐标系发生变化的里程计消息')
            return

        stamp = float(message.header.stamp.sec) + \
            float(message.header.stamp.nanosec) * 1.0e-9
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.get_logger().warning('忽略时间戳未递增的里程计消息')
            return
        pose = message.pose.pose
        try:
            sample = PoseSample(
                stamp,
                (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
                (
                    float(pose.orientation.x), float(pose.orientation.y),
                    float(pose.orientation.z), float(pose.orientation.w),
                ),
                self.topology.sample_count,
            ).normalized()
            self.topology.add(sample)
        except ValueError as error:
            self.get_logger().warning(f'忽略无效里程计：{error}')
            return

        self.last_stamp = stamp
        stamped = PoseStamped()
        stamped.header = message.header
        stamped.pose = pose
        self.raw_path.header.stamp = message.header.stamp
        self.raw_path.poses.append(stamped)
        self.path_publisher.publish(self.raw_path)
        self.dirty = True

    def write_snapshot(self) -> None:
        if not self.dirty or self.topology.sample_count == 0:
            return
        try:
            document = self.topology.document(self.frame_id or 'camera_init')
            atomic_write_json(self.output_file, document)
            self.publish_visualization(document)
            self.dirty = False
            self.publish_status('recording' if self.recording else 'idle')
        except (OSError, ValueError, RuntimeError) as error:
            self.get_logger().error(f'写入实时拓扑失败：{error}')

    def publish_visualization(self, document: dict) -> None:
        markers = build_result_markers(
            self.topology.builder.raw_samples,
            document,
            self.frame_id or self.expected_frame or 'camera_init',
            self.settings.body_height,
            clear_existing=True,
        )
        now = self.get_clock().now().to_msg()
        for marker in markers.markers:
            marker.header.stamp = now
        self.visualization_publisher.publish(markers)

    def finish(self) -> bool:
        if self.topology.sample_count == 0:
            return False
        if not self.topology.finalized:
            document = self.topology.document(
                self.frame_id or self.expected_frame or 'camera_init', finalize=True)
            atomic_write_json(self.output_file, document)
            self.publish_visualization(document)
        self.dirty = False
        return True

    def publish_status(self, state: str) -> None:
        message = String()
        message.data = json.dumps({
            'state': state,
            'recording': self.recording,
            'poses': self.topology.sample_count,
            'vertices': len(self.topology.builder.vertices),
            'edges': len(self.topology.builder.edges),
            'output': str(self.output_file),
            'point_cloud_used': False,
            'loop_closure_mode': (
                'geometry_only' if self.settings.geometric_loop_closure_enabled
                else 'disabled'),
        }, ensure_ascii=False)
        self.status_publisher.publish(message)

    def on_start(self, _request, response):
        if self.topology.finalized:
            response.success = False
            response.message = '当前会话已保存，请先调用 reset'
        else:
            self.recording = True
            response.success = True
            response.message = '已开始记录 /lio_odom'
        self.publish_status('recording' if self.recording else 'complete')
        return response

    def on_save(self, _request, response):
        self.recording = False
        try:
            response.success = self.finish()
            response.message = (
                f'已保存到 {self.output_file}' if response.success
                else '没有收到有效的里程计数据')
        except (OSError, ValueError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
        self.publish_status('complete' if response.success else 'idle')
        return response

    def on_reset(self, _request, response):
        self.recording = False
        self._new_session()
        empty_document = self.topology.document(
            self.expected_frame or 'camera_init')
        self.publish_visualization(empty_document)
        self.raw_path.header.frame_id = self.expected_frame or 'camera_init'
        self.raw_path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.raw_path)
        response.success = True
        response.message = '已清空内存中的当前会话；输出文件将在下次写入时覆盖'
        self.publish_status('idle')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomWaypointNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.finish():
                if rclpy.ok():
                    node.get_logger().info(f'已保存无点云拓扑：{node.output_file}')
        except (OSError, ValueError, RuntimeError) as error:
            if rclpy.ok():
                node.get_logger().error(f'退出时保存失败：{error}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
