"""Load a topoSingle JSON or legacy target.txt and publish a dense ground route."""

import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

from recorded_waypoint_route.route_utils import (
    interpolate_targets,
    load_route_file,
    TargetType,
)
from visualization_msgs.msg import Marker, MarkerArray


def route_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def default_target_file() -> str:
    return get_package_share_directory("recorded_waypoint_route") + "/config/target.txt"


def yaw_for_index(points, index: int) -> float:
    previous = max(0, index - 1)
    following = min(len(points) - 1, index + 1)
    return math.atan2(
        points[following][1] - points[previous][1],
        points[following][0] - points[previous][0],
    )


def pose_from_point(point, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = point
    pose.orientation.z = math.sin(0.5 * yaw)
    pose.orientation.w = math.cos(0.5 * yaw)
    return pose


class TargetPathGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__("target_path_generator")
        legacy_target_file = str(self.declare_parameter(
            "target_file", default_target_file()).value)
        route_file = str(self.declare_parameter("route_file", "").value).strip()
        selected_route_file = route_file or legacy_target_file
        fallback_frame = str(self.declare_parameter(
            "frames.fallback", "camera_init").value)
        spacing = float(self.declare_parameter("route.spacing", 0.10).value)
        body_height = float(self.declare_parameter("route.body_height", 0.40).value)
        normal_marker_diameter = float(self.declare_parameter(
            "visualization.normal_marker_diameter", 0.45).value)
        corner_marker_diameter = float(self.declare_parameter(
            "visualization.corner_marker_diameter", 0.75).value)
        if not math.isfinite(body_height) or body_height < 0.0:
            raise ValueError("route.body_height must be finite and non-negative")
        if not math.isfinite(normal_marker_diameter) or normal_marker_diameter <= 0.0:
            raise ValueError(
                "visualization.normal_marker_diameter must be finite and positive")
        if not math.isfinite(corner_marker_diameter) or corner_marker_diameter <= 0.0:
            raise ValueError(
                "visualization.corner_marker_diameter must be finite and positive")

        parsed = load_route_file(selected_route_file, fallback_frame)
        path_points = interpolate_targets(parsed.points, spacing)
        self.targets_publisher = self.create_publisher(
            PoseArray, "/recorded_waypoint_route/targets_ground", route_qos())
        self.path_publisher = self.create_publisher(
            Path, "/recorded_waypoint_route/full_path_ground", route_qos())
        self.types_publisher = self.create_publisher(
            UInt8MultiArray, "/recorded_waypoint_route/target_types", route_qos())
        self.slope_flags_publisher = self.create_publisher(
            UInt8MultiArray, "/recorded_waypoint_route/target_slope_flags", route_qos())
        self.labels_publisher = self.create_publisher(
            MarkerArray, "/recorded_waypoint_route/target_labels", route_qos())
        self.markers_publisher = self.create_publisher(
            MarkerArray, "/recorded_waypoint_route/target_markers", route_qos())

        stamp = self.get_clock().now().to_msg()
        targets_message = PoseArray()
        targets_message.header.frame_id = parsed.frame_id
        targets_message.header.stamp = stamp
        for index, point in enumerate(parsed.points):
            targets_message.poses.append(pose_from_point(
                point, yaw_for_index(parsed.points, index)))

        path_message = Path()
        path_message.header = targets_message.header
        for index, point in enumerate(path_points):
            stamped = PoseStamped()
            stamped.header = path_message.header
            stamped.pose = pose_from_point(point, yaw_for_index(path_points, index))
            path_message.poses.append(stamped)

        types_message = UInt8MultiArray()
        types_message.data = [
            1 if target_type == TargetType.CORNER else 0
            for target_type in parsed.types
        ]
        slope_flags_message = UInt8MultiArray()
        slope_flags_message.data = [int(is_slope) for is_slope in parsed.is_slope]

        labels_message = MarkerArray()
        markers_message = MarkerArray()
        normal_marker = Marker()
        normal_marker.header = targets_message.header
        normal_marker.ns = "route_normal_targets"
        normal_marker.id = 1
        normal_marker.type = Marker.SPHERE_LIST
        normal_marker.action = Marker.ADD
        normal_marker.pose.orientation.w = 1.0
        normal_marker.scale.x = normal_marker_diameter
        normal_marker.scale.y = normal_marker_diameter
        normal_marker.scale.z = normal_marker_diameter
        normal_marker.color.r = 1.0
        normal_marker.color.g = 0.86
        normal_marker.color.b = 0.05
        normal_marker.color.a = 0.95

        corner_marker = Marker()
        corner_marker.header = targets_message.header
        corner_marker.ns = "route_corner_targets"
        corner_marker.id = 2
        corner_marker.type = Marker.SPHERE_LIST
        corner_marker.action = Marker.ADD
        corner_marker.pose.orientation.w = 1.0
        corner_marker.scale.x = corner_marker_diameter
        corner_marker.scale.y = corner_marker_diameter
        corner_marker.scale.z = corner_marker_diameter
        corner_marker.color.r = 1.0
        corner_marker.color.g = 0.12
        corner_marker.color.b = 0.03
        corner_marker.color.a = 1.0

        for index, (point, target_type) in enumerate(zip(parsed.points, parsed.types)):
            display_point = pose_from_point(
                (point[0], point[1], point[2] + body_height), 0.0).position
            if target_type == TargetType.CORNER:
                corner_marker.points.append(display_point)
            else:
                normal_marker.points.append(display_point)
            marker = Marker()
            marker.header = targets_message.header
            marker.ns = "recorded_target_ids"
            marker.id = index + 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2] + body_height + 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.30 if target_type == TargetType.CORNER else 0.24
            marker.color.r = 1.0
            marker.color.g = 0.15 if target_type == TargetType.CORNER else 0.85
            marker.color.b = 0.03 if target_type == TargetType.CORNER else 0.05
            marker.color.a = 1.0
            marker.text = f"C{index + 1}" if target_type == TargetType.CORNER else str(index + 1)
            labels_message.markers.append(marker)
        markers_message.markers.extend([normal_marker, corner_marker])

        # Transient-local durability makes this single publication available to
        # sequencers and RViz instances that join after this constructor returns.
        self.targets_publisher.publish(targets_message)
        self.path_publisher.publish(path_message)
        self.types_publisher.publish(types_message)
        self.slope_flags_publisher.publish(slope_flags_message)
        self.labels_publisher.publish(labels_message)
        self.markers_publisher.publish(markers_message)
        # 使用 INFO 明确打印实际读取的路线文件。该节点只在启动时读取一次文件，
        # 因此这条日志也是确认 launch 场景参数是否真正生效的直接依据。
        self.get_logger().info(
            f"route generated once: route_file={selected_route_file} frame={parsed.frame_id} "
            f"targets={len(parsed.points)} corners="
            f"{sum(target_type == TargetType.CORNER for target_type in parsed.types)} "
            f"slopes={sum(parsed.is_slope)} "
            f"path_points={len(path_points)} spacing={spacing:.3f}m"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetPathGeneratorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError) as error:
        logger = node.get_logger() if node else rclpy.logging.get_logger("target_path_generator")
        logger.error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
