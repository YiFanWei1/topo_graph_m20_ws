"""TTY-operated recorder for ordered, ground-referenced 3-D targets."""

import math
import select
import sys
import termios
import tty

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from recorded_waypoint_route.route_utils import (
    atomic_write_target_file,
    distance_3d,
    TargetType,
)
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


def default_target_file() -> str:
    return get_package_share_directory("recorded_waypoint_route") + "/config/target.txt"


class TargetRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("target_recorder")
        self.odom_topic = str(self.declare_parameter(
            "input.odometry_topic", "/lio_odom_hf").value)
        self.planning_frame = str(self.declare_parameter(
            "frames.planning", "camera_init").value)
        self.body_height = float(self.declare_parameter(
            "recording.body_height", 0.57).value)
        self.minimum_spacing = float(self.declare_parameter(
            "recording.minimum_target_spacing", 0.05).value)
        self.output_file = str(self.declare_parameter(
            "output_file", default_target_file()).value)
        if not self.planning_frame:
            raise ValueError("frames.planning must not be empty")
        if not math.isfinite(self.body_height) or self.body_height < 0.0:
            raise ValueError("recording.body_height must be finite and non-negative")
        if not math.isfinite(self.minimum_spacing) or self.minimum_spacing < 0.0:
            raise ValueError("recording.minimum_target_spacing must be finite and non-negative")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_odom = None
        self.targets = []
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos_profile_sensor_data)

        # A recording session always creates a new route, as requested. The write is
        # atomic so an interrupted process never leaves a partially formatted file.
        self._save()
        self.get_logger().info(
            f"new target recording: odom={self.odom_topic} frame={self.planning_frame} "
            f"body_height={self.body_height:.3f}m output={self.output_file}"
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def _body_point_in_planning_frame(self):
        if self.latest_odom is None:
            return None
        source_frame = self.latest_odom.header.frame_id or self.planning_frame
        source = PoseStamped()
        source.header = self.latest_odom.header
        source.header.frame_id = source_frame
        source.pose = self.latest_odom.pose.pose
        if source_frame != self.planning_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.planning_frame, source_frame, Time())
                source = do_transform_pose_stamped(source, transform)
            except TransformException as error:
                self.get_logger().warning(
                    f"cannot transform odometry {source_frame} -> {self.planning_frame}: {error}")
                return None
        position = source.pose.position
        point = (float(position.x), float(position.y), float(position.z))
        return point if all(math.isfinite(value) for value in point) else None

    def _save(self) -> None:
        atomic_write_target_file(
            self.output_file,
            self.planning_frame,
            self.targets,
            [TargetType.CORNER] * len(self.targets),
        )

    def record_current(self) -> None:
        body_point = self._body_point_in_planning_frame()
        if body_point is None:
            self.get_logger().warning("no valid odometry is available")
            return
        ground_point = (body_point[0], body_point[1], body_point[2] - self.body_height)
        if self.targets and distance_3d(self.targets[-1], ground_point) < self.minimum_spacing:
            self.get_logger().warning(
                f"target ignored: only {distance_3d(self.targets[-1], ground_point):.3f}m "
                "from the previous target")
            return
        self.targets.append(ground_point)
        self._save()
        self.get_logger().info(
            f"recorded target {len(self.targets)}: ground=[{ground_point[0]:.3f} "
            f"{ground_point[1]:.3f} {ground_point[2]:.3f}] type=CORNER"
        )

    def undo(self) -> None:
        if not self.targets:
            self.get_logger().warning("there is no target to undo")
            return
        removed = self.targets.pop()
        self._save()
        self.get_logger().info(
            f"removed target: [{removed[0]:.3f} {removed[1]:.3f} {removed[2]:.3f}]")

    def list_targets(self) -> None:
        if not self.targets:
            print("No targets recorded")
            return
        print("\n".join(
            f"{index}: [{point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}]"
            for index, point in enumerate(self.targets, 1)
        ))

    def run(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard recording requires a TTY")
        print("Space/Enter/A: add, U: undo, L: list, Q: save and quit")
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
                if not ready:
                    continue
                key = sys.stdin.read(1).lower()
                if key in ("\n", "\r", " ", "a"):
                    self.record_current()
                elif key == "u":
                    self.undo()
                elif key == "l":
                    self.list_targets()
                elif key in ("q", "\x03"):
                    self._save()
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetRecorderNode()
        node.run()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        logger = node.get_logger() if node else rclpy.logging.get_logger("target_recorder")
        logger.error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
