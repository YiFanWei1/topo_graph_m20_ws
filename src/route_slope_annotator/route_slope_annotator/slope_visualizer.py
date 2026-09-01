"""Publish slope-annotated topology points as RViz markers."""

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from route_slope_annotator.slope_analysis import SlopeConfig, annotate_document


COLORS = {
    "normal": ColorRGBA(r=0.15, g=0.85, b=0.25, a=1.0),
    "slope": ColorRGBA(r=1.0, g=0.20, b=0.05, a=1.0),
}


class SlopeVisualizer(Node):
    def __init__(self):
        super().__init__("route_slope_visualizer")
        self.declare_parameter("input_file", "")
        self.declare_parameter("output_file", "")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("fit_radius", 2.0)
        self.declare_parameter("grade_threshold", 0.12)
        self.declare_parameter("minimum_core_length", 2.0)
        self.declare_parameter("minimum_height_change", 0.20)
        self.declare_parameter("maximum_core_gap", 1.0)
        self.declare_parameter("buffer_distance", 2.0)
        self.declare_parameter("point_z_offset", 0.08)
        self.declare_parameter("show_vertex_ids", True)

        input_file = str(self.get_parameter("input_file").value)
        output_file = str(self.get_parameter("output_file").value)
        if not input_file:
            raise ValueError("input_file is required")
        input_path = Path(input_file).expanduser().resolve()
        with input_path.open("r", encoding="utf-8") as stream:
            source = json.load(stream)
        config = SlopeConfig(
            fit_radius=float(self.get_parameter("fit_radius").value),
            grade_threshold=float(self.get_parameter("grade_threshold").value),
            minimum_core_length=float(
                self.get_parameter("minimum_core_length").value),
            minimum_height_change=float(
                self.get_parameter("minimum_height_change").value),
            maximum_core_gap=float(self.get_parameter("maximum_core_gap").value),
            buffer_distance=float(self.get_parameter("buffer_distance").value))
        self.document = annotate_document(source, config)
        self.buffer_distance = config.buffer_distance
        if output_file:
            output_path = Path(output_file).expanduser().resolve()
            if output_path == input_path:
                raise ValueError("output_file must not overwrite input_file")
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with output_path.open("w", encoding="utf-8") as stream:
                json.dump(self.document, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
            self.get_logger().info(f"Wrote annotated route: {output_path}")

        configured_frame = str(self.get_parameter("frame_id").value)
        self.frame_id = configured_frame or str(
            self.document.get("frame_id", "camera_init"))
        self.z_offset = float(self.get_parameter("point_z_offset").value)
        self.show_ids = bool(self.get_parameter("show_vertex_ids").value)
        if not math.isfinite(self.z_offset):
            raise ValueError("point_z_offset must be finite")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, "/route_slope_annotation/markers", qos)
        self.timer = self.create_timer(0.25, self.publish_once)

        annotation = self.document["slopeAnnotation"]
        self.get_logger().info(
            "Slope preview ready: frame=%s counts=%s" %
            (self.frame_id, annotation["counts"]))
        for segment in annotation["segments"]:
            self.get_logger().info(
                "slope core vertices=%d-%d direction=%s grade=%.3f "
                "length=%.2fm dz=%.2fm" %
                (segment["startVertex"], segment["endVertex"],
                 segment["direction"], segment["meanGrade"],
                 segment["length"], segment["heightChange"]))

    def marker(self, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def point(position, z_offset):
        return Point(x=float(position[0]), y=float(position[1]),
                     z=float(position[2]) + z_offset)

    def publish_once(self):
        markers = MarkerArray()
        delete = self.marker("route_slope", 0, Marker.POINTS)
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        groups = {}
        for marker_id, terrain_type in enumerate(COLORS, start=1):
            marker = self.marker("terrain_points", marker_id, Marker.SPHERE_LIST)
            diameter = 0.14 if terrain_type == "normal" else 0.24
            marker.scale.x = marker.scale.y = marker.scale.z = diameter
            marker.color = COLORS[terrain_type]
            groups[terrain_type] = marker

        line = self.marker("route_profile", 0, Marker.LINE_STRIP)
        line.scale.x = 0.055
        vertices = self.document["vertices"]
        for vertex_id in range(1, len(vertices) + 1):
            vertex = vertices[str(vertex_id)]
            terrain_type = "slope" if vertex["meta"]["isSlope"] else "normal"
            display_point = self.point(vertex["pos"], self.z_offset)
            groups[terrain_type].points.append(display_point)
            line.points.append(display_point)
            line.colors.append(COLORS[terrain_type])
            if self.show_ids:
                label = self.marker("vertex_ids", vertex_id, Marker.TEXT_VIEW_FACING)
                label.pose.position = self.point(vertex["pos"], self.z_offset + 0.20)
                label.scale.z = 0.13
                label.color = COLORS[terrain_type]
                label.text = str(vertex_id)
                markers.markers.append(label)
        markers.markers.insert(1, line)
        markers.markers[2:2] = list(groups.values())

        first = vertices["1"]["pos"]
        labels = [
            ("normal", "GREEN: normal point"),
            ("slope", "RED: slope point (core +/-%.1fm)" %
             self.buffer_distance),
        ]
        for index, (terrain_type, text) in enumerate(labels):
            legend = self.marker("legend", index, Marker.TEXT_VIEW_FACING)
            legend.pose.position = Point(
                x=float(first[0]), y=float(first[1]) + 0.45 * index,
                z=float(first[2]) + self.z_offset + 1.0)
            legend.scale.z = 0.24
            legend.color = COLORS[terrain_type]
            legend.text = text
            markers.markers.append(legend)

        self.publisher.publish(markers)
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SlopeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
