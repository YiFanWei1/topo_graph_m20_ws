"""Annotate one topology JSON and visualize per-vertex slope attributes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("route_slope_annotator")
    default_root = "/home/wei/github_code/topo_graph_ws_/data/79"
    float_parameter = lambda name: ParameterValue(
        LaunchConfiguration(name), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_file", default_value=os.path.join(
                default_root, "topoSingle_data.json")),
        DeclareLaunchArgument(
            "output_file", default_value=os.path.join(
                default_root, "topoSingle_data_slope_preview.json")),
        DeclareLaunchArgument("frame_id", default_value="camera_init"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("fit_radius", default_value="2.0"),
        DeclareLaunchArgument("grade_threshold", default_value="0.12"),
        DeclareLaunchArgument("minimum_core_length", default_value="2.0"),
        DeclareLaunchArgument("minimum_height_change", default_value="0.20"),
        DeclareLaunchArgument("maximum_core_gap", default_value="1.0"),
        DeclareLaunchArgument("buffer_distance", default_value="3.0"),
        Node(
            package="route_slope_annotator",
            executable="visualize_route_slopes",
            name="route_slope_visualizer",
            output="screen",
            parameters=[{
                "input_file": LaunchConfiguration("input_file"),
                "output_file": LaunchConfiguration("output_file"),
                "frame_id": LaunchConfiguration("frame_id"),
                "fit_radius": float_parameter("fit_radius"),
                "grade_threshold": float_parameter("grade_threshold"),
                "minimum_core_length": float_parameter("minimum_core_length"),
                "minimum_height_change": float_parameter(
                    "minimum_height_change"),
                "maximum_core_gap": float_parameter("maximum_core_gap"),
                "buffer_distance": float_parameter("buffer_distance"),
            }]),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d", os.path.join(share, "rviz", "slope_preview.rviz"),
                "-f", LaunchConfiguration("frame_id")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen"),
    ])
