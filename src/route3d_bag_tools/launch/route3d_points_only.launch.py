"""Visualize the offline Route3D graph and typed route targets without a bag or planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tools_share = get_package_share_directory("route3d_bag_tools")
    route_share = get_package_share_directory("nav2_route3d")
    data_root = "/home/wei/langyi/test/route_skeleton_ws/data/regu"

    graph_filepath = LaunchConfiguration("graph_filepath")
    target_file = LaunchConfiguration("target_file")
    body_height = ParameterValue(LaunchConfiguration("body_height"), value_type=float)
    route_spacing = ParameterValue(LaunchConfiguration("route_spacing"), value_type=float)
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    route_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(route_share, "launch", "route3d_visualization.launch.py")
        ),
        launch_arguments={
            "graph_filepath": graph_filepath,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "launch_route_server": "true",
            "use_rviz": "false",
            "publish_rejected_neighbor_markers": "false",
        }.items(),
    )

    target_generator = Node(
        package="recorded_waypoint_route",
        executable="target_path_generator_node",
        name="typed_skeleton_target_visualizer",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_file": target_file,
            "route.body_height": body_height,
            "route.spacing": route_spacing,
        }],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="route3d_points_only_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "graph_filepath",
            default_value=os.path.join(data_root, "route3d_graph.json")),
        DeclareLaunchArgument(
            "target_file",
            default_value=os.path.join(data_root, "route_targets_with_corners.txt")),
        DeclareLaunchArgument("body_height", default_value="0.40"),
        DeclareLaunchArgument("route_spacing", default_value="0.10"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Standalone visualization uses wall time and does not need a bag"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(tools_share, "rviz", "route3d_points_only.rviz")),
        LogInfo(msg=[
            "Point legend: blue=Route3D nodes, gold=shortcut anchors, "
            "yellow=NORMAL, red=CORNER; target_file=", target_file,
        ]),
        route_visualization,
        target_generator,
        rviz,
    ])
