"""Run the complete offline target, topoSingle conversion, and RViz flow."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    workspace = "/home/wei/github_code/topo_graph_ws"
    data_root = os.path.join(workspace, "data", "regu")
    tools_share = get_package_share_directory("route3d_bag_tools")
    route_share = get_package_share_directory("nav2_route3d")

    graph = LaunchConfiguration("graph_filepath")
    targets = LaunchConfiguration("target_file")
    topo = LaunchConfiguration("topo_output")
    reduced_graph = LaunchConfiguration("reduced_graph_output")
    body_height = LaunchConfiguration("body_height")
    target_spacing = LaunchConfiguration("target_spacing")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    generate_targets = ExecuteProcess(
        cmd=[
            "ros2", "run", "route3d_bag_tools", "route_corner_target_generator",
            "--graph", graph,
            "--output", targets,
            "--output-topo", topo,
            "--body-height", body_height,
            "--target-spacing", target_spacing,
        ],
        output="screen",
    )

    convert_graph = ExecuteProcess(
        cmd=[
            "ros2", "run", "topo_graph_tools", "nav2_to_topo_single",
            "--graph", graph,
            "--targets", targets,
            "--output-nav2", reduced_graph,
            "--body-height", body_height,
        ],
        output="screen",
    )

    route_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(route_share, "launch", "route3d_visualization.launch.py")
        ),
        launch_arguments={
            "graph_filepath": reduced_graph,
            "use_sim_time": "false",
            "autostart": "true",
            "launch_route_server": "true",
            "use_rviz": "false",
            "publish_rejected_neighbor_markers": "false",
        }.items(),
    )

    target_visualization = Node(
        package="recorded_waypoint_route",
        executable="target_path_generator_node",
        name="selected_target_visualizer",
        parameters=[{
            "target_file": targets,
            "route.body_height": ParameterValue(body_height, value_type=float),
            "route.spacing": 0.10,
        }],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="topo_conversion_rviz",
        arguments=["-d", rviz_config],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    def start_visualization(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg="ERROR: topoSingle conversion failed; visualization was not started."),
                EmitEvent(event=Shutdown(reason="topoSingle conversion failed")),
            ]
        return [
            LogInfo(msg=["topoSingle JSON ready: ", topo]),
            route_visualization,
            target_visualization,
            rviz,
        ]

    after_conversion = RegisterEventHandler(
        OnProcessExit(target_action=convert_graph, on_exit=start_visualization)
    )

    def start_conversion(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg="ERROR: target generation failed; conversion was not started."),
                EmitEvent(event=Shutdown(reason="target generation failed")),
            ]
        return [convert_graph]

    after_target_generation = RegisterEventHandler(
        OnProcessExit(target_action=generate_targets, on_exit=start_conversion)
    )

    return LaunchDescription([
        DeclareLaunchArgument("graph_filepath", default_value=os.path.join(data_root, "route3d_graph.json")),
        DeclareLaunchArgument("target_file", default_value=os.path.join(data_root, "route_targets_with_corners.txt")),
        DeclareLaunchArgument("topo_output", default_value=os.path.join(data_root, "topoSingle_data.json")),
        DeclareLaunchArgument("reduced_graph_output", default_value=os.path.join(data_root, "selected_route3d_graph.json")),
        DeclareLaunchArgument("body_height", default_value="0.40"),
        DeclareLaunchArgument("target_spacing", default_value="1.0"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(tools_share, "rviz", "route3d_points_only.rviz")),
        after_target_generation,
        after_conversion,
        generate_targets,
    ])
