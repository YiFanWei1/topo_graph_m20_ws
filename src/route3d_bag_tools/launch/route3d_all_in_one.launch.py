"""Generate typed skeleton targets, then start planner, Route3D, and RViz."""

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
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    planner_share = get_package_share_directory("efficient_3d_local_planner")
    tools_share = get_package_share_directory("route3d_bag_tools")

    graph_filepath = LaunchConfiguration("graph_filepath")
    target_file = LaunchConfiguration("target_file")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    body_height = LaunchConfiguration("body_height")
    normal_arrival_tolerance = LaunchConfiguration("normal_arrival_tolerance")
    corner_arrival_tolerance = LaunchConfiguration("corner_arrival_tolerance")
    goal_arrival_tolerance = LaunchConfiguration("goal_arrival_tolerance")
    finish_distance = LaunchConfiguration("finish_distance")
    target_spacing = LaunchConfiguration("target_spacing")
    corner_yaw_xy_radius = LaunchConfiguration("corner_yaw_xy_radius")
    corner_yaw_angle_deg = LaunchConfiguration("corner_yaw_angle_deg")
    corner_yaw_min_duration = LaunchConfiguration("corner_yaw_min_duration")
    corner_min_separation = LaunchConfiguration("corner_min_separation")
    corner_max_z_range = LaunchConfiguration("corner_max_z_range")
    launch_rviz = LaunchConfiguration("launch_rviz")

    planner_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_share, "launch", "up_and_down_demo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "rviz": "false",
            "target_file": target_file,
            "cmd_vel_topic": cmd_vel_topic,
            "body_height": body_height,
            "normal_arrival_tolerance": normal_arrival_tolerance,
            "corner_arrival_tolerance": corner_arrival_tolerance,
            "goal_arrival_tolerance": goal_arrival_tolerance,
            "finish_distance": finish_distance,
        }.items(),
    )

    route3d_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                tools_share,
                "launch",
                "route3d_up_and_down_visualization.launch.py",
            )
        ),
        launch_arguments={
            "graph_filepath": graph_filepath,
            "use_sim_time": "true",
            "launch_rviz": launch_rviz,
        }.items(),
    )

    corner_target_generator = ExecuteProcess(
        cmd=[
            "ros2", "run", "route3d_bag_tools", "route_corner_target_generator",
            "--graph", graph_filepath,
            "--output", target_file,
            "--body-height", body_height,
            "--target-spacing", target_spacing,
            "--corner-yaw-xy-radius", corner_yaw_xy_radius,
            "--corner-yaw-angle-deg", corner_yaw_angle_deg,
            "--corner-yaw-min-duration", corner_yaw_min_duration,
            "--corner-min-separation", corner_min_separation,
            "--corner-max-z-range", corner_max_z_range,
        ],
        output="screen",
    )

    def start_runtime_after_generation(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg="ERROR: corner target generation failed; runtime was not started."),
                EmitEvent(event=Shutdown(reason="corner target generation failed")),
            ]
        return [
            LogInfo(msg="Corner targets generated; starting planner and visualization."),
            planner_demo,
            route3d_visualization,
        ]

    start_after_generator = RegisterEventHandler(
        OnProcessExit(
            target_action=corner_target_generator,
            on_exit=start_runtime_after_generation,
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "graph_filepath",
            default_value=(
                "/home/wei/langyi/test/route_skeleton_ws/data/full_nav_replay_with_plan/"
                "route3d_graph.json"
            ),
            description="offline Route3D graph generated from the bag",
        ),
        DeclareLaunchArgument(
            "target_file",
            default_value=(
                "/home/wei/langyi/test/route_skeleton_ws/data/full_nav_replay_with_plan/"
                "route_targets_with_corners.txt"
            ),
            description="generated typed target file used by the planner",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel_smoothed",
            description="local follower velocity output topic",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="launch the combined Route3D and planner RViz view"),
        DeclareLaunchArgument("body_height", default_value="0.57"),
        DeclareLaunchArgument(
            "target_spacing", default_value="1.0",
            description="ordinary skeleton target spacing in metres"),
        DeclareLaunchArgument(
            "corner_yaw_xy_radius", default_value="0.50",
            description="maximum XY radius of one remote-control rotation event"),
        DeclareLaunchArgument(
            "corner_yaw_angle_deg", default_value="90.0",
            description="yaw sweep must be strictly greater than this angle"),
        DeclareLaunchArgument(
            "corner_yaw_min_duration", default_value="0.50",
            description="minimum duration of a remote-control rotation event"),
        DeclareLaunchArgument(
            "corner_min_separation", default_value="1.0",
            description="minimum route distance between detected corner targets"),
        DeclareLaunchArgument(
            "corner_max_z_range", default_value="0.10",
            description="maximum Z range allowed during one rotation event"),
        DeclareLaunchArgument(
            "normal_arrival_tolerance", default_value="2.80",
            description="pass-through tolerance for ordinary yellow targets"),
        DeclareLaunchArgument(
            "corner_arrival_tolerance", default_value="0.20",
            description="strict tolerance for red corner targets"),
        DeclareLaunchArgument(
            "goal_arrival_tolerance", default_value="0.15",
            description="strict tolerance for the user-selected final target"),
        DeclareLaunchArgument(
            "finish_distance", default_value="0.10",
            description="local follower stop threshold; must not exceed final tolerance"),
        start_after_generator,
        corner_target_generator,
    ])
