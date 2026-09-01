"""Run the existing local path follower against one fixed PCD-validated circle."""

from datetime import datetime
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _setup(context):
    package_share = Path(get_package_share_directory("circle_path_benchmark"))
    config = Path(LaunchConfiguration("config").perform(context)).expanduser().resolve()
    path_file = Path(LaunchConfiguration("path_file").perform(context)).expanduser().resolve()
    pcd_file = Path(LaunchConfiguration("pcd_file").perform(context)).expanduser().resolve()
    output_root = Path(
        LaunchConfiguration("output_root").perform(context)).expanduser().resolve()
    session_name = LaunchConfiguration("session_name").perform(context).strip()
    if not session_name:
        session_name = datetime.now().strftime("circle_%Y%m%d_%H%M%S")
    if Path(session_name).name != session_name:
        raise RuntimeError("session_name must be a single directory name")

    for required in (config, path_file, pcd_file):
        if not required.is_file():
            raise RuntimeError(f"required benchmark file does not exist: {required}")

    session = output_root / session_name
    if session.exists():
        raise RuntimeError(f"benchmark output already exists: {session}")
    session.mkdir(parents=True)

    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic").perform(context)
    path_topic = "/local_planner/local_path"
    enable_motion = _as_bool(LaunchConfiguration("enable_motion").perform(context))
    center_from_odometry = _as_bool(
        LaunchConfiguration("center_from_initial_odometry").perform(context))
    record_bag = _as_bool(LaunchConfiguration("record_bag").perform(context))
    show_rviz = _as_bool(LaunchConfiguration("rviz").perform(context))

    actions = [
        LogInfo(msg=f"Circle benchmark output: {session}"),
        LogInfo(msg=(
            "Motion path ARMED after start-pose check" if enable_motion else
            "Motion is DISARMED; reference visualization and recording only")),
        Node(
            package="circle_path_benchmark",
            executable="circle_path_benchmark_node",
            name="circle_path_benchmark",
            parameters=[str(config), {
                "benchmark.path_file": str(path_file),
                "benchmark.pcd_file": str(pcd_file),
                "benchmark.output_directory": str(session),
                "benchmark.enable_motion": enable_motion,
                "benchmark.center_from_initial_odometry": center_from_odometry,
                "topics.odometry": odom_topic,
                "topics.cmd_vel": cmd_vel_topic,
                "topics.control_path": path_topic,
            }],
            output="screen",
        ),
        Node(
            package="efficient_3d_local_planner",
            executable="local_path_follower_node",
            name="local_path_follower",
            parameters=[str(config)],
            remappings=[
                ("/local_planner/local_path", path_topic),
                ("/lio_odom_hf", odom_topic),
                ("/cmd_vel_smoothed", cmd_vel_topic),
            ],
            output="screen",
        ),
    ]
    if record_bag:
        actions.append(ExecuteProcess(
            cmd=[
                "ros2", "bag", "record", "-o", str(session / "bag"), "--topics",
                odom_topic, cmd_vel_topic, path_topic,
                "/circle_benchmark/reference_path",
                "/circle_benchmark/actual_path",
                "/circle_benchmark/diagnostics",
                "/local_planner/controller_diagnostics", "/tf", "/tf_static",
            ],
            output="screen",
        ))
    if show_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(package_share / "rviz" / "circle_tracking_benchmark.rviz")],
            output="screen",
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("circle_path_benchmark")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=os.path.join(
                share, "config", "circle_tracking_benchmark.yaml")),
        DeclareLaunchArgument("path_file", description="Offline generated circle YAML"),
        DeclareLaunchArgument("pcd_file", description="PCD used to validate and visualize the circle"),
        DeclareLaunchArgument("output_root", default_value="/tmp/circle_path_benchmark_runs"),
        DeclareLaunchArgument("session_name", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value="/lio_odom_hf"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_smoothed"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("center_from_initial_odometry", default_value="true"),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
