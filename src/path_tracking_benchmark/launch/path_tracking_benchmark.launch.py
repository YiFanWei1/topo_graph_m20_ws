from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    planner_share = get_package_share_directory("efficient_3d_local_planner")
    adapter_share = get_package_share_directory("robot_control_adapter")
    config = os.path.join(planner_share, "config", "up_and_down.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("trajectory", default_value="straight"),
        DeclareLaunchArgument("run_label", default_value="baseline"),
        DeclareLaunchArgument("output_directory", default_value=""),
        DeclareLaunchArgument("max_tracking_error", default_value="0.30"),
        DeclareLaunchArgument("timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_smoothed"),
        DeclareLaunchArgument("enable_robot_adapter", default_value="false"),
        DeclareLaunchArgument("network_interface", default_value="eth0"),
        Node(
            package="efficient_3d_local_planner",
            executable="local_path_follower_node",
            name="local_path_follower",
            parameters=[config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            remappings=[("/cmd_vel_smoothed", LaunchConfiguration("cmd_vel_topic"))],
            output="screen",
        ),
        Node(
            package="path_tracking_benchmark",
            executable="fixed_path_benchmark_node",
            name="fixed_path_benchmark",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "trajectory": LaunchConfiguration("trajectory"),
                "run_label": LaunchConfiguration("run_label"),
                "output_directory": LaunchConfiguration("output_directory"),
                "max_tracking_error": LaunchConfiguration("max_tracking_error"),
                "timeout_sec": LaunchConfiguration("timeout_sec"),
                "input.command_topic": LaunchConfiguration("cmd_vel_topic"),
            }],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(adapter_share, "launch", "cmd_vel_adapter.launch.py")),
            launch_arguments={
                "network_interface": LaunchConfiguration("network_interface"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("enable_robot_adapter")),
        ),
    ])
