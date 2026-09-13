"""Start the M20 basic_server bridge and optional automatic control preparation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    start_prepare = LaunchConfiguration("start_prepare")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("basic_server_bridge"),
                "config",
                "basic_server_bridge.yaml",
            ]),
            description="Path to the basic_server_bridge YAML configuration file.",
        ),
        DeclareLaunchArgument(
            "start_prepare",
            default_value="true",
            description="Whether to start m20_ready_cmd_vel for automatic control preparation.",
        ),
        Node(
            package="basic_server_bridge",
            executable="basic_server_bridge_node",
            name="basic_server_bridge",
            output="screen",
            arguments=[config_file],
        ),
        Node(
            package="basic_server_bridge",
            executable="m20_ready_cmd_vel",
            name="m20_ready_cmd_vel",
            output="screen",
            condition=IfCondition(start_prepare),
        ),
    ])
