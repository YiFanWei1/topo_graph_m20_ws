"""Start the exclusive M20 manual velocity path and basic_server bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    enable_motion = LaunchConfiguration("enable_motion")
    start_bridge = LaunchConfiguration("start_bridge")
    control_config = LaunchConfiguration("control_config")
    bridge_config = LaunchConfiguration("bridge_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="Arm manual motion and start M20 normal/Stand/RL preparation.",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start basic_server bridge; set false only for offline topic tests.",
        ),
        DeclareLaunchArgument(
            "control_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("m20_velocity_control"),
                "config",
                "manual_velocity_control.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("m20_velocity_control"),
                "config",
                "m20_manual_bridge.yaml",
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("basic_server_bridge"),
                "launch",
                "m20_control.launch.py",
            ])),
            launch_arguments={
                "config_file": bridge_config,
                "start_prepare": enable_motion,
            }.items(),
            condition=IfCondition(start_bridge),
        ),
        Node(
            package="m20_velocity_control",
            executable="m20_velocity_control_node",
            name="m20_velocity_control",
            output="screen",
            parameters=[
                control_config,
                {"enable_motion": enable_motion},
            ],
        ),
    ])
