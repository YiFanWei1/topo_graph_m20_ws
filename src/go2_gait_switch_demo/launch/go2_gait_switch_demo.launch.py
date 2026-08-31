from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("go2_gait_switch_demo"),
        "config",
        "go2_gait_switch_demo.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Go2 gait-switch demo parameter file",
        ),
        DeclareLaunchArgument(
            "network_interface",
            default_value="eth0",
            description="Network interface connected to the Go2",
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            choices=["true", "false"],
            description="Explicit safety interlock; true permits real robot motion",
        ),
        Node(
            package="go2_gait_switch_demo",
            executable="go2_gait_switch_demo_node",
            name="go2_gait_switch_demo",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "network_interface": LaunchConfiguration("network_interface"),
                    "enable_motion": ParameterValue(
                        LaunchConfiguration("enable_motion"), value_type=bool),
                },
            ],
        ),
    ])
