from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("route3d_web_console")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([share, "config", "web_console.yaml"])),
        DeclareLaunchArgument(
            "catalog_seed_file",
            default_value=PathJoinSubstitution([share, "config", "map_catalog.json"])),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("ros_domain_id", default_value="42"),
        DeclareLaunchArgument("rmw_implementation", default_value="rmw_cyclonedds_cpp"),
        DeclareLaunchArgument(
            "cyclonedds_uri",
            default_value=["file://", PathJoinSubstitution([
                share, "config", "cyclonedds_web.xml"]) ]),
        DeclareLaunchArgument(
            "cyclonedds_config_file",
            default_value=PathJoinSubstitution([share, "config", "cyclonedds_web.xml"])),
        Node(
            package="route3d_web_console",
            executable="route3d_web_console_node",
            name="route3d_web_console",
            output="screen",
            additional_env={
                "ROS_DOMAIN_ID": LaunchConfiguration("ros_domain_id"),
                "RMW_IMPLEMENTATION": LaunchConfiguration("rmw_implementation"),
                "CYCLONEDDS_URI": LaunchConfiguration("cyclonedds_uri"),
                "CYCLONEDDS_CONFIG_FILE": LaunchConfiguration("cyclonedds_config_file"),
            },
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "catalog_seed_file": LaunchConfiguration("catalog_seed_file"),
                    "server.port": LaunchConfiguration("port"),
                    "server.static_root": PathJoinSubstitution([share, "web"]),
                    "process.ros_domain_id": ParameterValue(
                        LaunchConfiguration("ros_domain_id"), value_type=int),
                },
            ],
        ),
    ])
