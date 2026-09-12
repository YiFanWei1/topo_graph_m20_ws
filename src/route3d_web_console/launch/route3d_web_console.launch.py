from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("route3d_web_console")
    initializer_share = FindPackageShare("route3d_vertex_initializer")
    # Keep the web console in the same ROS domain as the operator shell/system services.
    # Do not force a private RMW/CycloneDDS profile here; otherwise a radar driver started
    # by systemd can be invisible to the web node while a web-started driver is invisible
    # to an ordinary terminal.
    common_env = {
        "ROS_DOMAIN_ID": LaunchConfiguration("ros_domain_id"),
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([share, "config", "web_console.yaml"])),
        DeclareLaunchArgument(
            "catalog_seed_file",
            default_value=PathJoinSubstitution([share, "config", "map_catalog.json"])),
        DeclareLaunchArgument(
            "vertex_initializer_params",
            default_value=PathJoinSubstitution([
                initializer_share, "config", "vertex_initializer.yaml"])),
        DeclareLaunchArgument("launch_vertex_initializer", default_value="true"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0")),
        Node(
            package="route3d_vertex_initializer",
            executable="vertex_initializer_node",
            name="route3d_vertex_initializer",
            output="screen",
            condition=IfCondition(LaunchConfiguration("launch_vertex_initializer")),
            additional_env=common_env,
            parameters=[LaunchConfiguration("vertex_initializer_params")],
        ),
        Node(
            package="route3d_web_console",
            executable="route3d_web_console_node",
            name="route3d_web_console",
            output="screen",
            additional_env=common_env,
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
