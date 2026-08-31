from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "input_pose_topic", default_value="/route3d/synchronized_pose"),
        DeclareLaunchArgument("output_topo", default_value="topoSingle_live.json"),
        DeclareLaunchArgument("events_file", default_value="events.jsonl"),
        DeclareLaunchArgument("target_spacing", default_value="1.0"),
        DeclareLaunchArgument("body_height", default_value="0.40"),
        DeclareLaunchArgument("relocation_distance", default_value="0.50"),
        Node(
            package="route3d_online_skeleton",
            executable="online_route_skeleton_node",
            name="online_route_skeleton",
            output="screen",
            parameters=[{
                "input_pose_topic": LaunchConfiguration("input_pose_topic"),
                "output_topo": LaunchConfiguration("output_topo"),
                "events_file": LaunchConfiguration("events_file"),
                "target_spacing": LaunchConfiguration("target_spacing"),
                "body_height": LaunchConfiguration("body_height"),
                "relocation_distance": LaunchConfiguration("relocation_distance"),
            }],
        ),
    ])
