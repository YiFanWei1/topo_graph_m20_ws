from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('route3d_odom_waypoint'))
    return LaunchDescription([
        DeclareLaunchArgument('pose_file'),
        DeclareLaunchArgument('topology_file'),
        DeclareLaunchArgument('frame_id', default_value='camera_init'),
        DeclareLaunchArgument('body_height', default_value='0.57'),
        DeclareLaunchArgument('show_vertex_labels', default_value='true'),
        DeclareLaunchArgument('show_edge_labels', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        Node(
            package='route3d_odom_waypoint',
            executable='visualize_file_result',
            name='route3d_odom_waypoint_visualizer',
            output='screen',
            parameters=[{
                'pose_file': LaunchConfiguration('pose_file'),
                'topology_file': LaunchConfiguration('topology_file'),
                'frame_id': LaunchConfiguration('frame_id'),
                'body_height': ParameterValue(
                    LaunchConfiguration('body_height'), value_type=float),
                'show_vertex_labels': ParameterValue(
                    LaunchConfiguration('show_vertex_labels'), value_type=bool),
                'show_edge_labels': ParameterValue(
                    LaunchConfiguration('show_edge_labels'), value_type=bool),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='route3d_odom_waypoint_rviz',
            arguments=[
                '-d', str(share / 'rviz' / 'odom_waypoint_result.rviz'),
                '-f', LaunchConfiguration('frame_id'),
            ],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
        ),
    ])
