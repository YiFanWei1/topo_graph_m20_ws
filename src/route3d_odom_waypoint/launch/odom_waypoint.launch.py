from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory('route3d_odom_waypoint'))
        / 'config'
        / 'odom_waypoint.yaml')
    default_rviz_config = str(
        Path(get_package_share_directory('route3d_odom_waypoint'))
        / 'rviz'
        / 'odom_waypoint_result.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('output_file', default_value='topoGraph_odom.json'),
        DeclareLaunchArgument('frame_id', default_value='camera_init'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        Node(
            package='route3d_odom_waypoint',
            executable='odom_waypoint_node',
            name='route3d_odom_waypoint',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'output_file': LaunchConfiguration('output_file')},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='route3d_odom_waypoint_live_rviz',
            arguments=[
                '-d', LaunchConfiguration('rviz_config'),
                '-f', LaunchConfiguration('frame_id'),
            ],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
            output='screen',
        ),
    ])
