from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory('route3d_loop_patrol'))
        / 'config'
        / 'loop_patrol.yaml')
    config_file = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Loop patrol ROS parameter YAML file',
        ),
        Node(
            package='route3d_loop_patrol',
            executable='loop_patrol_node',
            name='route3d_loop_patrol',
            output='screen',
            parameters=[config_file],
        ),
    ])
