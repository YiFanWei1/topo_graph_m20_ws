from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('route3d_topology_editor'))
    return LaunchDescription([
        DeclareLaunchArgument('graph_file'),
        DeclareLaunchArgument('frame_id', default_value='camera_init'),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(share / 'rviz' / 'topology_editor.rviz')),
        Node(
            package='rviz2',
            executable='rviz2',
            name='route3d_topology_editor_rviz',
            output='screen',
            arguments=[
                '-d', LaunchConfiguration('rviz_config'),
                '-f', LaunchConfiguration('frame_id'),
            ],
            parameters=[{
                'topology_editor.graph_file': LaunchConfiguration('graph_file'),
            }],
        ),
    ])
