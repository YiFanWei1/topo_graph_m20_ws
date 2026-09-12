from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('route3d_vertex_initializer'),
        'config',
        'vertex_initializer.yaml',
    ])

    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='route3d_vertex_initializer',
            executable='vertex_initializer_node',
            name='route3d_vertex_initializer',
            output='screen',
            parameters=[params_file],
        ),
    ])
