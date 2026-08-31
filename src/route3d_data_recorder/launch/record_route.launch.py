from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_directory", default_value="map_data"),
        DeclareLaunchArgument("odom_topic", default_value="/lio_odom"),
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
        DeclareLaunchArgument(
            "synchronized_pose_topic", default_value="/route3d/synchronized_pose"),
        DeclareLaunchArgument("sync_slop_s", default_value="0.05"),
        DeclareLaunchArgument("queue_size", default_value="100"),
        DeclareLaunchArgument("expected_odom_frame", default_value=""),
        DeclareLaunchArgument("expected_odom_child_frame", default_value=""),
        DeclareLaunchArgument("expected_cloud_frame", default_value=""),
        Node(
            package="route3d_data_recorder",
            executable="route3d_data_recorder_node",
            name="route3d_data_recorder",
            output="screen",
            parameters=[{
                "output_directory": LaunchConfiguration("output_directory"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "synchronized_pose_topic": LaunchConfiguration("synchronized_pose_topic"),
                "sync_slop_s": LaunchConfiguration("sync_slop_s"),
                "queue_size": LaunchConfiguration("queue_size"),
                "expected_odom_frame": LaunchConfiguration("expected_odom_frame"),
                "expected_odom_child_frame": LaunchConfiguration("expected_odom_child_frame"),
                "expected_cloud_frame": LaunchConfiguration("expected_cloud_frame"),
            }],
        ),
    ])
