"""Launch the original Route3D server with the up-and-down RViz layout."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    tools_share = get_package_share_directory("route3d_bag_tools")
    route_share = get_package_share_directory("nav2_route3d")
    default_graph = "/home/wei/langyi/test/route_skeleton_ws/data/0825_1/route3d_graph.json"
    default_rviz = os.path.join(tools_share, "rviz", "up_and_down_route3d.rviz")

    graph_filepath = LaunchConfiguration("graph_filepath")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    route_visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(route_share, "launch", "route3d_visualization.launch.py")
        ),
        launch_arguments={
            "graph_filepath": graph_filepath,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "launch_route_server": "true",
            "use_rviz": "false",
            "publish_rejected_neighbor_markers": "false",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="up_and_down_route3d_rviz",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("graph_filepath", default_value=default_graph),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        route_visualization,
        rviz,
    ])
