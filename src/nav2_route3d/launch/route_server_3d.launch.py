from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    graph_filepath = LaunchConfiguration("graph_filepath")
    use_sim_time = LaunchConfiguration("use_sim_time")
    path_density = LaunchConfiguration("path_density")
    visualization_enabled = LaunchConfiguration("visualization_enabled")
    publish_graph_markers = LaunchConfiguration("publish_graph_markers")
    publish_route_markers = LaunchConfiguration("publish_route_markers")
    publish_route_event_markers = LaunchConfiguration("publish_route_event_markers")
    publish_rejected_neighbor_markers = LaunchConfiguration("publish_rejected_neighbor_markers")

    default_params = PathJoinSubstitution(
        [FindPackageShare("nav2_route3d"), "config", "route_server_3d.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("graph_filepath", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("path_density", default_value="0.5"),
            DeclareLaunchArgument("visualization_enabled", default_value="false"),
            DeclareLaunchArgument("publish_graph_markers", default_value="true"),
            DeclareLaunchArgument("publish_route_markers", default_value="true"),
            DeclareLaunchArgument("publish_route_event_markers", default_value="true"),
            DeclareLaunchArgument("publish_rejected_neighbor_markers", default_value="false"),
            LifecycleNode(
                package="nav2_route3d",
                executable="route_server_3d",
                name="route_server",
                namespace="",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "graph_filepath": graph_filepath,
                        "use_sim_time": use_sim_time,
                        "path_density": path_density,
                "path_density": path_density,
                        "visualization_enabled": visualization_enabled,
                        "publish_graph_markers": publish_graph_markers,
                        "publish_route_markers": publish_route_markers,
                        "publish_route_event_markers": publish_route_event_markers,
                        "publish_rejected_neighbor_markers": publish_rejected_neighbor_markers,
                    },
                ],
            ),
        ]
    )
