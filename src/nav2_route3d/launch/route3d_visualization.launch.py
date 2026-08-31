from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import EqualsSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    graph_filepath = LaunchConfiguration("graph_filepath")
    use_sim_time = LaunchConfiguration("use_sim_time")
    path_density = LaunchConfiguration("path_density")
    autostart = LaunchConfiguration("autostart")
    launch_route_server = LaunchConfiguration("launch_route_server")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    publish_rejected_neighbor_markers = LaunchConfiguration("publish_rejected_neighbor_markers")

    default_params = PathJoinSubstitution(
        [FindPackageShare("nav2_route3d"), "config", "route_server_3d.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("nav2_route3d"), "rviz", "route3d_validation.rviz"]
    )

    route_server = LifecycleNode(
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
                "visualization_enabled": True,
                "publish_graph_markers": True,
                "publish_route_markers": True,
                "publish_route_event_markers": True,
                "publish_rejected_neighbor_markers": publish_rejected_neighbor_markers,
            },
        ],
    )

    configure_route_server = EmitEvent(
        condition=IfCondition(autostart),
        event=ChangeState(
            lifecycle_node_matcher=matches_action(route_server),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
    )

    activate_route_server = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=route_server,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(route_server),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2_route3d",
        arguments=["-d", rviz_config],
        output="screen",
    )

    route_server_group = GroupAction(
        condition=IfCondition(
            EqualsSubstitution(launch_route_server, "true")
        ),
        actions=[
            route_server,
            configure_route_server,
            activate_route_server,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("graph_filepath", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("path_density", default_value="0.5"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "launch_route_server",
                default_value="true",
                description=(
                    "If true, start a standalone route_server (debug). "
                    "Set false when route_server is already started by Nav2 bringup."
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("publish_rejected_neighbor_markers", default_value="false"),
            route_server_group,
            rviz,
        ]
    )
