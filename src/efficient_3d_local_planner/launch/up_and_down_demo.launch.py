"""Launch the dynamic 3-D local planner for the up_and_down bag."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("efficient_3d_local_planner")
    default_config = os.path.join(share, "config", "up_and_down.yaml")
    # 产品运行直接读取在线/离线流程最终生成的 topoSingle 点边文件。
    default_route_file = (
        "/home/langyi/workspace/wyf/topo_graph_ws/data/regu/"
        "topoSingle_data_generated.json"
    )
    default_rviz = os.path.join(share, "rviz", "up_and_down.rviz")

    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    path_source = LaunchConfiguration("path_source")
    planner_path_topic = ParameterValue(PythonExpression([
        "'/recorded_waypoint_route/active_segment_ground' if '",
        path_source,
        "' == 'topology' else '",
        LaunchConfiguration("global_plan_topic"),
        "'",
    ]), value_type=str)
    route_spacing = ParameterValue(LaunchConfiguration("route_spacing"), value_type=float)
    normal_arrival_tolerance = ParameterValue(
        LaunchConfiguration("normal_arrival_tolerance"), value_type=float)
    corner_arrival_tolerance = ParameterValue(
        LaunchConfiguration("corner_arrival_tolerance"), value_type=float)
    goal_arrival_tolerance = ParameterValue(
        LaunchConfiguration("goal_arrival_tolerance"), value_type=float)
    finish_distance = ParameterValue(
        LaunchConfiguration("finish_distance"), value_type=float)
    body_height = ParameterValue(LaunchConfiguration("body_height"), value_type=float)
    normal_marker_diameter = ParameterValue(
        LaunchConfiguration("normal_marker_diameter"), value_type=float)
    corner_marker_diameter = ParameterValue(
        LaunchConfiguration("corner_marker_diameter"), value_type=float)
    max_start_distance = ParameterValue(
        LaunchConfiguration("max_start_distance"), value_type=float)
    direct_shortcut_enabled = ParameterValue(
        LaunchConfiguration("direct_shortcut_enabled"), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use system time on the robot; set true explicitly for rosbag playback"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "path_source", default_value="topology",
            choices=["topology", "plan"],
            description=(
                "topology: load route_file and select a target ID; "
                "plan: consume a complete ground nav_msgs/Path directly")),
        DeclareLaunchArgument(
            "global_plan_topic", default_value="/plan",
            description="Ground path topic used when path_source:=plan"),
        DeclareLaunchArgument(
            "route_file", default_value=default_route_file,
            description=(
                "Ground-referenced topoSingle JSON route. Legacy typed TXT remains "
                "supported by the loader for compatibility.")),
        DeclareLaunchArgument(
            "route_spacing", default_value="0.10",
            description="Maximum 3-D spacing of interpolated route points, in metres"),
        DeclareLaunchArgument(
            "normal_arrival_tolerance", default_value="2.80",
            description="Pass-through tolerance for ordinary route targets, in metres"),
        DeclareLaunchArgument(
            "corner_arrival_tolerance", default_value="0.3",
            description="Strict 3-D arrival tolerance for detected corners, in metres"),
        DeclareLaunchArgument(
            "goal_arrival_tolerance", default_value="0.3",
            description="Strict 3-D arrival tolerance for the selected final goal"),
        DeclareLaunchArgument(
            "finish_distance", default_value="0.10",
            description="Local follower finish distance; keep below final-goal tolerance"),
        DeclareLaunchArgument(
            "body_height", default_value="0.4",
            description="Height added to ground targets for body-centre planning"),
        DeclareLaunchArgument(
            "normal_marker_diameter", default_value="0.45",
            description="RViz diameter of ordinary route-point spheres, in metres"),
        DeclareLaunchArgument(
            "corner_marker_diameter", default_value="0.75",
            description="RViz diameter of corner route-point spheres, in metres"),
        DeclareLaunchArgument(
            "max_start_distance", default_value="3.0",
            description="Reject a goal when the nearest recorded target is farther away"),
        DeclareLaunchArgument(
            "direct_shortcut_enabled", default_value="true",
            description="Try a strict hard-and-soft-clear direct path before A-star"),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel_smoothed",
            description="Follower output; explicitly set /cmd_vel only on the real robot"),
        DeclareLaunchArgument(
            "publish_static_tf", default_value="true",
            description=(
                "Publish the corrected map->camera_init and body->base_link static TFs. "
                "When this is true, exclude /tf_static while playing a bag.")),
        DeclareLaunchArgument(
            "body_to_base_x", default_value="-0.15",
            description="base_link origin X expressed in the body frame, in metres"),
        DeclareLaunchArgument(
            "body_to_base_y", default_value="0.0",
            description="base_link origin Y expressed in the body frame, in metres"),
        DeclareLaunchArgument(
            "body_to_base_z", default_value="-0.21",
            description="base_link origin Z expressed in the body frame, in metres"),
        # 79 bag 的 /tf_static 中错误地记录了 body 与 base_link 完全重合。
        # 播包时屏蔽整个 /tf_static 后，原本正确的 map->camera_init 也会一并消失，
        # 因此这里同时补发两条静态变换，并用同一个开关避免与其他 TF 发布源冲突。
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_camera_init_static_tf",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "map",
                "--child-frame-id", "camera_init",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("publish_static_tf")),
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_base_link_static_tf",
            arguments=[
                "--x", LaunchConfiguration("body_to_base_x"),
                "--y", LaunchConfiguration("body_to_base_y"),
                "--z", LaunchConfiguration("body_to_base_z"),
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "body",
                "--child-frame-id", "base_link",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("publish_static_tf")),
            output="screen",
        ),
        Node(
            package="recorded_waypoint_route",
            executable="target_path_generator_node",
            name="target_path_generator",
            parameters=[{
                "use_sim_time": use_sim_time,
                "route_file": LaunchConfiguration("route_file"),
                "route.spacing": route_spacing,
                "route.body_height": body_height,
                "visualization.normal_marker_diameter": normal_marker_diameter,
                "visualization.corner_marker_diameter": corner_marker_diameter,
            }],
            condition=IfCondition(EqualsSubstitution(path_source, "topology")),
            output="screen",
        ),
        Node(
            package="recorded_waypoint_route",
            executable="target_route_sequencer_node",
            name="target_route_sequencer",
            parameters=[{
                "use_sim_time": use_sim_time,
                "route.normal_arrival_tolerance": normal_arrival_tolerance,
                "route.corner_arrival_tolerance": corner_arrival_tolerance,
                "route.goal_arrival_tolerance": goal_arrival_tolerance,
                "route.body_height": body_height,
                "route.spacing": route_spacing,
                "route.max_start_distance": max_start_distance,
                # C++ 节点的里程计回调只缓存；固定 50 Hz 定时器负责三维到点判断。
                "route.update_rate": 50.0,
            }],
            condition=IfCondition(EqualsSubstitution(path_source, "topology")),
            output="screen",
        ),
        Node(
            package="efficient_3d_local_planner",
            executable="local_voxel_mapper_node",
            name="local_voxel_mapper",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="efficient_3d_local_planner",
            executable="corridor_astar_planner_node",
            name="corridor_astar_planner",
            parameters=[config, {
                "use_sim_time": use_sim_time,
                "input.path_topic": planner_path_topic,
                "planner.path_height": body_height,
                "planner.direct_shortcut_enabled": direct_shortcut_enabled,
            }],
            output="screen",
        ),
        Node(
            package="efficient_3d_local_planner",
            executable="local_path_follower_node",
            name="local_path_follower",
            parameters=[config, {
                "use_sim_time": use_sim_time,
                "controller.finish_distance": finish_distance,
            }],
            remappings=[("/cmd_vel_smoothed", cmd_vel_topic)],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="efficient_3d_local_planner_rviz",
            arguments=["-d", default_rviz],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="screen",
        ),
    ])
