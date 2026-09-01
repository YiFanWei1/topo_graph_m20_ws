"""Launch the dynamic 3-D local planner from one YAML configuration file."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _require_mapping(value, name):
    if not isinstance(value, dict):
        raise RuntimeError(f"{name} must be a mapping in the YAML configuration")
    return value


def _require_vector(parameters, name, size):
    value = parameters.get(name)
    if not isinstance(value, list) or len(value) != size:
        raise RuntimeError(f"up_and_down_demo.ros__parameters.{name} must contain {size} values")
    return [str(component) for component in value]


def _launch_setup(context, package_share):
    config_path = LaunchConfiguration("config").perform(context)
    try:
        with open(config_path, "r", encoding="utf-8") as stream:
            root = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"failed to read launch configuration {config_path}: {error}") from error

    root = _require_mapping(root, "YAML root")
    demo = _require_mapping(root.get("up_and_down_demo"), "up_and_down_demo")
    parameters = _require_mapping(demo.get("ros__parameters"), "up_and_down_demo.ros__parameters")

    path_source = str(parameters.get("path_source", "topology"))
    if path_source not in ("topology", "plan"):
        raise RuntimeError("up_and_down_demo.ros__parameters.path_source must be topology or plan")

    topology_topic = str(parameters.get(
        "topics.topology_path", "/recorded_waypoint_route/active_segment_ground"))
    planner_path_topic = (
        topology_topic if path_source == "topology"
        else str(parameters.get("topics.global_plan", "/plan"))
    )
    cmd_vel_topic = str(parameters.get("topics.cmd_vel", "/cmd_vel_smoothed"))

    nodes = []
    if bool(parameters.get("static_tf.publish", True)):
        map_translation = _require_vector(parameters, "static_tf.map_to_camera_init.translation", 3)
        map_rotation = _require_vector(parameters, "static_tf.map_to_camera_init.rotation_rpy", 3)
        body_translation = _require_vector(parameters, "static_tf.body_to_base_link.translation", 3)
        body_rotation = _require_vector(parameters, "static_tf.body_to_base_link.rotation_rpy", 3)

        nodes.extend([
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_camera_init_static_tf",
                arguments=[
                    "--x", map_translation[0], "--y", map_translation[1], "--z", map_translation[2],
                    "--roll", map_rotation[0], "--pitch", map_rotation[1], "--yaw", map_rotation[2],
                    "--frame-id", str(parameters.get("static_tf.map_to_camera_init.parent_frame", "map")),
                    "--child-frame-id", str(parameters.get(
                        "static_tf.map_to_camera_init.child_frame", "camera_init")),
                ],
                parameters=[config_path],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="body_to_base_link_static_tf",
                arguments=[
                    "--x", body_translation[0], "--y", body_translation[1], "--z", body_translation[2],
                    "--roll", body_rotation[0], "--pitch", body_rotation[1], "--yaw", body_rotation[2],
                    "--frame-id", str(parameters.get("static_tf.body_to_base_link.parent_frame", "body")),
                    "--child-frame-id", str(parameters.get(
                        "static_tf.body_to_base_link.child_frame", "base_link")),
                ],
                parameters=[config_path],
                output="screen",
            ),
        ])

    if path_source == "topology":
        nodes.extend([
            Node(
                package="recorded_waypoint_route",
                executable="target_path_generator_node",
                name="target_path_generator",
                parameters=[config_path],
                output="screen",
            ),
            Node(
                package="recorded_waypoint_route",
                executable="target_route_sequencer_node",
                name="target_route_sequencer",
                parameters=[config_path],
                output="screen",
            ),
        ])

    nodes.extend([
        Node(
            package="efficient_3d_local_planner",
            executable="local_voxel_mapper_node",
            name="local_voxel_mapper",
            parameters=[config_path],
            output="screen",
        ),
        Node(
            package="efficient_3d_local_planner",
            executable="corridor_astar_planner_node",
            name="corridor_astar_planner",
            parameters=[config_path],
            remappings=[(topology_topic, planner_path_topic)],
            output="screen",
        ),
        Node(
            package="efficient_3d_local_planner",
            executable="local_path_follower_node",
            name="local_path_follower",
            parameters=[config_path],
            remappings=[("/cmd_vel_smoothed", cmd_vel_topic)],
            output="screen",
        ),
    ])

    if bool(parameters.get("rviz.enabled", True)):
        rviz_config = str(parameters.get("rviz.config_file", ""))
        if not rviz_config:
            rviz_config = os.path.join(package_share, "rviz", "up_and_down.rviz")
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="efficient_3d_local_planner_rviz",
            arguments=["-d", rviz_config],
            parameters=[config_path],
            output="screen",
        ))

    return nodes


def generate_launch_description():
    package_share = get_package_share_directory("efficient_3d_local_planner")
    default_config = os.path.join(package_share, "config", "up_and_down.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="Single YAML file containing launch settings and all ROS parameters",
        ),
        OpaqueFunction(function=_launch_setup, args=[package_share]),
    ])
