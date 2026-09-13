"""Launch the live keyboard workflow and optional static frame bridges."""

from pathlib import Path
import math

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _vector(entry, key, length):
    values = entry.get(key, [0.0] * length)
    if not isinstance(values, list) or len(values) != length:
        raise RuntimeError(f"static_tf.{key} must contain exactly {length} numbers")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError(f"static_tf.{key} contains a non-finite value")
    return result


def _static_tf_node(name, entry):
    parent = str(entry.get("parent_frame", "")).strip()
    child = str(entry.get("child_frame", "")).strip()
    if not parent or not child or parent == child:
        raise RuntimeError(
            f"static_tf.{name} requires different non-empty parent_frame and child_frame")
    xyz = _vector(entry, "translation", 3)
    rpy = _vector(entry, "rotation_rpy", 3)
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"route3d_{name}_static_tf",
        arguments=[
            "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
            "--roll", str(rpy[0]), "--pitch", str(rpy[1]), "--yaw", str(rpy[2]),
            "--frame-id", parent, "--child-frame-id", child,
        ],
        output="screen",
    )


def _launch_from_yaml(context):
    config_path = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"Route3D config file does not exist: {config_path}")
    with config_path.open("r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    if not isinstance(document.get("route3d_live"), dict):
        raise RuntimeError("YAML must contain a route3d_live mapping")
    static_config = document.get("static_tf", {})
    if not isinstance(static_config, dict):
        raise RuntimeError("static_tf must be a mapping")

    actions = [LogInfo(msg=f"Route3D product config: {config_path}")]
    for name, entry in static_config.items():
        entry = static_config.get(name, {})
        if not isinstance(entry, dict):
            raise RuntimeError(f"static_tf.{name} must be a mapping")
        if bool(entry.get("publish", False)):
            actions.append(_static_tf_node(name, entry))
            actions.append(LogInfo(msg=(
                f"Publishing static TF {entry.get('parent_frame')} -> "
                f"{entry.get('child_frame')}")))
        else:
            actions.append(LogInfo(msg=f"Static TF {name} disabled by YAML"))

    keyboard = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackagePrefix("route3d_product_demo"),
                "lib", "route3d_product_demo", "route3d_live_keyboard",
            ]),
            "--config", str(config_path),
        ],
        output="screen",
        emulate_tty=True,
    )
    actions.append(keyboard)
    actions.append(RegisterEventHandler(
        OnProcessExit(
            target_action=keyboard,
            on_exit=[EmitEvent(event=Shutdown(reason="Route3D keyboard exited"))],
        )
    ))
    return actions


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("route3d_product_demo"),
        "config",
        "live_route_product.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="YAML file for topics, online skeleton and optional static TFs",
        ),
        OpaqueFunction(function=_launch_from_yaml),
    ])
