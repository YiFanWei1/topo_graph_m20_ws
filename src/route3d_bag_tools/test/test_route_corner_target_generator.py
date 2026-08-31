from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path
import sys


def load_generator_module():
    path = Path(__file__).parents[1] / "scripts" / "route_corner_target_generator"
    loader = SourceFileLoader("route_corner_target_generator_tested", str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules[loader.name] = module
    loader.exec_module(module)
    return module


GENERATOR = load_generator_module()


def make_rotation(yaw_degrees, z_values=None):
    if z_values is None:
        z_values = [0.0] * len(yaw_degrees)
    return [
        GENERATOR.TimedPose(
            stamp=float(index),
            point=(0.02 * (index % 2), 0.01 * (index % 3), z_values[index]),
            yaw=GENERATOR.math.radians(yaw),
        )
        for index, yaw in enumerate(yaw_degrees)
    ]


def detect(poses):
    clean_poses = [poses[0], poses[-1]]
    return GENERATOR.detect_yaw_corners(
        poses,
        clean_poses,
        xy_radius=0.50,
        yaw_threshold_degrees=90.0,
        minimum_duration=0.50,
        max_z_range=0.10,
        minimum_route_separation=1.0,
    )


def test_local_yaw_sweep_greater_than_90_degrees_is_corner():
    corners = detect(make_rotation([0.0, 30.0, 70.0, 120.0]))
    assert len(corners) == 1
    assert abs(corners[0].angle_degrees - 120.0) < 1e-9


def test_yaw_sweep_equal_to_or_less_than_90_degrees_is_not_corner():
    assert detect(make_rotation([0.0, 30.0, 60.0, 90.0])) == []


def test_rotation_while_height_changes_is_rejected():
    poses = make_rotation(
        [0.0, 40.0, 80.0, 120.0],
        z_values=[0.0, 0.04, 0.08, 0.12],
    )
    assert detect(poses) == []


def test_yaw_detection_defaults_match_runtime_policy():
    args = GENERATOR.parse_args([
        "--graph", "graph.json", "--output", "targets.txt"])
    assert args.corner_yaw_xy_radius == 0.50
    assert args.corner_yaw_angle_deg == 90.0
    assert args.corner_yaw_min_duration == 0.50
    assert args.corner_max_z_range == 0.10
