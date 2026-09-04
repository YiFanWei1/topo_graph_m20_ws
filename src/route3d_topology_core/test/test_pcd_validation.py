from pathlib import Path

import numpy as np

from route3d_topology_core import (
    PcdLoopClosureValidator,
    PcdValidationConfig,
    PoseSample,
)


def write_pcd(path: Path, points: np.ndarray) -> None:
    points = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    path.write_bytes(header + points.tobytes())


def test_voxel_overlap_accepts_same_place_and_rejects_different_place(tmp_path):
    axis = np.linspace(-4.0, 4.0, 25)
    points = np.asarray([
        (x, y, 0.5 + 0.1 * ((ix + iy) % 3))
        for ix, x in enumerate(axis)
        for iy, y in enumerate(axis)
    ])
    write_pcd(tmp_path / "0.pcd", points)
    write_pcd(tmp_path / "1.pcd", points)
    write_pcd(tmp_path / "2.pcd", points)
    validator = PcdLoopClosureValidator(tmp_path, PcdValidationConfig(
        voxel_size=0.25, minimum_overlap=0.70,
        search_radius=0.50, minimum_voxels=100))

    reference = PoseSample(0.0, (0.0, 0.0, 0.0), frame_index=0)
    accepted, details = validator(
        PoseSample(10.0, (0.25, 0.0, 0.0), frame_index=1), reference)
    assert accepted
    assert details["available"]
    assert details["overlap"] >= 0.95

    accepted, details = validator(
        PoseSample(20.0, (5.0, 0.0, 0.0), frame_index=2), reference)
    assert not accepted
    assert details["available"]


def test_required_pcd_rejects_missing_file(tmp_path):
    validator = PcdLoopClosureValidator(tmp_path, PcdValidationConfig(required=True))
    accepted, details = validator(
        PoseSample(1.0, (0.0, 0.0, 0.0), frame_index=1),
        PoseSample(0.0, (0.0, 0.0, 0.0), frame_index=0))
    assert not accepted
    assert not details["available"]
