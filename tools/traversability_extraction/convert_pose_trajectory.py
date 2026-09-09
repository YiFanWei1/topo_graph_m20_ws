#!/usr/bin/env python3
"""Convert a whitespace pose trajectory to the PointXYZI PCD used by the extractor.

Input rows are: timestamp x y z qx qy qz qw.  Despite their .json suffix,
the SLAM trajectory files supplied with outdoor1 use this plain-text format.
"""

import argparse
import json
import math
from pathlib import Path
import subprocess
import tempfile


def load_pcd_xyz(path: Path):
    """Load XYZ from a PCD by asking the installed PCL utility for ASCII output."""
    try:
        import numpy as np
    except ImportError as error:
        raise RuntimeError("--snap-surface requires numpy") from error

    with tempfile.TemporaryDirectory(prefix="trajectory_snap_") as temp_dir:
        ascii_path = Path(temp_dir) / "surface_ascii.pcd"
        try:
            subprocess.run(
                ["pcl_convert_pcd_ascii_binary", str(path), str(ascii_path), "0"],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError as error:
            raise RuntimeError(
                "pcl_convert_pcd_ascii_binary is required for --snap-surface"
            ) from error
        except subprocess.CalledProcessError as error:
            raise RuntimeError(error.stderr.strip() or f"failed to read {path}") from error

        fields = []
        header_lines = 0
        with ascii_path.open("r", encoding="utf-8") as stream:
            for line in stream:
                header_lines += 1
                words = line.split()
                if words and words[0].upper() == "FIELDS":
                    fields = words[1:]
                if words and words[0].upper() == "DATA":
                    break
        missing = [field for field in ("x", "y", "z") if field not in fields]
        if missing:
            raise RuntimeError(f"{path} has no PCD fields: {', '.join(missing)}")
        xyz_columns = [fields.index(field) for field in ("x", "y", "z")]
        return np.loadtxt(ascii_path, skiprows=header_lines, usecols=xyz_columns)


def write_pcd(path: Path, points) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write("FIELDS x y z intensity\n")
        stream.write("SIZE 4 4 4 4\n")
        stream.write("TYPE F F F F\n")
        stream.write("COUNT 1 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\n")
        stream.write("HEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\n")
        stream.write("DATA ascii\n")
        for x, y, z, timestamp in points:
            stream.write(f"{x:.9f} {y:.9f} {z:.9f} {timestamp:.6f}\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--body-height", type=float, required=True)
    parser.add_argument("--min-spacing", type=float, default=0.04)
    parser.add_argument(
        "--snap-surface", type=Path,
        help="candidate-surface PCD used to snap trajectory Z onto mapped ground",
    )
    parser.add_argument("--snap-xy-radius", type=float, default=0.35)
    parser.add_argument("--snap-max-z", type=float, default=0.80)
    parser.add_argument(
        "--snap-median-window", type=int, default=51,
        help="odd trajectory window used to reject jumps between stacked surfaces",
    )
    args = parser.parse_args()

    poses = []
    with args.input.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 8:
                raise ValueError(f"{args.input}:{line_number}: expected 8 columns")
            values = [float(value) for value in fields]
            if not all(math.isfinite(value) for value in values):
                continue
            poses.append(values)

    selected_raw = []
    for pose in poses:
        point = (pose[1], pose[2], pose[3] - args.body_height, pose[0])
        if selected_raw:
            previous = selected_raw[-1]
            distance = math.sqrt(sum((point[i] - previous[i]) ** 2 for i in range(3)))
            if distance < args.min_spacing:
                continue
        selected_raw.append(point)

    selected = list(selected_raw)
    snapped_count = 0
    z_corrections = []
    if args.snap_surface is not None:
        try:
            import numpy as np
            from scipy.ndimage import median_filter
            from scipy.spatial import cKDTree
        except ImportError as error:
            raise RuntimeError("--snap-surface requires numpy and scipy") from error
        if args.snap_median_window < 1 or args.snap_median_window % 2 == 0:
            raise ValueError("--snap-median-window must be a positive odd integer")

        surface = load_pcd_xyz(args.snap_surface)
        surface_tree = cKDTree(surface[:, :2])
        observed_corrections = []
        for x, y, z, _ in selected_raw:
            correction = math.nan
            nearby = surface_tree.query_ball_point((x, y), args.snap_xy_radius)
            if nearby:
                nearby_array = np.asarray(nearby, dtype=int)
                dz = np.abs(surface[nearby_array, 2] - z)
                valid = dz <= args.snap_max_z
                if np.any(valid):
                    valid_ids = nearby_array[valid]
                    dxy = np.hypot(
                        surface[valid_ids, 0] - x,
                        surface[valid_ids, 1] - y,
                    )
                    # Prefer the surface closest in height; XY distance resolves
                    # ambiguities between points from the same local surface.
                    score = np.abs(surface[valid_ids, 2] - z) + 0.20 * dxy
                    surface_id = valid_ids[int(np.argmin(score))]
                    snapped_z = float(surface[surface_id, 2])
                    correction = snapped_z - z
                    snapped_count += 1
            observed_corrections.append(correction)

        valid = np.isfinite(observed_corrections)
        if np.any(valid):
            point_ids = np.arange(len(selected_raw))
            interpolated = np.interp(
                point_ids, point_ids[valid], np.asarray(observed_corrections)[valid])
            smoothed = median_filter(
                interpolated, size=args.snap_median_window, mode="nearest")
            selected = [
                (x, y, z + float(smoothed[index]), timestamp)
                for index, (x, y, z, timestamp) in enumerate(selected_raw)
            ]
            z_corrections = smoothed.tolist()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    pcd_path = args.output_dir / "trajectory_ground.pcd"
    write_pcd(pcd_path, selected)
    if args.snap_surface is not None:
        write_pcd(args.output_dir / "trajectory_before_snap.pcd", selected_raw)

    summary = {
        "input": str(args.input),
        "input_pose_count": len(poses),
        "output_point_count": len(selected),
        "body_height_m": args.body_height,
        "min_spacing_m": args.min_spacing,
        "snap_surface": str(args.snap_surface) if args.snap_surface else None,
        "snap_xy_radius_m": args.snap_xy_radius if args.snap_surface else None,
        "snap_max_z_m": args.snap_max_z if args.snap_surface else None,
        "snap_median_window": args.snap_median_window if args.snap_surface else None,
        "snapped_point_count": snapped_count,
        "unsnapped_point_count": len(selected) - snapped_count,
        "mean_abs_z_correction_m": (
            sum(abs(value) for value in z_corrections) / len(z_corrections)
            if z_corrections else 0.0
        ),
        "max_abs_z_correction_m": (
            max((abs(value) for value in z_corrections), default=0.0)
        ),
    }
    (args.output_dir / "trajectory_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
