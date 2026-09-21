"""Generate a Route3D topology from JSON or whitespace pose rows."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys

from .topology_builder import (
    BuilderSettings,
    OdomTopologyBuilder,
    atomic_write_json,
    load_builder_settings,
    load_pose_file,
)


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='pose file: timestamp x y z qx qy qz qw')
    parser.add_argument('-o', '--output', type=Path, help='output topology JSON')
    parser.add_argument(
        '--config-file', type=Path,
        help='与在线节点共用的 ROS 2 参数 YAML；显式命令行参数优先',
    )
    parser.add_argument('--frame-id', default='camera_init')
    parser.add_argument('--target-spacing', type=float)
    parser.add_argument('--dedup-distance', type=float)
    parser.add_argument('--body-height', type=float)
    parser.add_argument('--relocation-distance', type=float)
    parser.add_argument('--obstacle-mode', type=int, choices=range(5))
    parser.add_argument(
        '--retrace', action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument(
        '--geometric-loop-closure',
        action=argparse.BooleanOptionalAction,
        default=None,
        help='enable geometry-only global route re-entry',
    )
    parser.add_argument('--loop-corridor-xy-tolerance', type=float)
    parser.add_argument('--loop-corridor-exit-xy-tolerance', type=float)
    parser.add_argument('--loop-z-tolerance', type=float)
    parser.add_argument('--loop-heading-tolerance-degrees', type=float)
    parser.add_argument('--loop-confirmation-distance', type=float)
    parser.add_argument('--loop-exit-confirmation-distance', type=float)
    parser.add_argument('--loop-minimum-graph-separation', type=float)
    parser.add_argument('--loop-minimum-time-separation', type=float)
    parser.add_argument('--loop-vertex-snap-distance', type=float)
    parser.add_argument('--loop-rejection-cooldown-distance', type=float)
    parser.add_argument(
        '--slope', action=argparse.BooleanOptionalAction, default=None)
    return parser.parse_args(argv)


def settings_from_args(args: argparse.Namespace) -> BuilderSettings:
    # Keep the historical 2 m file-mode relocation default when no YAML is used.
    defaults = replace(BuilderSettings(), relocation_distance=2.0)
    settings = (
        load_builder_settings(args.config_file, defaults=defaults)
        if args.config_file is not None else defaults)
    overrides = {
        'target_spacing': args.target_spacing,
        'dedup_distance': args.dedup_distance,
        'body_height': args.body_height,
        'relocation_distance': args.relocation_distance,
        'obstacle_mode': args.obstacle_mode,
        'retrace_enabled': args.retrace,
        'geometric_loop_closure_enabled': args.geometric_loop_closure,
        'loop_corridor_xy_tolerance': args.loop_corridor_xy_tolerance,
        'loop_corridor_exit_xy_tolerance': args.loop_corridor_exit_xy_tolerance,
        'loop_z_tolerance': args.loop_z_tolerance,
        'loop_heading_tolerance_degrees': args.loop_heading_tolerance_degrees,
        'loop_confirmation_distance': args.loop_confirmation_distance,
        'loop_exit_confirmation_distance': args.loop_exit_confirmation_distance,
        'loop_minimum_graph_separation': args.loop_minimum_graph_separation,
        'loop_minimum_time_separation': args.loop_minimum_time_separation,
        'loop_vertex_snap_distance': args.loop_vertex_snap_distance,
        'loop_rejection_cooldown_distance': args.loop_rejection_cooldown_distance,
        'slope_enabled': args.slope,
    }
    settings = replace(
        settings, **{name: value for name, value in overrides.items()
                     if value is not None})
    settings.validate()
    return settings


def main(argv=None) -> int:
    args = parse_args(argv)
    source = args.input.expanduser().resolve()
    output = (
        args.output.expanduser().resolve() if args.output is not None
        else source.with_name('topoGraph_odom.json'))
    try:
        samples = load_pose_file(source)
        settings = settings_from_args(args)
        topology = OdomTopologyBuilder(settings)
        topology.add_all(samples)
        document = topology.document(args.frame_id, finalize=True)
        atomic_write_json(output, document)
    except (OSError, TypeError, ValueError, RuntimeError) as error:
        print(f'错误：{error}', file=sys.stderr)
        return 1

    print(
        f'已生成无点云拓扑：poses={len(samples)} '
        f'vertices={len(document["vertices"])} edges={len(document["edges"])}')
    if args.config_file is not None:
        print(f'配置：{args.config_file.expanduser().resolve()}')
    print(f'输出：{output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
