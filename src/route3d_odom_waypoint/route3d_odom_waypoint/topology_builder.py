"""Shared odometry-only topology builder for live and file modes."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
from typing import Any, Iterable

import yaml

from route3d_topology_core import (
    IncrementalCornerDetector,
    IncrementalTopologyBuilder,
    LoopClosureConfig,
    PoseSample,
    RetraceConfig,
    TopologyConfig,
    annotate_graph_slopes,
)
from route_slope_annotator.slope_analysis import SlopeConfig


@dataclass(frozen=True)
class BuilderSettings:
    target_spacing: float = 1.0
    dedup_distance: float = 0.05
    body_height: float = 0.57
    relocation_distance: float = 0.50
    retrace_enabled: bool = True
    geometric_loop_closure_enabled: bool = True
    loop_corridor_xy_tolerance: float = 0.40
    loop_corridor_exit_xy_tolerance: float = 0.55
    loop_z_tolerance: float = 0.20
    loop_heading_tolerance_degrees: float = 45.0
    loop_confirmation_distance: float = 0.80
    loop_exit_confirmation_distance: float = 0.80
    loop_minimum_graph_separation: float = 3.00
    loop_minimum_time_separation: float = 5.00
    loop_vertex_snap_distance: float = 0.45
    loop_rejection_cooldown_distance: float = 1.00
    corner_xy_radius: float = 0.50
    corner_yaw_degrees: float = 45.0
    corner_min_duration: float = 0.50
    corner_max_z_range: float = 0.10
    corner_merge_distance: float = 0.35
    slope_enabled: bool = True
    slope_config: SlopeConfig = field(default_factory=SlopeConfig)
    obstacle_mode: int = 0

    def validate(self) -> None:
        if self.obstacle_mode not in range(5):
            raise ValueError('obstacle_mode must be between 0 and 4')
        values = (
            self.target_spacing, self.dedup_distance, self.body_height,
            self.relocation_distance, self.corner_xy_radius,
            self.corner_yaw_degrees, self.corner_min_duration,
            self.corner_max_z_range, self.corner_merge_distance,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError('all numeric settings must be finite and non-negative')
        if min(
            self.target_spacing, self.dedup_distance,
            self.relocation_distance, self.corner_xy_radius,
        ) <= 0.0:
            raise ValueError('spacing, dedup, relocation and corner radius must be positive')
        if not 0.0 < self.corner_yaw_degrees < 360.0:
            raise ValueError('corner_yaw_degrees must be between 0 and 360')
        self.slope_config.validate()


def load_builder_settings(
    path: Path, *, defaults: BuilderSettings = BuilderSettings(),
) -> BuilderSettings:
    """Load builder settings from a ROS 2 parameter YAML file."""
    source = path.expanduser().resolve()
    payload = yaml.safe_load(source.read_text(encoding='utf-8'))
    if not isinstance(payload, dict):
        raise ValueError(f'配置文件必须是 YAML 对象：{source}')

    node = payload.get('route3d_odom_waypoint', payload)
    if not isinstance(node, dict):
        raise ValueError('route3d_odom_waypoint 配置必须是对象')
    parameters = node.get('ros__parameters', node)
    if not isinstance(parameters, dict):
        raise ValueError('ros__parameters 配置必须是对象')

    loop = parameters.get('loop_closure', {})
    slope = parameters.get('slope', {})
    if not isinstance(loop, dict):
        raise ValueError('loop_closure 配置必须是对象')
    if not isinstance(slope, dict):
        raise ValueError('slope 配置必须是对象')

    def value(name: str, fallback: Any) -> Any:
        return parameters.get(name, fallback)

    def nested(group: dict, prefix: str, name: str, fallback: Any) -> Any:
        return group.get(name, parameters.get(f'{prefix}.{name}', fallback))

    slope_defaults = defaults.slope_config
    settings = BuilderSettings(
        target_spacing=float(value('target_spacing', defaults.target_spacing)),
        dedup_distance=float(value('dedup_distance', defaults.dedup_distance)),
        body_height=float(value('body_height', defaults.body_height)),
        relocation_distance=float(value(
            'relocation_distance', defaults.relocation_distance)),
        retrace_enabled=bool(value('retrace_enabled', defaults.retrace_enabled)),
        geometric_loop_closure_enabled=bool(value(
            'geometric_loop_closure_enabled',
            defaults.geometric_loop_closure_enabled)),
        loop_corridor_xy_tolerance=float(nested(
            loop, 'loop_closure', 'corridor_xy_tolerance',
            defaults.loop_corridor_xy_tolerance)),
        loop_corridor_exit_xy_tolerance=float(nested(
            loop, 'loop_closure', 'corridor_exit_xy_tolerance',
            defaults.loop_corridor_exit_xy_tolerance)),
        loop_z_tolerance=float(nested(
            loop, 'loop_closure', 'z_tolerance', defaults.loop_z_tolerance)),
        loop_heading_tolerance_degrees=float(nested(
            loop, 'loop_closure', 'heading_tolerance_degrees',
            defaults.loop_heading_tolerance_degrees)),
        loop_confirmation_distance=float(nested(
            loop, 'loop_closure', 'confirmation_distance',
            defaults.loop_confirmation_distance)),
        loop_exit_confirmation_distance=float(nested(
            loop, 'loop_closure', 'exit_confirmation_distance',
            defaults.loop_exit_confirmation_distance)),
        loop_minimum_graph_separation=float(nested(
            loop, 'loop_closure', 'minimum_graph_separation',
            defaults.loop_minimum_graph_separation)),
        loop_minimum_time_separation=float(nested(
            loop, 'loop_closure', 'minimum_time_separation',
            defaults.loop_minimum_time_separation)),
        loop_vertex_snap_distance=float(nested(
            loop, 'loop_closure', 'vertex_snap_distance',
            defaults.loop_vertex_snap_distance)),
        loop_rejection_cooldown_distance=float(nested(
            loop, 'loop_closure', 'rejection_cooldown_distance',
            defaults.loop_rejection_cooldown_distance)),
        corner_xy_radius=float(value('corner_xy_radius', defaults.corner_xy_radius)),
        corner_yaw_degrees=float(value(
            'corner_yaw_degrees', defaults.corner_yaw_degrees)),
        corner_min_duration=float(value(
            'corner_min_duration', defaults.corner_min_duration)),
        corner_max_z_range=float(value(
            'corner_max_z_range', defaults.corner_max_z_range)),
        corner_merge_distance=float(value(
            'corner_merge_distance', defaults.corner_merge_distance)),
        slope_enabled=bool(value('slope_enabled', defaults.slope_enabled)),
        slope_config=SlopeConfig(
            fit_radius=float(nested(
                slope, 'slope', 'fit_radius', slope_defaults.fit_radius)),
            grade_threshold=float(nested(
                slope, 'slope', 'grade_threshold', slope_defaults.grade_threshold)),
            minimum_core_length=float(nested(
                slope, 'slope', 'minimum_core_length',
                slope_defaults.minimum_core_length)),
            minimum_height_change=float(nested(
                slope, 'slope', 'minimum_height_change',
                slope_defaults.minimum_height_change)),
            maximum_core_gap=float(nested(
                slope, 'slope', 'maximum_core_gap',
                slope_defaults.maximum_core_gap)),
            buffer_distance=float(nested(
                slope, 'slope', 'buffer_distance', slope_defaults.buffer_distance)),
        ),
        obstacle_mode=int(value('obstacle_mode', defaults.obstacle_mode)),
    )
    settings.validate()
    return settings


class OdomTopologyBuilder:
    """Build the controller topology using poses only; global loop closure stays disabled."""

    def __init__(self, settings: BuilderSettings = BuilderSettings()) -> None:
        settings.validate()
        self.settings = settings
        self.builder = IncrementalTopologyBuilder(TopologyConfig(
            target_spacing=settings.target_spacing,
            dedup_distance=settings.dedup_distance,
            body_height=settings.body_height,
            relocation_distance=settings.relocation_distance,
            retrace=RetraceConfig(enabled=settings.retrace_enabled),
            loop_closure=LoopClosureConfig(
                enabled=settings.geometric_loop_closure_enabled,
                corridor_xy_tolerance=settings.loop_corridor_xy_tolerance,
                corridor_exit_xy_tolerance=settings.loop_corridor_exit_xy_tolerance,
                z_tolerance=settings.loop_z_tolerance,
                heading_tolerance_degrees=settings.loop_heading_tolerance_degrees,
                confirmation_distance=settings.loop_confirmation_distance,
                exit_confirmation_distance=settings.loop_exit_confirmation_distance,
                minimum_graph_separation=settings.loop_minimum_graph_separation,
                minimum_time_separation=settings.loop_minimum_time_separation,
                vertex_snap_distance=settings.loop_vertex_snap_distance,
                rejection_cooldown_distance=settings.loop_rejection_cooldown_distance,
            ),
        ))
        self.corner_detector = IncrementalCornerDetector(
            settings.corner_xy_radius,
            settings.corner_yaw_degrees,
            settings.corner_min_duration,
            settings.corner_max_z_range,
        )
        self.sample_count = 0
        self.finalized = False

    def add(self, sample: PoseSample) -> None:
        if self.finalized:
            raise RuntimeError('topology has already been finalized')
        sample = sample.normalized()
        self.builder.add_pose(sample)
        corner = self.corner_detector.add_pose(sample)
        if corner is not None:
            self.builder.mark_corner(
                *corner, merge_distance=self.settings.corner_merge_distance)
        self.sample_count += 1

    def add_all(self, samples: Iterable[PoseSample]) -> None:
        for sample in samples:
            self.add(sample)

    def document(self, frame_id: str, *, finalize: bool = False) -> dict:
        if finalize and not self.finalized:
            corner = self.corner_detector.finalize()
            if corner is not None:
                self.builder.mark_corner(
                    *corner, merge_distance=self.settings.corner_merge_distance)
            self.builder.finalize()
            self.finalized = True

        slope_annotation = None
        if self.settings.slope_enabled and self.builder.edges:
            slope_annotation = annotate_graph_slopes(
                self.builder, self.settings.slope_config)
        document = self.builder.topology_dict(
            frame_id, status='complete' if self.finalized else 'recording')
        for edge in document['edges'].values():
            edge['meta']['obstacleMode'] = self.settings.obstacle_mode
        document['generation']['input'] = 'odometry_only'
        document['generation']['input_pose_count'] = self.sample_count
        document['generation']['point_cloud_used'] = False
        document['generation']['loop_closure_mode'] = (
            'geometry_only' if self.settings.geometric_loop_closure_enabled
            else 'disabled')
        document['generation']['loop_closure_parameters'] = {
            'corridor_xy_tolerance': self.settings.loop_corridor_xy_tolerance,
            'corridor_exit_xy_tolerance':
                self.settings.loop_corridor_exit_xy_tolerance,
            'z_tolerance': self.settings.loop_z_tolerance,
            'heading_tolerance_degrees':
                self.settings.loop_heading_tolerance_degrees,
            'confirmation_distance': self.settings.loop_confirmation_distance,
            'exit_confirmation_distance':
                self.settings.loop_exit_confirmation_distance,
            'minimum_graph_separation':
                self.settings.loop_minimum_graph_separation,
            'minimum_time_separation':
                self.settings.loop_minimum_time_separation,
            'vertex_snap_distance': self.settings.loop_vertex_snap_distance,
            'rejection_cooldown_distance':
                self.settings.loop_rejection_cooldown_distance,
        }
        if slope_annotation is not None:
            document['slopeAnnotation'] = slope_annotation
        return document


def atomic_write_json(path: Path, document: dict) -> None:
    destination = path.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + '.tmp')
    try:
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def _rows_from_json(payload: object) -> list:
    rows = payload.get('poses', []) if isinstance(payload, dict) else payload
    if not isinstance(rows, list):
        raise ValueError('JSON must be an array or contain a poses array')
    return rows


def _rows_from_text(text: str) -> list[list[str]]:
    rows = []
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.split('#', 1)[0].strip()
        if not line:
            continue
        values = line.split()
        if len(values) < 8:
            raise ValueError(
                f'line {line_number} must contain timestamp, xyz and quaternion')
        rows.append(values)
    return rows


def load_pose_file(path: Path) -> list[PoseSample]:
    source = path.expanduser().resolve()
    text = source.read_text(encoding='utf-8')
    try:
        rows = _rows_from_json(json.loads(text))
    except json.JSONDecodeError:
        rows = _rows_from_text(text)

    samples = []
    for index, row in enumerate(rows):
        if not isinstance(row, (list, tuple)) or len(row) < 8:
            raise ValueError(
                f'pose row {index + 1} must contain timestamp, xyz and quaternion')
        samples.append(PoseSample(
            float(row[0]),
            (float(row[1]), float(row[2]), float(row[3])),
            (float(row[4]), float(row[5]), float(row[6]), float(row[7])),
            index,
        ).normalized())
    samples.sort(key=lambda sample: sample.stamp)
    if len(samples) < 2:
        raise ValueError('pose file must contain at least two valid poses')
    return samples
