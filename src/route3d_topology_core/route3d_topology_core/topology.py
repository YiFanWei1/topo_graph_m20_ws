"""Incremental forest builder with adjacent-edge retrace suppression."""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import json
import math
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from route_slope_annotator.slope_analysis import SlopeConfig, annotate_document

from .schema import apply_topology_schema


Point3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
EventCallback = Callable[[str, dict], None]
LoopClosureValidator = Callable[["PoseSample", "PoseSample"], Tuple[bool, dict]]


def distance_3d(first: Point3, second: Point3) -> float:
    return math.sqrt(sum((first[i] - second[i]) ** 2 for i in range(3)))


def distance_xy(first: Point3, second: Point3) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def normalize_quaternion(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def quaternion_to_rpy(q: Quaternion) -> List[float]:
    qx, qy, qz, qw = q
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return [roll, pitch, yaw]


def interpolate_quaternion(first: Quaternion, second: Quaternion, ratio: float) -> Quaternion:
    if sum(a * b for a, b in zip(first, second)) < 0.0:
        second = tuple(-value for value in second)  # type: ignore[assignment]
    return normalize_quaternion(tuple(
        first[i] + ratio * (second[i] - first[i]) for i in range(4)
    ))  # type: ignore[arg-type]


def interpolate_sample(
    first: "PoseSample", second: "PoseSample", ratio: float,
) -> "PoseSample":
    ratio = max(0.0, min(1.0, ratio))
    return PoseSample(
        stamp=first.stamp + ratio * (second.stamp - first.stamp),
        point=tuple(
            first.point[i] + ratio * (second.point[i] - first.point[i])
            for i in range(3)),  # type: ignore[arg-type]
        quaternion=interpolate_quaternion(first.quaternion, second.quaternion, ratio),
        frame_index=(first.frame_index if ratio < 0.5 else second.frame_index),
    )


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    point: Point3
    quaternion: Quaternion = (0.0, 0.0, 0.0, 1.0)
    frame_index: Optional[int] = None

    def normalized(self) -> "PoseSample":
        values = (self.stamp, *self.point, *self.quaternion)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose sample must contain finite values")
        if self.frame_index is not None and self.frame_index < 0:
            raise ValueError("frame index must be non-negative when provided")
        return PoseSample(
            self.stamp, self.point, normalize_quaternion(self.quaternion),
            self.frame_index)


@dataclass
class Vertex:
    vertex_id: int
    sample: PoseSample
    component: int
    is_corner: bool = False
    is_slope: bool = False
    turn_degrees: float = 0.0
    is_junction: bool = False


@dataclass
class Edge:
    edge_id: int
    first: int
    second: int
    source: str = "discovery"


@dataclass(frozen=True)
class RetraceConfig:
    enabled: bool = True
    corridor_xy_tolerance: float = 0.40
    corridor_exit_xy_tolerance: float = 0.45
    z_tolerance: float = 0.20
    heading_tolerance_degrees: float = 35.0
    entry_confirmation_distance: float = 0.30
    exit_confirmation_distance: float = 0.30
    vertex_snap_distance: float = 0.30
    motion_min_distance: float = 0.10

    def validate(self) -> None:
        numeric = (
            self.corridor_xy_tolerance, self.corridor_exit_xy_tolerance,
            self.z_tolerance, self.heading_tolerance_degrees,
            self.entry_confirmation_distance, self.exit_confirmation_distance,
            self.vertex_snap_distance, self.motion_min_distance,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in numeric):
            raise ValueError("retrace parameters must be finite and non-negative")
        if self.corridor_xy_tolerance <= 0.0:
            raise ValueError("corridor_xy_tolerance must be positive")
        if self.corridor_exit_xy_tolerance < self.corridor_xy_tolerance:
            raise ValueError("corridor_exit_xy_tolerance must not be smaller than entry tolerance")
        if not 0.0 < self.heading_tolerance_degrees <= 90.0:
            raise ValueError("heading_tolerance_degrees must be between 0 and 90")


@dataclass(frozen=True)
class LoopClosureConfig:
    """Conservative global route re-entry used to close and re-traverse loops."""

    enabled: bool = True
    corridor_xy_tolerance: float = 0.40
    corridor_exit_xy_tolerance: float = 0.55
    z_tolerance: float = 0.20
    heading_tolerance_degrees: float = 45.0
    confirmation_distance: float = 0.80
    exit_confirmation_distance: float = 0.80
    minimum_graph_separation: float = 3.00
    minimum_time_separation: float = 5.00
    vertex_snap_distance: float = 0.45
    rejection_cooldown_distance: float = 1.00

    def validate(self) -> None:
        numeric = (
            self.corridor_xy_tolerance, self.corridor_exit_xy_tolerance,
            self.z_tolerance, self.heading_tolerance_degrees,
            self.confirmation_distance, self.exit_confirmation_distance,
            self.minimum_graph_separation,
            self.minimum_time_separation, self.vertex_snap_distance,
            self.rejection_cooldown_distance,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in numeric):
            raise ValueError("loop closure parameters must be finite and non-negative")
        if self.corridor_xy_tolerance <= 0.0:
            raise ValueError("loop closure corridor_xy_tolerance must be positive")
        if self.corridor_exit_xy_tolerance < self.corridor_xy_tolerance:
            raise ValueError(
                "loop closure exit tolerance must not be smaller than entry tolerance")
        if not 0.0 < self.heading_tolerance_degrees <= 90.0:
            raise ValueError(
                "loop closure heading_tolerance_degrees must be between 0 and 90")


@dataclass(frozen=True)
class TopologyConfig:
    target_spacing: float = 1.0
    dedup_distance: float = 0.05
    body_height: float = 0.57
    relocation_distance: float = 0.50
    topo_type: int = 0
    topo_pcd: str = ""
    topo_acc: float = 0.50
    retrace: RetraceConfig = field(default_factory=RetraceConfig)
    loop_closure: LoopClosureConfig = field(default_factory=LoopClosureConfig)

    def validate(self) -> None:
        positive = (self.target_spacing, self.dedup_distance, self.relocation_distance)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("spacing, dedup and relocation distances must be finite and positive")
        if not math.isfinite(self.body_height) or self.body_height < 0.0:
            raise ValueError("body_height must be finite and non-negative")
        if not math.isfinite(self.topo_acc) or self.topo_acc < 0.0:
            raise ValueError("topo_acc must be finite and non-negative")
        self.retrace.validate()
        self.loop_closure.validate()


class IncrementalCornerDetector:
    """Incremental local-position yaw sweep detector shared by live and replay modes."""

    def __init__(
        self, xy_radius: float = 0.50, yaw_degrees: float = 45.0,
        minimum_duration: float = 0.50, maximum_z_range: float = 0.10,
    ) -> None:
        self.xy_radius = xy_radius
        self.yaw_degrees = yaw_degrees
        self.minimum_duration = minimum_duration
        self.maximum_z_range = maximum_z_range
        self.window: List[PoseSample] = []

    @staticmethod
    def _yaw(sample: PoseSample) -> float:
        return quaternion_to_rpy(sample.quaternion)[2]

    @staticmethod
    def _unwrap(previous: float, current: float) -> float:
        while current - previous > math.pi:
            current -= 2.0 * math.pi
        while current - previous < -math.pi:
            current += 2.0 * math.pi
        return current

    def _candidate(self) -> Optional[Tuple[PoseSample, float]]:
        if len(self.window) < 2:
            return None
        yaws = [self._yaw(self.window[0])]
        for sample in self.window[1:]:
            yaws.append(self._unwrap(yaws[-1], self._yaw(sample)))
        turn = math.degrees(max(yaws) - min(yaws))
        duration = self.window[-1].stamp - self.window[0].stamp
        if turn <= self.yaw_degrees or duration < self.minimum_duration:
            return None
        middle = 0.5 * (self.window[0].stamp + self.window[-1].stamp)
        return min(self.window, key=lambda item: abs(item.stamp - middle)), turn

    def add_pose(self, sample: PoseSample) -> Optional[Tuple[PoseSample, float]]:
        sample = sample.normalized()
        if not self.window:
            self.window = [sample]
            return None
        anchor = self.window[0]
        points = [item.point for item in self.window]
        minimum_z = min(point[2] for point in points)
        maximum_z = max(point[2] for point in points)
        inside = distance_xy(anchor.point, sample.point) <= self.xy_radius
        z_range = max(maximum_z, sample.point[2]) - min(minimum_z, sample.point[2])
        if inside and z_range <= self.maximum_z_range:
            self.window.append(sample)
            return None
        candidate = self._candidate()
        self.window = [sample]
        return candidate

    def finalize(self) -> Optional[Tuple[PoseSample, float]]:
        candidate = self._candidate()
        self.window = []
        return candidate


@dataclass(frozen=True)
class Projection:
    edge_id: int
    ratio: float
    raw_ratio: float
    point: Point3
    xy_distance: float
    z_distance: float
    heading_error: float


class IncrementalTopologyBuilder:
    """Build a deterministic graph and suppress traversal of known route edges."""

    DISCOVERING = "DISCOVERING"
    RETRACE_PENDING = "RETRACE_PENDING"
    RETRACING = "RETRACING"
    EXIT_PENDING = "EXIT_PENDING"

    def __init__(
        self,
        config: TopologyConfig = TopologyConfig(),
        event_callback: Optional[EventCallback] = None,
        loop_closure_validator: Optional[LoopClosureValidator] = None,
    ) -> None:
        config.validate()
        self.config = config
        self.event_callback = event_callback
        self.loop_closure_validator = loop_closure_validator
        self.vertices: Dict[int, Vertex] = {}
        self.edges: Dict[int, Edge] = {}
        self.adjacency: Dict[int, set[int]] = {}
        self.next_vertex_id = 1
        self.next_edge_id = 1
        self.component = 0
        self.state = self.DISCOVERING
        self.version = 0
        self.current_vertex_id: Optional[int] = None
        self.current_retrace_edge_id: Optional[int] = None
        self.current_retrace_ratio = 0.0
        self.last_sample: Optional[PoseSample] = None
        self.last_input_sample: Optional[PoseSample] = None
        self.discovery_cursor: Optional[PoseSample] = None
        self.discovery_distance = 0.0
        self.last_motion: Optional[Point3] = None
        self.pending_edge_id: Optional[int] = None
        self.pending_kind = "retrace"
        self.pending_source_vertex_id: Optional[int] = None
        self.pending_started_stamp = 0.0
        self.pending_join_edge_id: Optional[int] = None
        self.pending_join_ratio = 0.0
        self.pending_join_vertex_id: Optional[int] = None
        self.pending_samples: List[PoseSample] = []
        self.pending_distance = 0.0
        self.pending_gap_distance = 0.0
        self.exit_anchor_edge_id: Optional[int] = None
        self.exit_anchor_ratio = 0.0
        self.exit_anchor_vertex_id: Optional[int] = None
        self.raw_samples: List[PoseSample] = []
        self.jump_points: List[Point3] = []
        self.retrace_count = 0
        self.route_reentry_count = 0
        self.loop_closure_count = 0
        self.loop_validation_rejection_count = 0
        self.loop_validation_available_count = 0
        self.tracking_kind = "retrace"
        self.global_search_cooldown = 0.0
        self.edge_split_count = 0
        self.suppressed_distance = 0.0
        self.finalized = False

    def _event(self, name: str, values: dict) -> None:
        if self.event_callback is not None:
            self.event_callback(name, values)

    def _touch(self) -> None:
        self.version += 1

    def _add_vertex(
        self, sample: PoseSample, *, junction: bool = False,
        is_corner: bool = False, turn_degrees: float = 0.0,
    ) -> int:
        vertex_id = self.next_vertex_id
        self.next_vertex_id += 1
        self.vertices[vertex_id] = Vertex(
            vertex_id, sample, self.component, is_corner=is_corner,
            turn_degrees=turn_degrees, is_junction=junction)
        self.adjacency[vertex_id] = set()
        self._touch()
        self._event("vertex_added", {
            "vertex_id": vertex_id, "stamp": sample.stamp,
            "point": list(sample.point), "component": self.component,
            "junction": junction,
        })
        return vertex_id

    def _edge_between(self, first: int, second: int) -> Optional[int]:
        for edge_id in self.adjacency.get(first, set()):
            edge = self.edges[edge_id]
            if {edge.first, edge.second} == {first, second}:
                return edge_id
        return None

    def _add_edge(self, first: int, second: int, source: str = "discovery") -> int:
        if first == second:
            raise ValueError("self edges are not allowed")
        existing = self._edge_between(first, second)
        if existing is not None:
            return existing
        edge_id = self.next_edge_id
        self.next_edge_id += 1
        self.edges[edge_id] = Edge(edge_id, first, second, source)
        self.adjacency[first].add(edge_id)
        self.adjacency[second].add(edge_id)
        self._touch()
        return edge_id

    def _remove_edge(self, edge_id: int) -> None:
        edge = self.edges.pop(edge_id)
        self.adjacency[edge.first].discard(edge_id)
        self.adjacency[edge.second].discard(edge_id)
        self._touch()

    def _edge_samples(self, edge_id: int) -> Tuple[PoseSample, PoseSample]:
        edge = self.edges[edge_id]
        return self.vertices[edge.first].sample, self.vertices[edge.second].sample

    @staticmethod
    def _motion(first: PoseSample, second: PoseSample) -> Point3:
        return tuple(
            second.point[i] - first.point[i] for i in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def _heading_error(motion: Point3, edge_vector: Point3) -> float:
        motion_norm = math.hypot(motion[0], motion[1])
        edge_norm = math.hypot(edge_vector[0], edge_vector[1])
        if motion_norm <= 1.0e-9 or edge_norm <= 1.0e-9:
            return 0.0
        cosine = abs((motion[0] * edge_vector[0] + motion[1] * edge_vector[1]) /
                     (motion_norm * edge_norm))
        return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))

    def _project(self, edge_id: int, sample: PoseSample, motion: Point3) -> Projection:
        first, second = self._edge_samples(edge_id)
        dx = second.point[0] - first.point[0]
        dy = second.point[1] - first.point[1]
        denominator = dx * dx + dy * dy
        if denominator <= 1.0e-12:
            raw_ratio = 0.0
        else:
            raw_ratio = ((sample.point[0] - first.point[0]) * dx +
                         (sample.point[1] - first.point[1]) * dy) / denominator
        ratio = max(0.0, min(1.0, raw_ratio))
        point = tuple(
            first.point[i] + ratio * (second.point[i] - first.point[i]) for i in range(3)
        )  # type: ignore[assignment]
        heading_error = self._heading_error(
            motion, (dx, dy, second.point[2] - first.point[2]))
        if math.hypot(motion[0], motion[1]) < \
                self.config.retrace.motion_min_distance:
            heading_error = 0.0
        return Projection(
            edge_id=edge_id,
            ratio=ratio,
            raw_ratio=raw_ratio,
            point=point,
            xy_distance=distance_xy(sample.point, point),
            z_distance=abs(sample.point[2] - point[2]),
            heading_error=heading_error,
        )

    def _projection_matches(self, projection: Projection, exit_mode: bool = False) -> bool:
        retrace = self.config.retrace
        xy_limit = (
            retrace.corridor_exit_xy_tolerance
            if exit_mode else retrace.corridor_xy_tolerance)
        return (
            projection.xy_distance <= xy_limit + 1.0e-9 and
            projection.z_distance <= retrace.z_tolerance + 1.0e-9 and
            projection.heading_error <= retrace.heading_tolerance_degrees + 1.0e-9
        )

    def _loop_projection_matches(
        self, projection: Projection, exit_mode: bool = False,
    ) -> bool:
        loop = self.config.loop_closure
        xy_limit = (
            loop.corridor_exit_xy_tolerance
            if exit_mode else loop.corridor_xy_tolerance)
        return (
            projection.xy_distance <= xy_limit + 1.0e-9 and
            projection.z_distance <= loop.z_tolerance + 1.0e-9 and
            projection.heading_error <= loop.heading_tolerance_degrees + 1.0e-9
        )

    def _matches_tracking_mode(
        self, projection: Projection, *, exit_mode: bool = False,
        tracking_kind: Optional[str] = None,
    ) -> bool:
        kind = tracking_kind or self.tracking_kind
        if kind == "loop":
            return self._loop_projection_matches(projection, exit_mode)
        return self._projection_matches(projection, exit_mode)

    def _incident_best_projection(
        self, vertex_id: int, sample: PoseSample, motion: Point3,
        *, require_inside: bool, tracking_kind: str = "retrace",
        exit_mode: bool = False,
    ) -> Optional[Projection]:
        candidates = []
        for edge_id in sorted(self.adjacency.get(vertex_id, set())):
            projection = self._project(edge_id, sample, motion)
            # A candidate reached through a vertex must still project onto the
            # finite edge segment.  Checking only the bound nearest that vertex
            # allowed a point beyond the far endpoint to select the wrong edge.
            moving_inside = -1.0e-6 <= projection.raw_ratio <= 1.0 + 1.0e-6
            if require_inside and not moving_inside:
                continue
            if self._matches_tracking_mode(
                projection, exit_mode=exit_mode, tracking_kind=tracking_kind,
            ):
                candidates.append(projection)
        if not candidates:
            return None
        return min(candidates, key=lambda item: (
            item.xy_distance, item.heading_error, item.edge_id))

    def _graph_distances(self, source: int) -> Dict[int, float]:
        """Return deterministic shortest graph distances from a source vertex."""
        distances = {source: 0.0}
        queue = [(0.0, source)]
        while queue:
            distance, vertex_id = heapq.heappop(queue)
            if distance > distances.get(vertex_id, math.inf) + 1.0e-12:
                continue
            for edge_id in sorted(self.adjacency.get(vertex_id, set())):
                edge = self.edges[edge_id]
                neighbor = edge.second if edge.first == vertex_id else edge.first
                weight = distance_3d(
                    self.vertices[edge.first].sample.point,
                    self.vertices[edge.second].sample.point)
                candidate = distance + weight
                if candidate + 1.0e-12 < distances.get(neighbor, math.inf):
                    distances[neighbor] = candidate
                    heapq.heappush(queue, (candidate, neighbor))
        return distances

    def _global_best_projection(
        self, sample: PoseSample, motion: Point3,
    ) -> Optional[Projection]:
        """Find a geometrically matching old edge far away in graph distance."""
        loop = self.config.loop_closure
        if (
            not loop.enabled or self.current_vertex_id is None or
            self.global_search_cooldown > 1.0e-9 or len(self.edges) < 3
        ):
            return None
        distances = self._graph_distances(self.current_vertex_id)
        candidates = []
        for edge_id in sorted(self.edges):
            edge = self.edges[edge_id]
            if self.vertices[edge.first].component != self.component or \
                    self.vertices[edge.second].component != self.component:
                continue
            graph_separation = min(
                distances.get(edge.first, math.inf),
                distances.get(edge.second, math.inf))
            if graph_separation + 1.0e-9 < loop.minimum_graph_separation:
                continue
            projection = self._project(edge_id, sample, motion)
            if not -1.0e-6 <= projection.raw_ratio <= 1.0 + 1.0e-6:
                continue
            if self._loop_projection_matches(projection):
                candidates.append(projection)
        if not candidates:
            return None
        return min(candidates, key=lambda item: (
            item.xy_distance, item.heading_error, item.edge_id))

    def _historical_reference(
        self, point: Point3, current: PoseSample, started_stamp: float,
    ) -> Optional[PoseSample]:
        cutoff = started_stamp - self.config.loop_closure.minimum_time_separation
        candidates = [
            sample for sample in self.raw_samples
            if sample.stamp <= cutoff and sample.frame_index is not None]
        if not candidates:
            return None
        reference = min(candidates, key=lambda item: (
            distance_3d(item.point, point), item.stamp))
        if distance_xy(reference.point, point) > \
                self.config.loop_closure.corridor_exit_xy_tolerance + 1.0e-9:
            return None
        if abs(reference.point[2] - point[2]) > \
                self.config.loop_closure.z_tolerance + 1.0e-9:
            return None
        return reference

    def _commit_discovery_endpoint(self, sample: PoseSample) -> int:
        if self.current_vertex_id is None:
            vertex_id = self._add_vertex(sample)
        elif distance_3d(
            self.vertices[self.current_vertex_id].sample.point, sample.point,
        ) <= 1.0e-9:
            vertex_id = self.current_vertex_id
        else:
            vertex_id = self._add_vertex(sample)
            self._add_edge(self.current_vertex_id, vertex_id)
        self.current_vertex_id = vertex_id
        self.discovery_cursor = sample
        self.discovery_distance = 0.0
        return vertex_id

    def _append_discovery(self, sample: PoseSample) -> None:
        if self.current_vertex_id is None:
            self.current_vertex_id = self._add_vertex(sample)
            self.discovery_cursor = sample
            return
        if self.discovery_cursor is None:
            self.discovery_cursor = self.vertices[self.current_vertex_id].sample
        cursor = self.discovery_cursor
        segment_length = distance_3d(cursor.point, sample.point)
        if segment_length <= 1.0e-12:
            self.discovery_cursor = sample
            return
        consumed = 0.0
        while self.discovery_distance + (segment_length - consumed) >= \
                self.config.target_spacing - 1.0e-9:
            needed = self.config.target_spacing - self.discovery_distance
            ratio = (consumed + needed) / segment_length
            node_sample = interpolate_sample(cursor, sample, ratio)
            new_id = self._add_vertex(node_sample)
            self._add_edge(self.current_vertex_id, new_id)
            self.current_vertex_id = new_id
            consumed += needed
            self.discovery_distance = 0.0
        self.discovery_distance += segment_length - consumed
        self.discovery_cursor = sample

    def _detect_reversal(self, sample: PoseSample, motion: Point3) -> None:
        if self.last_motion is None or self.discovery_cursor is None:
            return
        old_norm = math.hypot(self.last_motion[0], self.last_motion[1])
        new_norm = math.hypot(motion[0], motion[1])
        if old_norm < self.config.retrace.motion_min_distance or new_norm < 1.0e-9:
            return
        cosine = (
            self.last_motion[0] * motion[0] + self.last_motion[1] * motion[1]
        ) / (old_norm * new_norm)
        reversal_limit = -math.cos(math.radians(
            self.config.retrace.heading_tolerance_degrees))
        if cosine <= reversal_limit and self.discovery_distance >= self.config.dedup_distance:
            self._commit_discovery_endpoint(self.last_sample or self.discovery_cursor)

    def _start_retrace_pending(
        self, projection: Projection, sample: PoseSample, step: float,
    ) -> None:
        self.state = self.RETRACE_PENDING
        self.pending_kind = "retrace"
        self.pending_edge_id = projection.edge_id
        self.pending_source_vertex_id = self.current_vertex_id
        self.pending_started_stamp = sample.stamp
        self.pending_join_edge_id = None
        self.pending_join_vertex_id = None
        self.pending_samples = [sample]
        self.pending_distance = step
        self.pending_gap_distance = 0.0

    def _start_loop_pending(
        self, projection: Projection, sample: PoseSample, step: float,
    ) -> None:
        self.state = self.RETRACE_PENDING
        self.pending_kind = "loop"
        self.pending_edge_id = projection.edge_id
        self.pending_source_vertex_id = self.current_vertex_id
        self.pending_started_stamp = sample.stamp
        self.pending_join_edge_id = projection.edge_id
        self.pending_join_ratio = projection.ratio
        self.pending_join_vertex_id = None
        self.pending_samples = [sample]
        self.pending_distance = step
        self.pending_gap_distance = 0.0
        self._event("route_reentry_candidate", {
            "edge_id": projection.edge_id,
            "stamp": sample.stamp,
            "xy_distance_m": projection.xy_distance,
            "heading_error_deg": projection.heading_error,
        })

    def _clear_pending_metadata(self) -> None:
        self.pending_edge_id = None
        self.pending_kind = "retrace"
        self.pending_source_vertex_id = None
        self.pending_started_stamp = 0.0
        self.pending_join_edge_id = None
        self.pending_join_ratio = 0.0
        self.pending_join_vertex_id = None
        self.pending_samples = []
        self.pending_distance = 0.0
        self.pending_gap_distance = 0.0

    def _validate_loop_reentry(self, projection: Projection) -> Tuple[bool, dict]:
        current = self.pending_samples[-1]
        reference = self._historical_reference(
            projection.point, current, self.pending_started_stamp)
        if self.loop_closure_validator is None:
            return True, {"available": False, "reason": "validator_disabled"}
        validator_required = bool(getattr(
            getattr(self.loop_closure_validator, "config", None),
            "required", False))
        if reference is None:
            return (not validator_required), {
                "available": False, "accepted": not validator_required,
                "reason": "reference_unavailable"}
        try:
            accepted, details = self.loop_closure_validator(current, reference)
        except (OSError, ValueError, RuntimeError) as error:
            return (not validator_required), {
                "available": False, "accepted": not validator_required,
                "reason": str(error)}
        details = dict(details)
        details.setdefault("available", True)
        details["current_frame_index"] = current.frame_index
        details["reference_frame_index"] = reference.frame_index
        if details.get("available"):
            self.loop_validation_available_count += 1
        return bool(accepted), details

    def _enter_retrace(self) -> bool:
        if self.pending_edge_id not in self.edges or not self.pending_samples:
            self._cancel_retrace_pending()
            return False
        kind = self.pending_kind
        motion = self.last_motion or (1.0, 0.0, 0.0)
        current_projection = self._project(
            self.pending_edge_id, self.pending_samples[-1], motion)
        validation = {"available": False, "reason": "not_loop_reentry"}
        if kind == "loop":
            accepted, validation = self._validate_loop_reentry(current_projection)
            if not accepted:
                self.loop_validation_rejection_count += 1
                self._event("route_reentry_rejected", {
                    "edge_id": self.pending_edge_id,
                    "stamp": self.pending_samples[-1].stamp,
                    "validation": validation,
                })
                self._cancel_retrace_pending()
                self.global_search_cooldown = \
                    self.config.loop_closure.rejection_cooldown_distance
                return False

            source_id = self.pending_source_vertex_id
            if source_id not in self.vertices:
                self._cancel_retrace_pending()
                return False
            if self.pending_join_vertex_id in self.vertices:
                join_id = self.pending_join_vertex_id
            elif self.pending_join_edge_id in self.edges:
                join_edge_id = self.pending_join_edge_id
                join_first, join_second = self._edge_samples(join_edge_id)
                ratio = self.pending_join_ratio
                join_point = tuple(
                    join_first.point[i] + ratio *
                    (join_second.point[i] - join_first.point[i])
                    for i in range(3))
                join_projection = Projection(
                    join_edge_id, ratio, ratio, join_point,
                    distance_xy(self.pending_samples[0].point, join_point),
                    abs(self.pending_samples[0].point[2] - join_point[2]), 0.0)
                join_id = self._snap_projection_vertex(
                    join_projection,
                    threshold=self.config.loop_closure.vertex_snap_distance)
                if join_id is None:
                    join_id = self._split_edge(
                        join_edge_id, ratio, self.pending_samples[0])
            else:
                self._cancel_retrace_pending()
                return False

            existing = self._edge_between(source_id, join_id)
            closure_edge_id = (
                existing if existing is not None
                else self._add_edge(source_id, join_id, "loop_closure"))
            if existing is None:
                self.loop_closure_count += 1
            for vertex_id in (source_id, join_id):
                if len(self.adjacency[vertex_id]) >= 3:
                    self.vertices[vertex_id].is_junction = True
            self.route_reentry_count += 1
            self._event("loop_closure", {
                "closure_edge_id": closure_edge_id,
                "source_vertex_id": source_id,
                "join_vertex_id": join_id,
                "matched_edge_id": self.pending_edge_id,
                "stamp": self.pending_samples[-1].stamp,
                "confirmation_distance_m": self.pending_distance,
                "validation": validation,
            })

        # Joining in the middle of the matched edge can split and remap it.
        # Recompute the live cursor against the surviving edge id.
        current_projection = self._project(
            self.pending_edge_id, self.pending_samples[-1], motion)
        self.state = self.RETRACING
        self.current_retrace_edge_id = self.pending_edge_id
        if self.current_retrace_edge_id is not None and self.pending_samples:
            self.current_retrace_ratio = self._project(
                self.current_retrace_edge_id, self.pending_samples[-1], motion).ratio
        self.suppressed_distance += self.pending_distance
        self.tracking_kind = kind
        if kind == "retrace":
            self.retrace_count += 1
            self._event("retrace_enter", {
                "edge_id": self.current_retrace_edge_id,
                "stamp": self.pending_samples[-1].stamp,
                "confirmation_distance": self.pending_distance,
            })
        else:
            self._event("route_reentry_enter", {
                "edge_id": self.current_retrace_edge_id,
                "stamp": self.pending_samples[-1].stamp,
                "confirmation_distance": self.pending_distance,
                "validation": validation,
            })
        snapped = self._snap_projection_vertex(
            current_projection,
            threshold=(self.config.loop_closure.vertex_snap_distance
                       if kind == "loop" else None))
        self.current_vertex_id = snapped
        self._clear_pending_metadata()
        return True

    def _cancel_retrace_pending(self) -> None:
        buffered = list(self.pending_samples)
        source_id = self.pending_source_vertex_id
        self.state = self.DISCOVERING
        if source_id in self.vertices:
            self.current_vertex_id = source_id
        self._clear_pending_metadata()
        for buffered_sample in buffered:
            self._append_discovery(buffered_sample)

    def _snap_projection_vertex(
        self, projection: Projection, threshold: Optional[float] = None,
    ) -> Optional[int]:
        edge = self.edges[projection.edge_id]
        first_distance = distance_3d(projection.point, self.vertices[edge.first].sample.point)
        second_distance = distance_3d(projection.point, self.vertices[edge.second].sample.point)
        limit = (
            self.config.retrace.vertex_snap_distance
            if threshold is None else threshold)
        candidates = []
        if first_distance <= limit:
            candidates.append((first_distance, edge.first))
        if second_distance <= limit:
            candidates.append((second_distance, edge.second))
        return min(candidates)[1] if candidates else None

    def _begin_exit(self, projection: Optional[Projection], sample: PoseSample) -> None:
        self.state = self.EXIT_PENDING
        self.pending_samples = [sample]
        self.pending_distance = (
            distance_3d(self.last_sample.point, sample.point)
            if self.last_sample else 0.0)
        self.exit_anchor_vertex_id = None
        self.exit_anchor_edge_id = None
        if projection is not None and projection.edge_id in self.edges:
            snapped = self._snap_projection_vertex(
                projection,
                threshold=(self.config.loop_closure.vertex_snap_distance
                           if self.tracking_kind == "loop" else None))
            if snapped is not None:
                self.exit_anchor_vertex_id = snapped
            else:
                self.exit_anchor_edge_id = projection.edge_id
                self.exit_anchor_ratio = projection.ratio
        elif self.current_vertex_id is not None:
            self.exit_anchor_vertex_id = self.current_vertex_id

    def _split_edge(
        self, edge_id: int, ratio: float,
        sample_hint: Optional[PoseSample] = None,
    ) -> int:
        edge = self.edges[edge_id]
        first = self.vertices[edge.first].sample
        second = self.vertices[edge.second].sample
        split_sample = interpolate_sample(first, second, ratio)
        if sample_hint is not None:
            split_sample = PoseSample(
                sample_hint.stamp, split_sample.point, sample_hint.quaternion,
                split_sample.frame_index)
        tracked_exit = self.exit_anchor_edge_id == edge_id
        tracked_exit_ratio = self.exit_anchor_ratio
        tracked_retrace = self.current_retrace_edge_id == edge_id
        tracked_retrace_ratio = self.current_retrace_ratio
        tracked_pending = self.pending_edge_id == edge_id
        tracked_pending_ratio = None
        if tracked_pending and self.pending_samples:
            tracked_pending_ratio = self._project(
                edge_id, self.pending_samples[-1],
                self.last_motion or (1.0, 0.0, 0.0)).ratio
        tracked_join = self.pending_join_edge_id == edge_id
        tracked_join_ratio = self.pending_join_ratio
        self._remove_edge(edge_id)
        junction_id = self._add_vertex(split_sample, junction=True)
        first_edge_id = self._add_edge(edge.first, junction_id, "edge_split")
        second_edge_id = self._add_edge(junction_id, edge.second, "edge_split")

        def remap(old_ratio: float) -> Tuple[int, float]:
            if old_ratio <= ratio:
                return first_edge_id, old_ratio / ratio if ratio > 1.0e-9 else 0.0
            denominator = 1.0 - ratio
            return second_edge_id, (
                (old_ratio - ratio) / denominator if denominator > 1.0e-9 else 1.0)

        # Corner insertion and retrace-exit splitting use the same primitive.
        # Keep every state-machine reference valid when the old edge disappears.
        if tracked_exit:
            exit_point = interpolate_sample(first, second, tracked_exit_ratio).point
            exit_snap_distance = (
                self.config.loop_closure.vertex_snap_distance
                if self.tracking_kind == "loop"
                else self.config.retrace.vertex_snap_distance)
            if distance_3d(exit_point, split_sample.point) <= \
                    exit_snap_distance:
                self.exit_anchor_vertex_id = junction_id
                self.exit_anchor_edge_id = None
            else:
                self.exit_anchor_edge_id, self.exit_anchor_ratio = remap(tracked_exit_ratio)
        if tracked_retrace:
            self.current_retrace_edge_id, self.current_retrace_ratio = remap(
                tracked_retrace_ratio)
        if tracked_pending:
            self.pending_edge_id, _ = remap(
                tracked_pending_ratio if tracked_pending_ratio is not None else ratio)
        if tracked_join:
            join_point = interpolate_sample(first, second, tracked_join_ratio).point
            if distance_3d(join_point, split_sample.point) <= \
                    self.config.loop_closure.vertex_snap_distance:
                self.pending_join_vertex_id = junction_id
                self.pending_join_edge_id = None
            else:
                self.pending_join_edge_id, self.pending_join_ratio = remap(
                    tracked_join_ratio)
        self.edge_split_count += 1
        self._event("edge_split", {
            "old_edge_id": edge_id, "junction_vertex_id": junction_id,
            "endpoints": [edge.first, edge.second], "ratio": ratio,
        })
        return junction_id

    def _confirm_exit(self) -> None:
        tracking_kind = self.tracking_kind
        if self.exit_anchor_vertex_id is not None:
            anchor_id = self.exit_anchor_vertex_id
        elif self.exit_anchor_edge_id is not None and self.exit_anchor_edge_id in self.edges:
            anchor_id = self._split_edge(
                self.exit_anchor_edge_id, self.exit_anchor_ratio,
                self.pending_samples[0] if self.pending_samples else None)
        elif self.current_vertex_id is not None:
            anchor_id = self.current_vertex_id
        else:
            raise RuntimeError("retrace exit has no graph anchor")
        anchor_sample = self.vertices[anchor_id].sample
        buffered = list(self.pending_samples)
        self.current_vertex_id = anchor_id
        self.current_retrace_edge_id = None
        self.discovery_cursor = anchor_sample
        self.discovery_distance = 0.0
        self.state = self.DISCOVERING
        self._event("retrace_exit", {
            "anchor_vertex_id": anchor_id,
            "stamp": buffered[-1].stamp if buffered else anchor_sample.stamp,
            "tracking_kind": tracking_kind,
        })
        self.pending_samples = []
        self.pending_distance = 0.0
        self.exit_anchor_vertex_id = None
        self.exit_anchor_edge_id = None
        self.tracking_kind = "retrace"
        for buffered_sample in buffered:
            self._append_discovery(buffered_sample)

    def _handle_retrace_pending(self, sample: PoseSample, motion: Point3, step: float) -> None:
        if self.pending_edge_id not in self.edges:
            self._cancel_retrace_pending()
            self._append_discovery(sample)
            return
        kind = self.pending_kind
        projection = self._project(self.pending_edge_id, sample, motion)
        if self._matches_tracking_mode(projection, tracking_kind=kind) and not \
                -1.0e-6 <= projection.raw_ratio <= 1.0 + 1.0e-6:
            edge = self.edges[projection.edge_id]
            endpoint_id = edge.first if projection.raw_ratio < 0.0 else edge.second
            continuation = self._incident_best_projection(
                endpoint_id, sample, motion, require_inside=True,
                tracking_kind=kind)
            if continuation is not None:
                if kind == "retrace":
                    self.current_vertex_id = endpoint_id
                self.pending_edge_id = continuation.edge_id
                projection = continuation
        projection_valid = (
            self._matches_tracking_mode(projection, tracking_kind=kind) and
            -1.0e-6 <= projection.raw_ratio <= 1.0 + 1.0e-6)
        if not projection_valid and kind == "loop":
            edge = self.edges[projection.edge_id]
            endpoint_distances = (
                distance_3d(sample.point, self.vertices[edge.first].sample.point),
                distance_3d(sample.point, self.vertices[edge.second].sample.point),
            )
            loop = self.config.loop_closure
            if min(endpoint_distances) <= loop.corridor_exit_xy_tolerance and \
                    self.pending_gap_distance + step <= \
                    loop.exit_confirmation_distance + 1.0e-9:
                self.pending_samples.append(sample)
                self.pending_gap_distance += step
                return
        if not projection_valid:
            self._cancel_retrace_pending()
            self._append_discovery(sample)
            return
        self.pending_samples.append(sample)
        self.pending_distance += step
        self.pending_gap_distance = 0.0
        confirmation = (
            self.config.loop_closure.confirmation_distance
            if kind == "loop" else self.config.retrace.entry_confirmation_distance)
        displacement = (
            distance_3d(self.pending_samples[0].point, self.pending_samples[-1].point)
            if len(self.pending_samples) >= 2 else 0.0)
        displacement_ok = (
            kind != "loop" or displacement + 1.0e-9 >= 0.70 * confirmation)
        if self.pending_distance + 1.0e-9 >= confirmation and displacement_ok:
            self._enter_retrace()

    def _handle_retracing(self, sample: PoseSample, motion: Point3, step: float) -> None:
        kind = self.tracking_kind
        projection: Optional[Projection] = None
        if self.current_retrace_edge_id in self.edges:
            candidate = self._project(self.current_retrace_edge_id, sample, motion)
            if self._matches_tracking_mode(
                candidate, exit_mode=True, tracking_kind=kind) and \
                    -1.0e-6 <= candidate.raw_ratio <= 1.0 + 1.0e-6:
                projection = candidate
            elif self._matches_tracking_mode(
                candidate, exit_mode=True, tracking_kind=kind):
                # We passed an endpoint of the current edge.  Continue over an
                # adjacent old edge when one matches; only start an exit when
                # there is no connected continuation.
                edge = self.edges[candidate.edge_id]
                endpoint_id = edge.first if candidate.raw_ratio < 0.0 else edge.second
                self.current_vertex_id = endpoint_id
                projection = self._incident_best_projection(
                    endpoint_id, sample, motion, require_inside=True,
                    tracking_kind=kind, exit_mode=True)
        if projection is None and self.current_vertex_id is not None:
            projection = self._incident_best_projection(
                self.current_vertex_id, sample, motion, require_inside=True,
                tracking_kind=kind, exit_mode=True)

        if projection is None:
            anchor = None
            if self.current_retrace_edge_id in self.edges:
                edge = self.edges[self.current_retrace_edge_id]
                first, second = self._edge_samples(self.current_retrace_edge_id)
                anchor_point = tuple(
                    first.point[i] + self.current_retrace_ratio *
                    (second.point[i] - first.point[i]) for i in range(3)
                )
                anchor = Projection(
                    self.current_retrace_edge_id, self.current_retrace_ratio,
                    self.current_retrace_ratio, anchor_point, 0.0, 0.0, 0.0)
            self._begin_exit(anchor, sample)
            return

        edge = self.edges[projection.edge_id]
        # Passing beyond an endpoint starts a new branch immediately; the buffered
        # poses retain the distance inside the exit hysteresis band.
        if projection.raw_ratio < -1.0e-6:
            self.current_vertex_id = edge.first
            self._begin_exit(Projection(
                projection.edge_id, 0.0, projection.raw_ratio,
                self.vertices[edge.first].sample.point, projection.xy_distance,
                projection.z_distance, projection.heading_error), sample)
            return
        if projection.raw_ratio > 1.0 + 1.0e-6:
            self.current_vertex_id = edge.second
            self._begin_exit(Projection(
                projection.edge_id, 1.0, projection.raw_ratio,
                self.vertices[edge.second].sample.point, projection.xy_distance,
                projection.z_distance, projection.heading_error), sample)
            return

        self.current_retrace_edge_id = projection.edge_id
        self.current_retrace_ratio = projection.ratio
        snapped = self._snap_projection_vertex(projection)
        self.current_vertex_id = snapped
        self.suppressed_distance += step

    def _handle_exit_pending(self, sample: PoseSample, motion: Point3, step: float) -> None:
        # Only cancel an exit when the robot returns to the same adjacent graph
        # location; never search arbitrary old edges here.
        projection = None
        if self.exit_anchor_edge_id in self.edges:
            candidate = self._project(self.exit_anchor_edge_id, sample, motion)
            if self._matches_tracking_mode(
                candidate, tracking_kind=self.tracking_kind,
            ) and 0.0 <= candidate.raw_ratio <= 1.0:
                projection = candidate
        elif self.exit_anchor_vertex_id is not None:
            projection = self._incident_best_projection(
                self.exit_anchor_vertex_id, sample, motion, require_inside=True,
                tracking_kind=self.tracking_kind)
        if projection is not None:
            self.suppressed_distance += self.pending_distance + step
            self.pending_samples = []
            self.pending_distance = 0.0
            self.exit_anchor_vertex_id = None
            self.exit_anchor_edge_id = None
            self.state = self.RETRACING
            self.current_retrace_edge_id = projection.edge_id
            self.current_vertex_id = self._snap_projection_vertex(
                projection,
                threshold=(self.config.loop_closure.vertex_snap_distance
                           if self.tracking_kind == "loop" else None))
            return
        self.pending_samples.append(sample)
        self.pending_distance += step
        exit_confirmation = (
            self.config.loop_closure.exit_confirmation_distance
            if self.tracking_kind == "loop"
            else self.config.retrace.exit_confirmation_distance)
        if self.pending_distance + 1.0e-9 >= exit_confirmation:
            self._confirm_exit()

    def add_pose(self, sample: PoseSample) -> None:
        if self.finalized:
            raise RuntimeError("cannot append poses after finalize")
        sample = sample.normalized()
        self.raw_samples.append(sample)
        if self.last_sample is None:
            self.current_vertex_id = self._add_vertex(sample)
            self.discovery_cursor = sample
            self.last_sample = sample
            self.last_input_sample = sample
            return

        # Relocation is a discontinuity between consecutive input messages,
        # independent of the spatial de-jittering used by topology geometry.
        raw_step = distance_3d(
            self.last_input_sample.point, sample.point
        ) if self.last_input_sample is not None else 0.0
        self.last_input_sample = sample
        if raw_step > self.config.relocation_distance:
            self._handle_relocation(sample, raw_step)
            self.last_sample = sample
            self.last_motion = None
            return

        step = distance_3d(self.last_sample.point, sample.point)
        # Do not turn localization jitter into travelled route length.  Keep the
        # last accepted geometry pose fixed so genuine slow motion accumulates
        # until it reaches dedup_distance.
        if step < self.config.dedup_distance:
            return
        motion = self._motion(self.last_sample, sample)
        self.global_search_cooldown = max(
            0.0, self.global_search_cooldown - step)

        if self.state == self.RETRACE_PENDING:
            self._handle_retrace_pending(sample, motion, step)
        elif self.state == self.RETRACING:
            self._handle_retracing(sample, motion, step)
        elif self.state == self.EXIT_PENDING:
            self._handle_exit_pending(sample, motion, step)
        else:
            pending_started = False
            if self.config.retrace.enabled:
                self._detect_reversal(sample, motion)
                candidate = self._incident_best_projection(
                    self.current_vertex_id, sample, motion, require_inside=True
                ) if self.current_vertex_id is not None else None
                if candidate is not None:
                    edge = self.edges[candidate.edge_id]
                    at_first = self.current_vertex_id == edge.first
                    entering = (
                        candidate.raw_ratio > 1.0e-6 if at_first
                        else candidate.raw_ratio < 1.0 - 1.0e-6)
                    if entering:
                        self._start_retrace_pending(candidate, sample, step)
                        pending_started = True
            if not pending_started:
                loop_candidate = self._global_best_projection(sample, motion)
                if loop_candidate is not None:
                    self._start_loop_pending(loop_candidate, sample, step)
                else:
                    self._append_discovery(sample)

        self.last_motion = motion
        self.last_sample = sample

    def _handle_relocation(self, sample: PoseSample, distance: float) -> None:
        if self.state == self.EXIT_PENDING and self.pending_samples:
            self._confirm_exit()
        self.component += 1
        self.state = self.DISCOVERING
        self._clear_pending_metadata()
        self.current_retrace_edge_id = None
        self.tracking_kind = "retrace"
        self.global_search_cooldown = 0.0
        self.exit_anchor_edge_id = None
        self.exit_anchor_vertex_id = None
        self.current_vertex_id = self._add_vertex(sample)
        self.discovery_cursor = sample
        self.discovery_distance = 0.0
        self.jump_points.append(sample.point)
        self._event("pose_jump", {
            "stamp": sample.stamp, "distance_m": distance,
            "threshold_m": self.config.relocation_distance,
            "component": self.component,
        })

    def mark_corner(
        self, sample: PoseSample, turn_degrees: float,
        merge_distance: float = 0.35,
    ) -> int:
        """Mark or insert the nearest graph location as a corner."""
        sample = sample.normalized()
        nearest_vertex = min(
            self.vertices.values(),
            key=lambda vertex: distance_3d(vertex.sample.point, sample.point),
            default=None)
        if nearest_vertex is not None and distance_3d(
            nearest_vertex.sample.point, sample.point,
        ) <= merge_distance:
            vertex_id = nearest_vertex.vertex_id
        else:
            motion = self.last_motion or (1.0, 0.0, 0.0)
            projections = [
                self._project(edge_id, sample, motion) for edge_id in self.edges]
            valid = [item for item in projections if item.xy_distance <= merge_distance and
                     item.z_distance <= self.config.retrace.z_tolerance]
            if not valid:
                vertex_id = (
                    nearest_vertex.vertex_id if nearest_vertex is not None
                    else self._add_vertex(sample))
            else:
                projection = min(valid, key=lambda item: (item.xy_distance, item.edge_id))
                snapped = self._snap_projection_vertex(projection)
                vertex_id = snapped if snapped is not None else self._split_edge(
                    projection.edge_id, projection.ratio, sample)
        vertex = self.vertices[vertex_id]
        if turn_degrees > vertex.turn_degrees:
            vertex.is_corner = True
            vertex.turn_degrees = turn_degrees
            vertex.sample = PoseSample(
                sample.stamp, vertex.sample.point, sample.quaternion,
                vertex.sample.frame_index)
            self._touch()
        return vertex_id

    def finalize(self) -> None:
        if self.finalized:
            return
        if self.state == self.RETRACE_PENDING:
            self._cancel_retrace_pending()
        elif self.state == self.EXIT_PENDING:
            if self.pending_distance >= self.config.dedup_distance:
                self._confirm_exit()
            else:
                self.suppressed_distance += self.pending_distance
        if self.state == self.DISCOVERING and self.discovery_cursor is not None and \
                self.discovery_distance >= self.config.dedup_distance:
            self._commit_discovery_endpoint(self.discovery_cursor)
        self.finalized = True
        self._touch()

    @property
    def active_retrace_edge_id(self) -> Optional[int]:
        if self.state in (self.RETRACING, self.RETRACE_PENDING):
            return self.current_retrace_edge_id or self.pending_edge_id
        return None

    @property
    def active_tracking_kind(self) -> Optional[str]:
        if self.state == self.RETRACE_PENDING:
            return self.pending_kind
        if self.state in (self.RETRACING, self.EXIT_PENDING):
            return self.tracking_kind
        return None

    def topology_dict(self, frame_id: str, status: Optional[str] = None) -> dict:
        vertices = {}
        for vertex_id in sorted(self.vertices):
            vertex = self.vertices[vertex_id]
            vertices[str(vertex_id)] = {
                "pos": [
                    vertex.sample.point[0], vertex.sample.point[1],
                    vertex.sample.point[2] - self.config.body_height,
                ],
                "rpy": quaternion_to_rpy(vertex.sample.quaternion),
                "meta": {
                    "type": 0,
                    "typeId": 0,
                    "isCorner": vertex.is_corner,
                    "isSlope": vertex.is_slope,
                    "isJunction": vertex.is_junction,
                    "state": "confirmed",
                    "source": "incremental_topology",
                    "sourceStamp": vertex.sample.stamp,
                    "component": vertex.component,
                    "turnDeg": vertex.turn_degrees,
                },
                "pcd": self.config.topo_pcd,
                "acc": self.config.topo_acc,
                "turnable": True,
            }
        edges = {}
        for edge_id in sorted(self.edges):
            edge = self.edges[edge_id]
            edges[str(edge_id)] = {
                "v": [edge.first, edge.second],
                "weight": distance_3d(
                    self.vertices[edge.first].sample.point,
                    self.vertices[edge.second].sample.point),
                "meta": {"dir": 0, "source": edge.source},
            }
        return apply_topology_schema({
            "version": self.version,
            "type": self.config.topo_type,
            "frame_id": frame_id,
            "status": status or ("complete" if self.finalized else "recording"),
            "topology_mode": (
                "incremental_graph" if self.loop_closure_count
                else "incremental_forest"),
            "vertices": vertices,
            "edges": edges,
            "generation": {
                "retrace_count": self.retrace_count,
                "route_reentry_count": self.route_reentry_count,
                "loop_closure_count": self.loop_closure_count,
                "loop_validation_available_count":
                    self.loop_validation_available_count,
                "loop_validation_rejection_count":
                    self.loop_validation_rejection_count,
                "suppressed_distance_m": self.suppressed_distance,
                "edge_split_count": self.edge_split_count,
                "component_count": len({v.component for v in self.vertices.values()}),
            },
        })


def _maximal_forest_chains(vertices: Iterable[int], edges: Dict[int, Edge]) -> List[List[int]]:
    adjacency: Dict[int, List[Tuple[int, int]]] = {vertex_id: [] for vertex_id in vertices}
    for edge_id, edge in edges.items():
        adjacency[edge.first].append((edge.second, edge_id))
        adjacency[edge.second].append((edge.first, edge_id))
    used: set[int] = set()
    chains: List[List[int]] = []
    starts = sorted(vertex_id for vertex_id, neighbors in adjacency.items() if len(neighbors) != 2)
    for start in starts:
        for neighbor, edge_id in sorted(adjacency[start]):
            if edge_id in used:
                continue
            chain = [start]
            previous = start
            current = neighbor
            used.add(edge_id)
            while True:
                chain.append(current)
                choices = [
                    item for item in adjacency[current]
                    if item[0] != previous and item[1] not in used]
                if len(adjacency[current]) != 2 or not choices:
                    break
                next_vertex, next_edge = min(choices, key=lambda item: item[1])
                used.add(next_edge)
                previous, current = current, next_vertex
            chains.append(chain)
    # Phase two permits cycles. Traverse every remaining degree-two component as
    # one closed chain instead of treating its individual edges as unrelated.
    for edge_id, edge in sorted(edges.items()):
        if edge_id in used:
            continue
        start = min(edge.first, edge.second)
        first_choices = sorted(
            item for item in adjacency[start] if item[1] not in used)
        if not first_choices:
            continue
        current, current_edge = first_choices[0]
        previous = start
        chain = [start]
        used.add(current_edge)
        while True:
            chain.append(current)
            if current == start:
                break
            choices = sorted(
                item for item in adjacency[current]
                if item[0] != previous and item[1] not in used)
            if not choices:
                break
            next_vertex, next_edge = choices[0]
            used.add(next_edge)
            previous, current = current, next_vertex
        chains.append(chain)
    return chains


def annotate_graph_slopes(builder: IncrementalTopologyBuilder, config: SlopeConfig) -> dict:
    """Annotate each maximal non-branching graph chain and OR flags at junctions."""
    for vertex in builder.vertices.values():
        vertex.is_slope = False
    segment_records = []
    for chain_index, chain in enumerate(_maximal_forest_chains(builder.vertices, builder.edges), 1):
        if len(chain) < 2:
            continue
        document = {"vertices": {
            str(index): {
                "pos": list(builder.vertices[vertex_id].sample.point), "meta": {}
            }
            for index, vertex_id in enumerate(chain, 1)
        }}
        try:
            annotated = annotate_document(document, config)
        except ValueError:
            continue
        for index, vertex_id in enumerate(chain, 1):
            builder.vertices[vertex_id].is_slope |= bool(
                annotated["vertices"][str(index)]["meta"]["isSlope"])
        for segment in annotated.get("slopeAnnotation", {}).get("segments", []):
            segment_records.append({"chain": chain_index, **segment})
    return {
        "method": "maximal_graph_chains/local_linear_z_over_xy_distance",
        "counts": {
            "normal": sum(not vertex.is_slope for vertex in builder.vertices.values()),
            "slope": sum(vertex.is_slope for vertex in builder.vertices.values()),
        },
        "segments": segment_records,
    }


def load_pose_rows(path: Path) -> List[PoseSample]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    rows = payload.get("poses", []) if isinstance(payload, dict) else payload
    if not isinstance(rows, list):
        raise ValueError("pose JSON must be an array or contain a poses array")
    result = []
    for frame_index, row in enumerate(rows):
        if not isinstance(row, list) or len(row) < 8:
            raise ValueError("each pose row must contain timestamp, xyz and quaternion")
        result.append(PoseSample(
            float(row[0]), (float(row[1]), float(row[2]), float(row[3])),
            (float(row[4]), float(row[5]), float(row[6]), float(row[7])),
            frame_index,
        ).normalized())
    result.sort(key=lambda sample: sample.stamp)
    return result
