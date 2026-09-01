"""Pure file, interpolation, and route-progress logic."""

from dataclasses import dataclass
from enum import Enum
import json
import math
import os
from pathlib import Path
import re
from typing import Iterable, List, Optional, Sequence, Tuple


Point3 = Tuple[float, float, float]
_FRAME_PATTERN = re.compile(r"^#\s*frame_id\s*:\s*(\S+)\s*$")


class TargetType(Enum):
    NORMAL = "NORMAL"
    CORNER = "CORNER"


@dataclass(frozen=True)
class TargetFile:
    frame_id: str
    points: List[Point3]
    types: List[TargetType]
    is_slope: List[bool]


def distance_3d(first: Point3, second: Point3) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(first, second)))


def format_float(value: float) -> str:
    text = f"{float(value):.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("", "-0") else text


def parse_target_text(text: str, fallback_frame: str = "camera_init") -> TargetFile:
    frame_id = fallback_frame
    points: List[Point3] = []
    types: List[TargetType] = []
    numbered_rows: Optional[bool] = None
    for line_number, raw_line in enumerate(text.splitlines(), 1):
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            match = _FRAME_PATTERN.match(line)
            if match:
                frame_id = match.group(1)
            continue
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) not in (3, 4, 5):
            raise ValueError(
                f"line {line_number}: expected 'id x y z_ground [NORMAL|CORNER]' "
                "or legacy 'x y z_ground'")
        row_is_numbered = len(fields) in (4, 5)
        if numbered_rows is not None and row_is_numbered != numbered_rows:
            raise ValueError(f"line {line_number}: numbered and legacy rows must not be mixed")
        numbered_rows = row_is_numbered
        if row_is_numbered:
            try:
                target_id = int(fields[0])
            except ValueError as error:
                raise ValueError(f"line {line_number}: target id is not an integer") from error
            expected_id = len(points) + 1
            if target_id != expected_id:
                raise ValueError(
                    f"line {line_number}: expected target id {expected_id}, got {target_id}")
            fields = fields[1:]
        # Untyped rows were produced by the manual recorder.  Preserve their
        # intended strict-arrival semantics when loading legacy files.
        target_type = TargetType.CORNER
        if len(fields) == 4:
            try:
                target_type = TargetType(fields[-1].upper())
            except ValueError as error:
                raise ValueError(
                    f"line {line_number}: target type must be NORMAL or CORNER") from error
            fields = fields[:-1]
        try:
            point = tuple(float(field) for field in fields)
        except ValueError as error:
            raise ValueError(f"line {line_number}: coordinate is not a number") from error
        if not all(math.isfinite(value) for value in point):
            raise ValueError(f"line {line_number}: coordinates must be finite")
        points.append(point)  # type: ignore[arg-type]
        types.append(target_type)
    if not frame_id:
        raise ValueError("frame_id must not be empty")
    return TargetFile(
        frame_id=frame_id, points=points, types=types,
        is_slope=[False] * len(points))


def _json_integer(value, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field} must be an integer")
    return value


def parse_topo_single_json(text: str, fallback_frame: str = "camera_init") -> TargetFile:
    """Parse the product topoSingle JSON used by the linear route sequencer.

    The current sequencer follows one non-branching route in either direction. It
    therefore accepts only contiguous 1..N vertices connected by undirected
    (1,2), (2,3), ... edges, avoiding silent execution of a branched graph.
    """
    try:
        document = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError(
            f"invalid topoSingle JSON at line {error.lineno}, column {error.colno}: "
            f"{error.msg}") from error
    if not isinstance(document, dict):
        raise ValueError("topoSingle root must be a JSON object")

    version = _json_integer(document.get("version", 0), "version")
    if version < 0:
        raise ValueError("version must be non-negative")
    graph_type = _json_integer(document.get("type", 0), "type")
    if graph_type not in (0, 1):
        raise ValueError("type must be 0 (normal) or 1 (corridor)")

    frame_id = document.get("frame_id", fallback_frame)
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a non-empty string")
    frame_id = frame_id.strip()

    vertices = document.get("vertices")
    if not isinstance(vertices, dict):
        raise ValueError("vertices must be a JSON object")
    if len(vertices) < 2:
        raise ValueError("topoSingle route requires at least two vertices")

    parsed_vertices = {}
    for raw_id, vertex in vertices.items():
        try:
            vertex_id = int(raw_id)
        except (TypeError, ValueError) as error:
            raise ValueError(f"vertex id {raw_id!r} must be a positive integer") from error
        if raw_id != str(vertex_id) or vertex_id < 1:
            raise ValueError(f"vertex id {raw_id!r} must use canonical positive integer text")
        if vertex_id in parsed_vertices:
            raise ValueError(f"duplicate vertex id {vertex_id}")
        if not isinstance(vertex, dict):
            raise ValueError(f"vertex {vertex_id} must be a JSON object")
        position = vertex.get("pos")
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError(f"vertex {vertex_id}.pos must be an [x, y, z] array")
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in position):
            raise ValueError(f"vertex {vertex_id}.pos coordinates must be numbers")
        point = tuple(float(value) for value in position)
        if not all(math.isfinite(value) for value in point):
            raise ValueError(f"vertex {vertex_id}.pos coordinates must be finite")
        metadata = vertex.get("meta", {})
        if not isinstance(metadata, dict):
            raise ValueError(f"vertex {vertex_id}.meta must be a JSON object")
        is_corner = metadata.get("isCorner", False)
        if not isinstance(is_corner, bool):
            raise ValueError(f"vertex {vertex_id}.meta.isCorner must be boolean")
        is_slope = metadata.get("isSlope", False)
        if not isinstance(is_slope, bool):
            raise ValueError(f"vertex {vertex_id}.meta.isSlope must be boolean")
        parsed_vertices[vertex_id] = (
            point, TargetType.CORNER if is_corner else TargetType.NORMAL,
            is_slope)

    vertex_count = len(parsed_vertices)
    expected_ids = set(range(1, vertex_count + 1))
    if set(parsed_vertices) != expected_ids:
        raise ValueError(
            f"linear topoSingle vertex ids must be contiguous 1..{vertex_count}")

    edges = document.get("edges")
    if not isinstance(edges, dict):
        raise ValueError("edges must be a JSON object")
    if len(edges) != vertex_count - 1:
        raise ValueError(
            f"linear topoSingle requires {vertex_count - 1} edges for "
            f"{vertex_count} vertices, got {len(edges)}")

    actual_pairs = set()
    for raw_edge_id, edge in edges.items():
        if not isinstance(edge, dict):
            raise ValueError(f"edge {raw_edge_id!r} must be a JSON object")
        endpoints = edge.get("v")
        if not isinstance(endpoints, list) or len(endpoints) != 2:
            raise ValueError(f"edge {raw_edge_id!r}.v must contain two vertex ids")
        left = _json_integer(endpoints[0], f"edge {raw_edge_id!r}.v[0]")
        right = _json_integer(endpoints[1], f"edge {raw_edge_id!r}.v[1]")
        if left not in parsed_vertices or right not in parsed_vertices:
            raise ValueError(f"edge {raw_edge_id!r} references an unknown vertex")
        if left == right:
            raise ValueError(f"edge {raw_edge_id!r} must connect distinct vertices")
        metadata = edge.get("meta", {})
        if not isinstance(metadata, dict):
            raise ValueError(f"edge {raw_edge_id!r}.meta must be a JSON object")
        direction = _json_integer(
            metadata.get("dir", 0), f"edge {raw_edge_id!r}.meta.dir")
        if direction != 0:
            raise ValueError(
                f"edge {raw_edge_id!r} is directed; the current sequencer requires "
                "undirected edges for forward and reverse navigation")
        pair = tuple(sorted((left, right)))
        if pair in actual_pairs:
            raise ValueError(f"duplicate edge between vertices {pair[0]} and {pair[1]}")
        actual_pairs.add(pair)

    expected_pairs = {(vertex_id, vertex_id + 1) for vertex_id in range(1, vertex_count)}
    if actual_pairs != expected_pairs:
        raise ValueError(
            "current route sequencer requires the single chain "
            "1-2-...-N without branches or skipped ids")

    ordered = [parsed_vertices[vertex_id] for vertex_id in range(1, vertex_count + 1)]
    return TargetFile(
        frame_id=frame_id,
        points=[entry[0] for entry in ordered],
        types=[entry[1] for entry in ordered],
        is_slope=[entry[2] for entry in ordered],
    )


def load_route_file(path: str, fallback_frame: str = "camera_init") -> TargetFile:
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    if Path(path).suffix.lower() == ".json":
        return parse_topo_single_json(text, fallback_frame)
    return parse_target_text(text, fallback_frame)


def load_target_file(path: str, fallback_frame: str = "camera_init") -> TargetFile:
    """Backward-compatible alias for callers that still use the old name."""
    return load_route_file(path, fallback_frame)


def build_target_text(
    frame_id: str, points: Iterable[Point3],
    types: Optional[Iterable[TargetType]] = None,
) -> str:
    if not frame_id:
        raise ValueError("frame_id must not be empty")
    lines = [
        f"# frame_id: {frame_id}",
        "# z_semantics: ground",
        "# columns: id x y z_ground type",
    ]
    point_list = list(points)
    type_list = list(types) if types is not None else [TargetType.CORNER] * len(point_list)
    if len(type_list) != len(point_list):
        raise ValueError("target type count must match target point count")
    for target_id, (point, target_type) in enumerate(zip(point_list, type_list), 1):
        if len(point) != 3 or not all(math.isfinite(value) for value in point):
            raise ValueError("target coordinates must be finite xyz triples")
        if not isinstance(target_type, TargetType):
            raise ValueError("target type must be TargetType.NORMAL or TargetType.CORNER")
        lines.append(
            f"{target_id} " + " ".join(format_float(value) for value in point) +
            f" {target_type.value}")
    return "\n".join(lines) + "\n"


def atomic_write_target_file(
    path: str, frame_id: str, points: Iterable[Point3],
    types: Optional[Iterable[TargetType]] = None,
) -> None:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(build_target_text(frame_id, points, types))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, destination)


def validate_targets(points: Sequence[Point3], minimum_count: int = 2) -> None:
    if len(points) < minimum_count:
        raise ValueError(f"route requires at least {minimum_count} targets")
    for index, point in enumerate(points):
        if len(point) != 3 or not all(math.isfinite(value) for value in point):
            raise ValueError(f"target {index} is not a finite xyz point")
        if index > 0 and distance_3d(points[index - 1], point) <= 1e-9:
            raise ValueError(f"targets {index - 1} and {index} are duplicates")


def interpolate_targets(points: Sequence[Point3], spacing: float) -> List[Point3]:
    validate_targets(points)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("route spacing must be finite and positive")
    result = [points[0]]
    for start, end in zip(points, points[1:]):
        length = distance_3d(start, end)
        intervals = max(1, int(math.ceil(length / spacing)))
        for step in range(1, intervals + 1):
            ratio = step / intervals
            result.append(tuple(
                start[axis] + ratio * (end[axis] - start[axis]) for axis in range(3)
            ))
    return result


def split_interpolated_path(
    targets: Sequence[Point3], path: Sequence[Point3], tolerance: float = 1e-7
) -> List[List[Point3]]:
    validate_targets(targets)
    if len(path) < len(targets):
        raise ValueError("interpolated path has fewer points than targets")
    target_indices: List[int] = []
    search_start = 0
    for target in targets:
        match: Optional[int] = None
        for index in range(search_start, len(path)):
            if distance_3d(target, path[index]) <= tolerance:
                match = index
                break
        if match is None:
            raise ValueError("interpolated path does not contain every target in order")
        target_indices.append(match)
        search_start = match + 1
    segments: List[List[Point3]] = []
    for start, end in zip(target_indices, target_indices[1:]):
        segment = list(path[start:end + 1])
        if len(segment) < 2:
            raise ValueError("interpolated segment must contain at least two points")
        segments.append(segment)
    return segments


class RouteState(Enum):
    WAITING_GOAL = "waiting_goal"
    TRACKING = "tracking"
    COMPLETE = "complete"


@dataclass(frozen=True)
class ProgressUpdate:
    state: RouteState
    distance: float
    changed: bool
    event: str
    active_leg: Optional[Tuple[int, int]]
    required_target: int
    target_type: TargetType = TargetType.NORMAL
    arrival_tolerance: float = 0.15


def nearest_target(body_position: Point3, body_targets: Sequence[Point3]) -> Tuple[int, float]:
    """Return the zero-based id and 3-D distance of the nearest recorded target."""
    if not body_targets:
        raise ValueError("at least one target is required")
    distances = [distance_3d(body_position, target) for target in body_targets]
    nearest_index = min(range(len(distances)), key=distances.__getitem__)
    return nearest_index, distances[nearest_index]


def ordered_target_indices(start_index: int, goal_index: int, target_count: int) -> List[int]:
    """Build the unique forward or reverse sequence on a non-looping route."""
    if target_count < 1:
        raise ValueError("target count must be positive")
    if not 0 <= start_index < target_count or not 0 <= goal_index < target_count:
        raise ValueError("start and goal target indices must be in range")
    step = 1 if goal_index >= start_index else -1
    return list(range(start_index, goal_index + step, step))


class RouteProgress:
    """Advance at most one target in a selected forward/reverse sequence per update."""

    def __init__(
        self, target_indices: Sequence[int], arrival_tolerance: float,
        target_types: Optional[Sequence[TargetType]] = None,
        corner_arrival_tolerance: Optional[float] = None,
        goal_arrival_tolerance: Optional[float] = None,
    ):
        if not target_indices:
            raise ValueError("selected route requires at least one target")
        if not math.isfinite(arrival_tolerance) or arrival_tolerance <= 0.0:
            raise ValueError("arrival tolerance must be finite and positive")
        if any(index < 0 for index in target_indices):
            raise ValueError("target indices must be non-negative")
        if any(abs(right - left) != 1 for left, right in zip(target_indices, target_indices[1:])):
            raise ValueError("selected route targets must be adjacent and ordered")
        self.target_indices = list(target_indices)
        self.target_types = list(target_types) if target_types is not None else []
        if self.target_types and max(self.target_indices) >= len(self.target_types):
            raise ValueError("target type list does not cover selected route")
        self.normal_arrival_tolerance = arrival_tolerance
        self.corner_arrival_tolerance = (
            arrival_tolerance if corner_arrival_tolerance is None
            else corner_arrival_tolerance)
        self.goal_arrival_tolerance = (
            arrival_tolerance if goal_arrival_tolerance is None
            else goal_arrival_tolerance)
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (
                self.corner_arrival_tolerance, self.goal_arrival_tolerance)
        ):
            raise ValueError("arrival tolerances must be finite and positive")
        self.state = RouteState.TRACKING
        self.sequence_position = 0
        self.active_leg: Optional[Tuple[int, int]] = None
        self.required_target = self.target_indices[0]

    def required_target_type(self) -> TargetType:
        if not self.target_types:
            return TargetType.NORMAL
        return self.target_types[self.required_target]

    def required_target_is_goal(self) -> bool:
        return self.sequence_position == len(self.target_indices) - 1

    def effective_tolerance(self) -> float:
        if self.required_target_is_goal():
            return self.goal_arrival_tolerance
        if self.required_target_type() == TargetType.CORNER:
            return self.corner_arrival_tolerance
        return self.normal_arrival_tolerance

    def update(self, body_position: Point3, body_targets: Sequence[Point3]) -> ProgressUpdate:
        if self.required_target >= len(body_targets):
            raise ValueError("selected target is outside the current route")
        distance = distance_3d(body_position, body_targets[self.required_target])
        changed = False
        event = "none"
        if self.state != RouteState.COMPLETE and distance <= self.effective_tolerance():
            changed = True
            if self.sequence_position == len(self.target_indices) - 1:
                self.state = RouteState.COMPLETE
                event = "goal_reached"
            else:
                previous = self.required_target
                self.sequence_position += 1
                self.required_target = self.target_indices[self.sequence_position]
                self.active_leg = (previous, self.required_target)
                event = "target_advanced"
        return ProgressUpdate(
            state=self.state,
            distance=distance,
            changed=changed,
            event=event,
            active_leg=self.active_leg,
            required_target=self.required_target,
            target_type=self.required_target_type(),
            arrival_tolerance=self.effective_tolerance(),
        )
