"""Pure route-slope analysis, independent of ROS."""

import copy
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SlopeConfig:
    """Parameters for robust local-Z slope detection."""

    fit_radius: float = 2.0
    grade_threshold: float = 0.12
    minimum_core_length: float = 2.0
    minimum_height_change: float = 0.20
    maximum_core_gap: float = 1.0
    buffer_distance: float = 2.0

    def validate(self):
        values = (
            self.fit_radius, self.grade_threshold,
            self.minimum_core_length, self.minimum_height_change,
            self.maximum_core_gap, self.buffer_distance)
        if not all(math.isfinite(value) and value >= 0.0
                   for value in values):
            raise ValueError("slope annotation parameters must be finite and non-negative")
        if self.fit_radius <= 0.0:
            raise ValueError("fit_radius must be positive")


@dataclass(frozen=True)
class SlopeSegment:
    """Inclusive core slope segment in ordered-vertex indices."""

    start: int
    end: int
    direction: str
    mean_grade: float
    length: float
    height_change: float


def ordered_vertices(document):
    """Return contiguous 1..N vertices and validated XYZ positions."""
    vertices = document.get("vertices")
    if not isinstance(vertices, dict) or len(vertices) < 2:
        raise ValueError("topology JSON must contain at least two vertices")
    expected = list(range(1, len(vertices) + 1))
    try:
        ids = sorted(int(raw_id) for raw_id in vertices)
    except (TypeError, ValueError) as error:
        raise ValueError("vertex IDs must be integer strings") from error
    if ids != expected:
        raise ValueError("vertex IDs must be contiguous from 1 to N")
    ordered = []
    points = []
    for vertex_id in expected:
        vertex = vertices[str(vertex_id)]
        if not isinstance(vertex, dict):
            raise ValueError(f"vertex {vertex_id} must be an object")
        position = vertex.get("pos")
        if (not isinstance(position, list) or len(position) != 3 or
                isinstance(position[0], bool) or
                not all(isinstance(value, (int, float)) and
                        math.isfinite(float(value)) for value in position)):
            raise ValueError(f"vertex {vertex_id}.pos must be finite XYZ")
        ordered.append(vertex)
        points.append(tuple(float(value) for value in position))
    return ordered, points


def cumulative_xy_distance(points):
    distances = [0.0]
    for first, second in zip(points, points[1:]):
        step = math.hypot(second[0] - first[0], second[1] - first[1])
        distances.append(distances[-1] + step)
    if distances[-1] <= 1e-9:
        raise ValueError("route has zero horizontal length")
    return distances


def local_linear_grades(points, distances, fit_radius):
    """Fit z=a*s+b around each point using horizontal route distance."""
    grades = []
    for centre in distances:
        indices = [index for index, distance in enumerate(distances)
                   if abs(distance - centre) <= fit_radius + 1e-9]
        if len(indices) < 2:
            grades.append(0.0)
            continue
        mean_s = sum(distances[index] for index in indices) / len(indices)
        mean_z = sum(points[index][2] for index in indices) / len(indices)
        denominator = sum((distances[index] - mean_s) ** 2
                          for index in indices)
        if denominator <= 1e-12:
            grades.append(0.0)
            continue
        numerator = sum(
            (distances[index] - mean_s) * (points[index][2] - mean_z)
            for index in indices)
        grades.append(numerator / denominator)
    return grades


def _candidate_ranges(grades, threshold):
    ranges = []
    start = None
    sign = 0
    for index, grade in enumerate(grades + [0.0]):
        current_sign = 1 if grade >= threshold else (-1 if grade <= -threshold else 0)
        if current_sign != 0 and start is None:
            start = index
            sign = current_sign
        elif start is not None and current_sign != sign:
            ranges.append([start, index - 1, sign])
            start = index if current_sign != 0 else None
            sign = current_sign
    return ranges


def _merge_ranges(ranges, distances, maximum_gap):
    merged = []
    for current in ranges:
        if (merged and merged[-1][2] == current[2] and
                distances[current[0]] - distances[merged[-1][1]] <=
                maximum_gap + 1e-9):
            merged[-1][1] = current[1]
        else:
            merged.append(list(current))
    return merged


def detect_slope_segments(points, config=SlopeConfig()):
    """Detect sustained signed Z trends and reject short/noisy candidates."""
    config.validate()
    distances = cumulative_xy_distance(points)
    grades = local_linear_grades(points, distances, config.fit_radius)
    candidates = _merge_ranges(
        _candidate_ranges(grades, config.grade_threshold), distances,
        config.maximum_core_gap)
    segments = []
    for start, end, sign in candidates:
        length = distances[end] - distances[start]
        height_change = points[end][2] - points[start][2]
        if (length + 1e-9 < config.minimum_core_length or
                abs(height_change) + 1e-9 < config.minimum_height_change):
            continue
        segment_grades = grades[start:end + 1]
        mean_grade = sum(segment_grades) / len(segment_grades)
        direction = "uphill" if sign > 0 else "downhill"
        segments.append(SlopeSegment(
            start, end, direction, mean_grade, length, height_change))
    return distances, grades, segments


def annotate_document(document, config=SlopeConfig()):
    """Return a copy whose only generated per-vertex attribute is isSlope."""
    ordered, points = ordered_vertices(document)
    distances, _grades, segments = detect_slope_segments(points, config)
    result = copy.deepcopy(document)
    result_vertices = [result["vertices"][str(index)]
                       for index in range(1, len(ordered) + 1)]

    affected_owner = [None] * len(points)
    for segment_index, segment in enumerate(segments):
        lower = distances[segment.start] - config.buffer_distance
        upper = distances[segment.end] + config.buffer_distance
        for index, distance in enumerate(distances):
            if lower - 1e-9 <= distance <= upper + 1e-9:
                previous = affected_owner[index]
                if previous is None:
                    affected_owner[index] = segment_index
                else:
                    old = segments[previous]
                    old_distance = min(
                        abs(distance - distances[old.start]),
                        abs(distance - distances[old.end]))
                    new_distance = min(
                        abs(distance - distances[segment.start]),
                        abs(distance - distances[segment.end]))
                    if new_distance < old_distance:
                        affected_owner[index] = segment_index

    slope_count = 0
    for index, vertex in enumerate(result_vertices):
        metadata = vertex.setdefault("meta", {})
        if not isinstance(metadata, dict):
            raise ValueError(f"vertex {index + 1}.meta must be an object")
        # Remove fields written by the earlier preview format. Existing product
        # metadata such as isCorner/type/typeId remains untouched.
        for generated_key in (
                "terrainType", "isSlopeCore", "slopeDirection",
                "localSlope", "plannerPathHeight"):
            metadata.pop(generated_key, None)
        is_slope = affected_owner[index] is not None
        metadata["isSlope"] = is_slope
        slope_count += int(is_slope)

    result["slopeAnnotation"] = {
        "method": "local_linear_z_over_xy_distance",
        "fitRadius": config.fit_radius,
        "gradeThreshold": config.grade_threshold,
        "minimumCoreLength": config.minimum_core_length,
        "minimumHeightChange": config.minimum_height_change,
        "maximumCoreGap": config.maximum_core_gap,
        "bufferDistance": config.buffer_distance,
        "counts": {"normal": len(points) - slope_count, "slope": slope_count},
        "segments": [{
            "startVertex": segment.start + 1,
            "endVertex": segment.end + 1,
            "direction": segment.direction,
            "meanGrade": round(segment.mean_grade, 6),
            "length": round(segment.length, 6),
            "heightChange": round(segment.height_change, 6),
        } for segment in segments],
    }
    return result
