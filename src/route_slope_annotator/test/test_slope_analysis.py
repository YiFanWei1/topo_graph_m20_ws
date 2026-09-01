import math

from route_slope_annotator.slope_analysis import (
    SlopeConfig, annotate_document, detect_slope_segments)


def route_points(direction=1.0):
    points = []
    for index in range(20):
        if index <= 4:
            z = 0.0
        elif index <= 10:
            z = direction * 0.20 * (index - 4)
        else:
            z = direction * 1.20
        points.append((float(index), 0.0, z))
    return points


def document_for(points):
    vertices = {}
    edges = {}
    for index, point in enumerate(points, start=1):
        vertices[str(index)] = {
            "pos": list(point),
            "meta": {"isCorner": index == 8},
        }
        if index > 1:
            edges[str(index - 1)] = {"v": [index - 1, index], "weight": 1.0}
    return {"version": 0, "type": 0, "frame_id": "map",
            "vertices": vertices, "edges": edges}


def config():
    return SlopeConfig(
        fit_radius=1.0, grade_threshold=0.10,
        minimum_core_length=2.0, minimum_height_change=0.20,
        maximum_core_gap=1.0, buffer_distance=3.0)


def test_detects_uphill_and_downhill_from_signed_z_trend():
    _, _, uphill = detect_slope_segments(route_points(1.0), config())
    _, _, downhill = detect_slope_segments(route_points(-1.0), config())

    assert len(uphill) == 1
    assert uphill[0].direction == "uphill"
    assert uphill[0].mean_grade > 0.10
    assert len(downhill) == 1
    assert downhill[0].direction == "downhill"
    assert downhill[0].mean_grade < -0.10


def test_annotation_expands_three_metres_and_sets_planner_height():
    annotated = annotate_document(document_for(route_points()), config())
    vertices = annotated["vertices"]
    core_ids = [index for index in range(1, 21)
                if vertices[str(index)]["meta"]["isSlopeCore"]]
    affected_ids = [index for index in range(1, 21)
                    if vertices[str(index)]["meta"]["isSlope"]]

    assert min(affected_ids) <= max(1, min(core_ids) - 3)
    assert max(affected_ids) >= min(20, max(core_ids) + 3)
    assert all(math.isclose(
        vertices[str(index)]["meta"]["plannerPathHeight"], 0.4)
        for index in affected_ids)
    assert all(math.isclose(
        vertices[str(index)]["meta"]["plannerPathHeight"], 0.0)
        for index in set(range(1, 21)) - set(affected_ids))
    assert vertices["8"]["meta"]["isCorner"] is True


def test_small_flat_z_noise_is_not_a_slope():
    points = [(float(index), 0.0, 0.015 * math.sin(index))
              for index in range(30)]
    _, grades, segments = detect_slope_segments(points, config())

    assert max(abs(grade) for grade in grades) < 0.10
    assert segments == []
