import json
import math

import pytest

from recorded_waypoint_route.route_utils import (
    atomic_write_target_file,
    distance_3d,
    interpolate_targets,
    load_route_file,
    load_target_file,
    parse_target_text,
    parse_topo_single_json,
    split_interpolated_path,
    TargetType,
)


def topo_document():
    return {
        "version": 3,
        "type": 0,
        "frame_id": "map",
        "vertices": {
            "3": {"pos": [2.0, 1.0, -0.4], "meta": {"isCorner": False}},
            "1": {"pos": [0.0, 0.0, -0.4], "meta": {"isCorner": False}},
            "2": {"pos": [1.0, 0.0, -0.4], "meta": {"isCorner": True}},
        },
        "edges": {
            "9": {"v": [2, 3], "meta": {"dir": 0}},
            "4": {"v": [2, 1], "meta": {"dir": 0}},
        },
    }


def test_parse_topo_single_json_uses_vertex_ids_edges_and_corner_metadata():
    parsed = parse_topo_single_json(json.dumps(topo_document()))
    assert parsed.frame_id == "map"
    assert parsed.points == [
        (0.0, 0.0, -0.4),
        (1.0, 0.0, -0.4),
        (2.0, 1.0, -0.4),
    ]
    assert parsed.types == [TargetType.NORMAL, TargetType.CORNER, TargetType.NORMAL]
    assert parsed.is_slope == [False, False, False]


def test_parse_topo_single_json_reads_slope_attribute():
    document = topo_document()
    document["vertices"]["2"]["meta"]["isSlope"] = True
    parsed = parse_topo_single_json(json.dumps(document))
    assert parsed.is_slope == [False, True, False]


def test_load_route_file_selects_json_parser_from_suffix(tmp_path):
    output = tmp_path / "topoSingle_data.json"
    output.write_text(json.dumps(topo_document()), encoding="utf-8")
    assert load_route_file(str(output)).types[1] == TargetType.CORNER


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda data: data["vertices"].pop("2"), "contiguous"),
        (lambda data: data["edges"].__setitem__(
            "9", {"v": [1, 3], "meta": {"dir": 0}}), "single chain"),
        (lambda data: data["edges"]["9"]["meta"].__setitem__("dir", 1), "directed"),
        (lambda data: data["vertices"]["2"]["meta"].__setitem__(
            "isCorner", "true"), "boolean"),
        (lambda data: data["vertices"]["2"]["meta"].__setitem__(
            "isSlope", 1), "boolean"),
    ],
)
def test_parse_topo_single_json_rejects_unsupported_or_invalid_graphs(mutate, message):
    document = topo_document()
    mutate(document)
    with pytest.raises(ValueError, match=message):
        parse_topo_single_json(json.dumps(document))


def test_parse_target_text_reads_frame_and_ordered_ground_points():
    parsed = parse_target_text(
        "# frame_id: camera_init\n# z_semantics: ground\n1 1 2 3\n2 4.5 5.5 -6\n"
    )
    assert parsed.frame_id == "camera_init"
    assert parsed.points == [(1.0, 2.0, 3.0), (4.5, 5.5, -6.0)]
    assert parsed.types == [TargetType.CORNER, TargetType.CORNER]
    assert parsed.is_slope == [False, False]


def test_parse_target_text_reads_typed_rows_and_inline_metadata():
    parsed = parse_target_text(
        "# frame_id: map\n1 1 2 3 NORMAL\n2 4 5 6 CORNER # turn_deg=90\n"
    )
    assert parsed.points == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    assert parsed.types == [TargetType.NORMAL, TargetType.CORNER]


@pytest.mark.parametrize(
    "text",
    [
        "1 2\n",
        "1 2 3 4 5\n",
        "1 bad 3\n",
        "1 nan 3\n",
        "1 inf 3\n",
        "2 1 2 3\n",
        "one 1 2 3\n",
        "1 1 2 3\n4 4 5 6\n",
        "1 1 2 3\n4 5 6\n",
    ],
)
def test_parse_target_text_rejects_invalid_rows(text):
    with pytest.raises(ValueError):
        parse_target_text(text)


def test_atomic_write_replaces_file_with_parseable_content(tmp_path):
    output = tmp_path / "target.txt"
    output.write_text("incomplete", encoding="utf-8")
    points = [(1.23456789, 2.0, -3.0), (4.0, 5.0, 6.0)]
    atomic_write_target_file(str(output), "camera_init", points)
    assert load_target_file(str(output)).points == [
        (1.234568, 2.0, -3.0),
        (4.0, 5.0, 6.0),
    ]
    assert load_target_file(str(output)).types == [
        TargetType.CORNER,
        TargetType.CORNER,
    ]
    assert output.read_text(encoding="utf-8").splitlines()[2] == (
        "# columns: id x y z_ground type")
    assert output.read_text(encoding="utf-8").splitlines()[3].endswith(" CORNER")
    assert output.read_text(encoding="utf-8").splitlines()[4].endswith(" CORNER")
    assert output.read_text(encoding="utf-8").splitlines()[3].startswith("1 ")
    assert output.read_text(encoding="utf-8").splitlines()[4].startswith("2 ")
    assert not (tmp_path / "target.txt.tmp").exists()


def test_legacy_unnumbered_file_remains_readable():
    parsed = parse_target_text("1 2 3\n4 5 6\n")
    assert parsed.points == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    assert parsed.types == [TargetType.CORNER, TargetType.CORNER]


def test_atomic_write_preserves_explicit_normal_type(tmp_path):
    output = tmp_path / "normal_target.txt"
    atomic_write_target_file(
        str(output), "map", [(1.0, 2.0, 3.0)], [TargetType.NORMAL])
    assert load_target_file(str(output)).types == [TargetType.NORMAL]


def test_interpolation_limits_three_dimensional_spacing_and_preserves_targets():
    targets = [(0.0, 0.0, 0.0), (0.24, 0.0, 0.18), (0.24, 0.4, 0.18)]
    path = interpolate_targets(targets, 0.10)
    assert path[0] == targets[0]
    assert targets[1] in path
    assert path[-1] == targets[-1]
    assert max(distance_3d(a, b) for a, b in zip(path, path[1:])) <= 0.10 + 1e-12
    segments = split_interpolated_path(targets, path)
    assert len(segments) == 2
    assert segments[0][0] == targets[0]
    assert segments[0][-1] == targets[1]
    assert segments[1][0] == targets[1]
    assert segments[1][-1] == targets[2]


def test_duplicate_targets_are_rejected():
    with pytest.raises(ValueError, match="duplicates"):
        interpolate_targets([(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)], 0.10)


@pytest.mark.parametrize("spacing", [0.0, -0.1, math.inf, math.nan])
def test_invalid_spacing_is_rejected(spacing):
    with pytest.raises(ValueError):
        interpolate_targets([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], spacing)


def test_split_rejects_a_path_that_lost_an_original_target():
    targets = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    with pytest.raises(ValueError, match="every target"):
        split_interpolated_path(targets, [targets[0], (0.5, 0.0, 0.0), targets[2]])
