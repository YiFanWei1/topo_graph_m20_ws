import math

from route3d_topology_core import (
    IncrementalTopologyBuilder,
    LoopClosureConfig,
    PoseSample,
    RetraceConfig,
    TopologyConfig,
    annotate_graph_slopes,
)
from route_slope_annotator.slope_analysis import SlopeConfig


def sample(stamp, x, y=0.0, z=0.0):
    return PoseSample(stamp, (x, y, z), (0.0, 0.0, 0.0, 1.0))


def feed(builder, points, step=0.1):
    stamp = 0.0
    builder.add_pose(sample(stamp, *points[0]))
    stamp += 0.1
    for first, second in zip(points, points[1:]):
        length = math.dist(first, second)
        count = max(1, int(round(length / step)))
        for index in range(1, count + 1):
            ratio = index / count
            builder.add_pose(sample(
                stamp,
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
                first[2] + ratio * (second[2] - first[2])))
            stamp += 0.1
    builder.finalize()


def edge_pairs(builder):
    return {frozenset((edge.first, edge.second)) for edge in builder.edges.values()}


def test_out_and_back_then_extend_keeps_old_ids_and_connects_one_to_six():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    feed(builder, [
        (0.0, 0.0, 0.0), (4.0, 0.0, 0.0),
        (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
    ])
    assert len(builder.vertices) == 6
    assert len(builder.edges) == 5
    assert edge_pairs(builder) == {
        frozenset((1, 2)), frozenset((2, 3)), frozenset((3, 4)),
        frozenset((4, 5)), frozenset((1, 6)),
    }
    assert builder.retrace_count == 1
    assert builder.suppressed_distance > 3.5


def test_noisy_retrace_is_suppressed():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    points = [(0.0, 0.0, 0.0), (4.0, 0.0, 0.0)]
    points.extend((x, 0.08 * math.sin(x * 4.0), 0.0) for x in [3.5, 3, 2.5, 2, 1.5, 1, 0.5, 0])
    points.append((-1.0, 0.0, 0.0))
    feed(builder, points)
    assert len(builder.vertices) == 6
    assert frozenset((1, 6)) in edge_pairs(builder)


def test_parallel_route_outside_exit_tolerance_is_new_discovery():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    feed(builder, [
        (0.0, 0.0, 0.0), (4.0, 0.0, 0.0),
        (4.0, 0.6, 0.0), (0.0, 0.6, 0.0),
    ])
    assert len(builder.vertices) >= 9
    assert builder.retrace_count == 0


def test_middle_departure_splits_edge_and_creates_branch():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    feed(builder, [
        (0.0, 0.0, 0.0), (4.0, 0.0, 0.0),
        (2.5, 0.0, 0.0), (2.5, 1.0, 0.0),
    ])
    junctions = [vertex for vertex in builder.vertices.values() if vertex.is_junction]
    assert len(junctions) == 1
    junction = junctions[0]
    assert math.isclose(junction.sample.point[0], 2.5, abs_tol=0.2)
    assert len(builder.adjacency[junction.vertex_id]) == 3
    assert builder.edge_split_count == 1


def test_height_separated_path_is_not_retrace():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=2.0))
    feed(builder, [
        (0.0, 0.0, 0.0), (4.0, 0.0, 0.0),
        (4.0, 0.0, 0.5), (0.0, 0.0, 0.5),
    ])
    assert builder.retrace_count == 0
    assert len(builder.vertices) >= 9


def test_relocation_starts_disconnected_component():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=0.5))
    builder.add_pose(sample(0.0, 0.0))
    builder.add_pose(sample(0.1, 0.1))
    builder.add_pose(sample(0.2, 2.0))
    builder.add_pose(sample(0.3, 2.1))
    builder.finalize()
    assert len({vertex.component for vertex in builder.vertices.values()}) == 2
    assert all(
        builder.vertices[edge.first].component == builder.vertices[edge.second].component
        for edge in builder.edges.values())


def test_stationary_localization_jitter_does_not_accumulate_route_length():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    builder.add_pose(sample(0.0, 0.0))
    for index in range(1, 101):
        builder.add_pose(sample(index * 0.1, 0.03 if index % 2 else -0.03))
    builder.finalize()
    assert len(builder.vertices) == 1
    assert len(builder.edges) == 0


def test_partial_retrace_then_forward_again_does_not_duplicate_edges():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    feed(builder, [
        (0.0, 0.0, 0.0), (4.0, 0.0, 0.0),
        (2.0, 0.0, 0.0), (4.0, 0.0, 0.0),
    ])
    assert len(builder.vertices) == 5
    assert len(builder.edges) == 4
    assert edge_pairs(builder) == {
        frozenset((1, 2)), frozenset((2, 3)),
        frozenset((3, 4)), frozenset((4, 5)),
    }


def test_retrace_confirmation_crosses_a_short_terminal_edge():
    builder = IncrementalTopologyBuilder(TopologyConfig(
        relocation_distance=1.0,
        retrace=RetraceConfig(motion_min_distance=0.01)))
    feed(builder, [
        (0.0, 0.0, 0.0), (4.08, 0.0, 0.0),
        (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
    ], step=0.04)
    assert len(builder.vertices) == 7
    assert len(builder.edges) == 6
    assert builder.retrace_count == 1
    assert builder.suppressed_distance > 3.8
    assert frozenset((1, 7)) in edge_pairs(builder)


def test_near_non_adjacent_old_vertex_does_not_close_a_loop():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    feed(builder, [
        (0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (3.0, 2.0, 0.0),
        (0.0, 2.0, 0.0), (0.0, 0.10, 0.0),
    ])
    assert len(builder.edges) == len(builder.vertices) - 1
    assert builder.retrace_count == 0
    assert builder.current_vertex_id != 1
    assert builder._edge_between(1, builder.current_vertex_id) is None


def _two_lap_circle(radial_offset=0.0):
    points = [(-2.0, 0.0, 0.0), (-1.0, 0.0, 0.0)]
    for lap in range(2):
        radius = 1.0 + (radial_offset if lap else 0.0)
        for index in range(1, 81):
            angle = math.pi + index * 2.0 * math.pi / 80.0
            points.append((radius * math.cos(angle), radius * math.sin(angle), 0.0))
    return points


def test_same_direction_second_lap_closes_once_and_is_suppressed():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    feed(builder, _two_lap_circle(radial_offset=0.15), step=0.05)

    assert builder.loop_closure_count == 1
    assert builder.route_reentry_count == 1
    assert builder.suppressed_distance > 5.0
    assert len(builder.edges) == len(builder.vertices)
    assert sum(edge.source == "loop_closure" for edge in builder.edges.values()) == 1
    assert len(builder.vertices) <= 9


def test_pcd_rejection_prevents_geometric_loop_closure():
    def reject(_current, _reference):
        return False, {"available": True, "overlap": 0.05}

    builder = IncrementalTopologyBuilder(
        TopologyConfig(
            relocation_distance=1.0,
            loop_closure=LoopClosureConfig(rejection_cooldown_distance=0.50)),
        loop_closure_validator=reject)
    # Frame indices make historical PCD references available to the validator.
    stamp = 0.0
    points = _two_lap_circle()
    builder.add_pose(PoseSample(stamp, points[0], frame_index=0))
    frame_index = 1
    for first, second in zip(points, points[1:]):
        count = max(1, int(round(math.dist(first, second) / 0.05)))
        for index in range(1, count + 1):
            ratio = index / count
            stamp += 0.1
            builder.add_pose(PoseSample(
                stamp,
                tuple(first[axis] + ratio * (second[axis] - first[axis])
                      for axis in range(3)),
                frame_index=frame_index))
            frame_index += 1
    builder.finalize()

    assert builder.loop_closure_count == 0
    assert builder.loop_validation_rejection_count >= 1
    assert len(builder.edges) == len(builder.vertices) - 1


def test_graph_slope_analysis_does_not_cross_disconnected_numeric_ids():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=0.5))
    for index in range(4):
        builder.add_pose(sample(index * 0.1, index * 0.1, z=0.0))
    builder.add_pose(sample(0.5, 5.0, z=5.0))
    for index in range(1, 4):
        builder.add_pose(sample(0.5 + index * 0.1, 5.0 + index * 0.1, z=5.0))
    builder.finalize()
    annotate_graph_slopes(builder, SlopeConfig(
        fit_radius=0.1, grade_threshold=0.1,
        minimum_core_length=0.1, minimum_height_change=0.1,
        maximum_core_gap=0.2, buffer_distance=0.0,
    ))
    assert not any(vertex.is_slope for vertex in builder.vertices.values())


def test_corner_split_keeps_exit_anchor_valid():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    stamp = 0.0
    for x in [index * 0.1 for index in range(41)]:
        builder.add_pose(sample(stamp, x))
        stamp += 0.1
    for x in [4.0 - index * 0.1 for index in range(1, 16)]:
        builder.add_pose(sample(stamp, x))
        stamp += 0.1
    builder.add_pose(sample(stamp, 2.5, 0.1))
    stamp += 0.1
    assert builder.state == builder.EXIT_PENDING

    builder.mark_corner(sample(stamp, 2.5), 90.0, merge_distance=0.05)
    builder.add_pose(sample(stamp + 0.1, 2.5, 0.4))
    builder.finalize()

    assert builder.edge_split_count == 1
    assert any(vertex.is_corner and vertex.is_junction
               for vertex in builder.vertices.values())


def test_vertex_snap_chooses_nearest_endpoint_when_both_are_inside_threshold():
    builder = IncrementalTopologyBuilder(TopologyConfig(relocation_distance=1.0))
    first = builder._add_vertex(sample(0.0, 0.0))
    second = builder._add_vertex(sample(0.1, 0.30))
    edge_id = builder._add_edge(first, second)
    projection = builder._project(
        edge_id, sample(0.2, 0.28), (0.10, 0.0, 0.0))

    assert builder._snap_projection_vertex(projection, threshold=0.45) == second
