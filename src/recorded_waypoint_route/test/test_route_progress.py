import pytest

from recorded_waypoint_route.route_utils import (
    nearest_target,
    ordered_target_indices,
    RouteProgress,
    RouteState,
    TargetType,
)


BODY_TARGETS = [
    (0.0, 0.0, 0.4),
    (1.0, 0.0, 0.4),
    (1.0, 1.0, 1.4),
    (2.0, 1.0, 1.4),
]


def test_nearest_target_uses_three_dimensional_distance():
    index, distance = nearest_target((1.0, 1.0, 0.3), BODY_TARGETS)
    assert index == 1
    assert distance == pytest.approx((1.0 ** 2 + 0.1 ** 2) ** 0.5)


def test_non_looping_sequence_supports_both_directions_and_same_point():
    assert ordered_target_indices(1, 3, 4) == [1, 2, 3]
    assert ordered_target_indices(3, 0, 4) == [3, 2, 1, 0]
    assert ordered_target_indices(2, 2, 4) == [2]


@pytest.mark.parametrize("start, goal", [(-1, 1), (0, 4), (4, 0)])
def test_sequence_rejects_out_of_range_ids(start, goal):
    with pytest.raises(ValueError):
        ordered_target_indices(start, goal, 4)


def test_forward_targets_advance_strictly_without_skipping():
    progress = RouteProgress([1, 2, 3], 0.15)
    outside = progress.update((1.151, 0.0, 0.4), BODY_TARGETS)
    assert not outside.changed
    assert outside.required_target == 1

    first = progress.update(BODY_TARGETS[1], BODY_TARGETS)
    assert first.event == "target_advanced"
    assert first.active_leg == (1, 2)
    assert first.required_target == 2

    # Being at target 4 cannot bypass required target 3.
    later = progress.update(BODY_TARGETS[3], BODY_TARGETS)
    assert not later.changed
    assert later.required_target == 2

    middle = progress.update(BODY_TARGETS[2], BODY_TARGETS)
    assert middle.active_leg == (2, 3)
    assert middle.required_target == 3
    finish = progress.update(BODY_TARGETS[3], BODY_TARGETS)
    assert finish.event == "goal_reached"
    assert finish.state == RouteState.COMPLETE


def test_reverse_targets_advance_in_descending_order():
    progress = RouteProgress([3, 2, 1], 0.15)
    first = progress.update(BODY_TARGETS[3], BODY_TARGETS)
    assert first.active_leg == (3, 2)
    second = progress.update(BODY_TARGETS[2], BODY_TARGETS)
    assert second.active_leg == (2, 1)
    finish = progress.update(BODY_TARGETS[1], BODY_TARGETS)
    assert finish.state == RouteState.COMPLETE


def test_xy_match_with_wrong_height_does_not_advance():
    progress = RouteProgress([2, 3], 0.15)
    update = progress.update((1.0, 1.0, 1.249), BODY_TARGETS)
    assert not update.changed
    assert update.required_target == 2


def test_single_target_selection_completes_only_at_arrival_boundary():
    progress = RouteProgress([1], 0.15)
    outside = progress.update((1.151, 0.0, 0.4), BODY_TARGETS)
    assert not outside.changed
    boundary = progress.update((1.15, 0.0, 0.4), BODY_TARGETS)
    assert boundary.event == "goal_reached"
    assert boundary.state == RouteState.COMPLETE


def test_completed_route_never_restarts():
    progress = RouteProgress([0], 0.15)
    progress.update(BODY_TARGETS[0], BODY_TARGETS)
    update = progress.update((3.0, 0.0, 0.4), BODY_TARGETS)
    assert update.state == RouteState.COMPLETE
    assert not update.changed


def test_point_type_tolerances_and_final_goal_override():
    progress = RouteProgress(
        [0, 1, 2], 0.50,
        [TargetType.NORMAL, TargetType.CORNER, TargetType.NORMAL],
        corner_arrival_tolerance=0.20,
        goal_arrival_tolerance=0.15,
    )
    assert progress.update((-0.50, 0.0, 0.4), BODY_TARGETS).changed
    assert progress.effective_tolerance() == pytest.approx(0.20)
    assert not progress.update((1.0, -0.201, 0.4), BODY_TARGETS).changed
    assert progress.update((1.0, -0.20, 0.4), BODY_TARGETS).changed
    assert progress.required_target_is_goal()
    assert progress.effective_tolerance() == pytest.approx(0.15)


def test_reverse_route_uses_corner_then_final_goal_tolerance():
    progress = RouteProgress(
        [2, 1, 0], 2.80,
        [TargetType.NORMAL, TargetType.CORNER, TargetType.NORMAL],
        corner_arrival_tolerance=0.20,
        goal_arrival_tolerance=0.15,
    )
    assert progress.update(BODY_TARGETS[2], BODY_TARGETS).changed
    assert progress.required_target_type() == TargetType.CORNER
    assert not progress.update((1.0, -0.201, 0.4), BODY_TARGETS).changed
    assert progress.update((1.0, -0.20, 0.4), BODY_TARGETS).changed
    assert progress.required_target_is_goal()
    assert not progress.update((0.151, 0.0, 0.4), BODY_TARGETS).changed
    assert progress.update((0.15, 0.0, 0.4), BODY_TARGETS).state == RouteState.COMPLETE
