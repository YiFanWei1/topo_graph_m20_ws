from route3d_loop_patrol.patrol_logic import Event, PatrolCoordinator, Phase


def test_next_leg_requires_matching_new_route_and_explicit_finished():
    logic = PatrolCoordinator(10, 20)
    logic.start()
    assert logic.request_pair(7).start_id == 10
    assert logic.observe_plan(True, 10, 20) is Event.PLAN_ACCEPTED

    # A stale FINISHED from the previous route cannot advance the patrol.
    assert logic.observe_controller(7, 'FINISHED', False) is Event.IGNORED
    assert logic.phase is Phase.WAITING_ROUTE

    assert logic.observe_route(8, 10, 20) is Event.ROUTE_ACCEPTED
    assert logic.observe_controller(8, 'TRACKING', True) is Event.ROUTE_ACCEPTED
    assert logic.phase is Phase.WAITING_ARRIVAL
    assert logic.observe_controller(8, 'FINISHED', True) is Event.ROUTE_ACCEPTED
    assert logic.phase is Phase.WAITING_ARRIVAL

    assert logic.observe_controller(8, 'FINISHED', False) is Event.ARRIVED
    assert logic.phase is Phase.DWELL
    logic.finish_dwell()
    reverse = logic.request_pair(8)
    assert (reverse.start_id, reverse.goal_id) == (20, 10)


def test_unrelated_plan_status_is_ignored():
    logic = PatrolCoordinator(1, 9)
    logic.start()
    logic.request_pair(3)
    assert logic.observe_plan(True, 2, 8) is Event.IGNORED
    assert logic.phase is Phase.WAITING_PLAN


def test_goal_only_plan_accepts_and_records_resolved_start():
    logic = PatrolCoordinator(1, 9)
    logic.start()
    logic.request_pair(3)
    assert logic.observe_plan(
        True, 4, 9, allow_resolved_start=True) is Event.PLAN_ACCEPTED
    assert logic.current_pair() == (4, 9)


def test_goal_only_failure_does_not_require_resolved_start():
    logic = PatrolCoordinator(1, 9)
    logic.start()
    logic.request_pair(3)
    assert logic.observe_plan(
        False, None, 9, 'nearest vertex is too far',
        allow_resolved_start=True) is Event.ERROR
    assert logic.error == 'nearest vertex is too far'


def test_unrelated_sliced_route_cannot_supply_arrival_sequence():
    logic = PatrolCoordinator(1, 9)
    logic.start()
    logic.request_pair(3)
    logic.observe_plan(True, 1, 9)
    assert logic.observe_route(4, 2, 8) is Event.IGNORED
    assert logic.observe_controller(4, 'FINISHED', False) is Event.IGNORED
    assert logic.phase is Phase.WAITING_ROUTE


def test_external_route_replacement_stops_patrol():
    logic = PatrolCoordinator(1, 9)
    logic.start()
    logic.request_pair(3)
    logic.observe_plan(True, 1, 9)
    logic.observe_route(4, 1, 9)
    logic.observe_controller(4, 'TRACKING', True)
    assert logic.observe_controller(5, 'TRACKING', True) is Event.ERROR
    assert logic.phase is Phase.ERROR


def test_round_trip_limit_stops_after_return_leg():
    logic = PatrolCoordinator(1, 9, max_round_trips=1)
    logic.start()
    logic.request_pair(0)
    logic.observe_plan(True, 1, 9)
    logic.observe_route(1, 1, 9)
    assert logic.observe_controller(1, 'FINISHED', False) is Event.ARRIVED
    logic.finish_dwell()
    logic.request_pair(1)
    logic.observe_plan(True, 9, 1)
    logic.observe_route(2, 9, 1)
    assert logic.observe_controller(2, 'FINISHED', False) is Event.COMPLETED
    assert logic.completed_round_trips == 1
    assert logic.phase is Phase.COMPLETED
