from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class Phase(str, Enum):
    STOPPED = 'STOPPED'
    READY = 'READY'
    WAITING_PLAN = 'WAITING_PLAN'
    WAITING_ROUTE = 'WAITING_ROUTE'
    WAITING_ARRIVAL = 'WAITING_ARRIVAL'
    DWELL = 'DWELL'
    COMPLETED = 'COMPLETED'
    ERROR = 'ERROR'


class Event(str, Enum):
    IGNORED = 'IGNORED'
    PLAN_ACCEPTED = 'PLAN_ACCEPTED'
    ROUTE_ACCEPTED = 'ROUTE_ACCEPTED'
    ARRIVED = 'ARRIVED'
    COMPLETED = 'COMPLETED'
    ERROR = 'ERROR'


@dataclass(frozen=True)
class PatrolPair:
    start_id: int
    goal_id: int


class PatrolCoordinator:
    """Pure state machine that prevents a new leg before explicit arrival."""

    def __init__(
        self,
        start_id: int,
        goal_id: int,
        initial_direction: str = 'start_to_goal',
        max_round_trips: int = 0,
    ) -> None:
        if start_id == goal_id:
            raise ValueError('start_vertex_id and goal_vertex_id must be different')
        if initial_direction not in ('start_to_goal', 'goal_to_start'):
            raise ValueError(
                "initial_direction must be 'start_to_goal' or 'goal_to_start'")
        if max_round_trips < 0:
            raise ValueError('max_round_trips must be non-negative')
        self.start_id = start_id
        self.goal_id = goal_id
        self.initial_direction = initial_direction
        self.max_round_trips = max_round_trips
        self.phase = Phase.STOPPED
        self.direction = 0
        self.completed_legs = 0
        self.completed_round_trips = 0
        self.baseline_route_sequence = -1
        self.active_route_sequence: Optional[int] = None
        self.pending_pair: Optional[PatrolPair] = None
        self.error = ''

    def start(self) -> None:
        self.direction = 0 if self.initial_direction == 'start_to_goal' else 1
        self.completed_legs = 0
        self.completed_round_trips = 0
        self.baseline_route_sequence = -1
        self.active_route_sequence = None
        self.pending_pair = None
        self.error = ''
        self.phase = Phase.READY

    def stop(self) -> None:
        self.phase = Phase.STOPPED
        self.active_route_sequence = None
        self.pending_pair = None

    def request_pair(self, current_route_sequence: int) -> PatrolPair:
        if self.phase is not Phase.READY:
            raise RuntimeError(f'cannot request a route while phase is {self.phase.value}')
        pair = PatrolPair(self.start_id, self.goal_id) if self.direction == 0 else PatrolPair(
            self.goal_id, self.start_id)
        self.baseline_route_sequence = current_route_sequence
        self.active_route_sequence = None
        self.pending_pair = pair
        self.phase = Phase.WAITING_PLAN
        return pair

    def observe_plan(self, success: bool, start_id: int, goal_id: int, error: str = '') -> Event:
        if self.phase is not Phase.WAITING_PLAN or self.pending_pair is None:
            return Event.IGNORED
        if (start_id, goal_id) != (self.pending_pair.start_id, self.pending_pair.goal_id):
            return Event.IGNORED
        if not success:
            return self.fail(error or 'Dijkstra planning failed')
        self.phase = Phase.WAITING_ROUTE
        return Event.PLAN_ACCEPTED

    def observe_route(self, route_sequence: int, start_id: int, goal_id: int) -> Event:
        if self.phase is not Phase.WAITING_ROUTE or self.pending_pair is None:
            return Event.IGNORED
        if (start_id, goal_id) != (self.pending_pair.start_id, self.pending_pair.goal_id):
            return Event.IGNORED
        if route_sequence <= self.baseline_route_sequence:
            return Event.IGNORED
        self.active_route_sequence = route_sequence
        self.phase = Phase.WAITING_ARRIVAL
        return Event.ROUTE_ACCEPTED

    def observe_controller(self, route_sequence: int, state: str, active: bool) -> Event:
        if self.phase is not Phase.WAITING_ARRIVAL or self.active_route_sequence is None:
            return Event.IGNORED
        if route_sequence < self.active_route_sequence:
            return Event.IGNORED
        if route_sequence != self.active_route_sequence:
            return self.fail(
                'active route was replaced before patrol arrival confirmation')
        if state in ('ERROR', 'CANCELLED'):
            return self.fail(f'controller entered terminal state {state}')
        # FINISHED plus inactive is the controller's explicit whole-route arrival signal.
        if state != 'FINISHED' or active:
            return Event.ROUTE_ACCEPTED
        self.completed_legs += 1
        if self.direction == 1:
            self.completed_round_trips += 1
        if self.max_round_trips > 0 and self.completed_round_trips >= self.max_round_trips:
            self.phase = Phase.COMPLETED
            return Event.COMPLETED
        self.direction = 1 - self.direction
        self.phase = Phase.DWELL
        return Event.ARRIVED

    def finish_dwell(self) -> None:
        if self.phase is not Phase.DWELL:
            raise RuntimeError(f'cannot finish dwell while phase is {self.phase.value}')
        self.phase = Phase.READY

    def fail(self, message: str) -> Event:
        self.error = message
        self.phase = Phase.ERROR
        return Event.ERROR

    def current_pair(self) -> Optional[Tuple[int, int]]:
        if self.pending_pair is None:
            return None
        return self.pending_pair.start_id, self.pending_pair.goal_id
