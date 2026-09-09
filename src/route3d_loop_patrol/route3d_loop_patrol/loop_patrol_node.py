import json
import math
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from route3d_route_slicer.msg import RouteTaskArray
from std_msgs.msg import Int32, Int32MultiArray, String
from std_srvs.srv import Trigger

from route3d_loop_patrol.patrol_logic import Event, PatrolCoordinator, Phase


def latched_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class LoopPatrolNode(Node):
    def __init__(self) -> None:
        super().__init__('route3d_loop_patrol')
        start_id = self.declare_parameter('patrol.start_vertex_id', 1).value
        goal_id = self.declare_parameter('patrol.goal_vertex_id', 2).value
        initial_direction = self.declare_parameter(
            'patrol.initial_direction', 'start_to_goal').value
        max_round_trips = self.declare_parameter('patrol.max_round_trips', 0).value
        self.auto_start = self.declare_parameter('patrol.auto_start', True).value
        self.request_mode = str(self.declare_parameter(
            'patrol.request_mode', 'start_goal').value)
        if self.request_mode not in ('start_goal', 'goal_only'):
            raise ValueError("patrol.request_mode must be 'start_goal' or 'goal_only'")
        self.wait_for_idle_before_first_request = self.declare_parameter(
            'patrol.wait_for_idle_before_first_request', True).value
        self.startup_delay_s = self.declare_parameter('patrol.startup_delay_s', 3.0).value
        self.dwell_time_s = self.declare_parameter('patrol.dwell_time_s', 2.0).value
        self.plan_timeout_s = self.declare_parameter('timeouts.plan_response_s', 10.0).value
        self.route_timeout_s = self.declare_parameter('timeouts.route_activation_s', 10.0).value
        self.arrival_timeout_s = self.declare_parameter('timeouts.arrival_s', 0.0).value
        request_topic = self.declare_parameter(
            'topics.plan_request', '/route3d_dijkstra/plan_request').value
        goal_request_topic = self.declare_parameter(
            'topics.goal_request', '/route3d_dijkstra/goal_request').value
        dijkstra_status_topic = self.declare_parameter(
            'topics.dijkstra_status', '/route3d_dijkstra/status').value
        controller_status_topic = self.declare_parameter(
            'topics.controller_status', '/route3d_pid_controller/status').value
        sliced_route_topic = self.declare_parameter(
            'topics.sliced_route', '/route3d_route_slicer/tasks').value
        status_topic = self.declare_parameter(
            'topics.status', '/route3d_loop_patrol/status').value

        for name, value in (
            ('patrol.startup_delay_s', self.startup_delay_s),
            ('patrol.dwell_time_s', self.dwell_time_s),
            ('timeouts.plan_response_s', self.plan_timeout_s),
            ('timeouts.route_activation_s', self.route_timeout_s),
            ('timeouts.arrival_s', self.arrival_timeout_s),
        ):
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f'{name} must be finite and non-negative')

        self.logic = PatrolCoordinator(
            int(start_id), int(goal_id), str(initial_direction), int(max_round_trips))
        self.last_controller_route_sequence = -1
        self.last_controller_status: Optional[Dict[str, Any]] = None
        self.last_sliced_route: Optional[RouteTaskArray] = None
        self.phase_deadline: Optional[float] = None
        self.last_status_detail = 'loop patrol initialized'

        self.request_publisher = self.create_publisher(
            Int32MultiArray, request_topic, QoSProfile(depth=10))
        self.goal_request_publisher = self.create_publisher(
            Int32, goal_request_topic, QoSProfile(depth=10))
        self.status_publisher = self.create_publisher(String, status_topic, latched_qos())
        self.dijkstra_subscription = self.create_subscription(
            String, dijkstra_status_topic, self.dijkstra_status_callback, latched_qos())
        self.controller_subscription = self.create_subscription(
            String, controller_status_topic, self.controller_status_callback, latched_qos())
        self.sliced_route_subscription = self.create_subscription(
            RouteTaskArray, sliced_route_topic, self.sliced_route_callback, latched_qos())
        self.start_service = self.create_service(Trigger, '~/start', self.start_callback)
        self.stop_service = self.create_service(Trigger, '~/stop', self.stop_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)

        if self.auto_start:
            self.logic.start()
            self.phase_deadline = time.monotonic() + float(self.startup_delay_s)
            self.last_status_detail = 'waiting for startup delay'
        self.publish_status()
        self.get_logger().info(
            f'loop patrol ready: {start_id}<->{goal_id}, auto_start={self.auto_start}, '
            f'dwell={self.dwell_time_s:.2f}s, max_round_trips={max_round_trips}')

    def start_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.logic.phase not in (Phase.STOPPED, Phase.COMPLETED, Phase.ERROR):
            response.success = False
            response.message = f'patrol is already {self.logic.phase.value}'
            return response
        self.logic.start()
        self.phase_deadline = time.monotonic() + float(self.startup_delay_s)
        self.last_status_detail = 'patrol started; waiting for startup delay'
        self.publish_status()
        response.success = True
        response.message = self.last_status_detail
        return response

    def stop_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.logic.stop()
        self.phase_deadline = None
        self.last_status_detail = 'patrol loop stopped; current robot route was not cancelled'
        self.publish_status()
        response.success = True
        response.message = self.last_status_detail
        return response

    def dijkstra_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            if str(status.get('request_mode', 'start_goal')) != self.request_mode:
                return
            goal_id = status.get('goal_id')
            if goal_id is None:
                return
            start_value = status.get('start_id')
            start_id = int(start_value) if start_value is not None else None
            event = self.logic.observe_plan(
                bool(status.get('success', False)), start_id, int(goal_id),
                str(status.get('error', '')),
                allow_resolved_start=self.request_mode == 'goal_only')
            if event is Event.PLAN_ACCEPTED:
                self.phase_deadline = time.monotonic() + float(self.route_timeout_s)
                self.last_status_detail = (
                    'Dijkstra plan accepted; waiting for new controller route')
                self._consider_last_sliced_route()
                self._consider_last_controller_status()
                self.publish_status()
            elif event is Event.ERROR:
                self._report_error()
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'ignored malformed Dijkstra status: {error}')

    def controller_status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
            route_sequence = int(status.get('route_sequence', 0))
            self.last_controller_route_sequence = max(
                self.last_controller_route_sequence, route_sequence)
            self.last_controller_status = status
            self._consider_last_controller_status()
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().warning(f'ignored malformed controller status: {error}')

    def sliced_route_callback(self, message: RouteTaskArray) -> None:
        self.last_sliced_route = message
        self._consider_last_sliced_route()

    def _consider_last_sliced_route(self) -> None:
        if self.last_sliced_route is None:
            return
        route = self.last_sliced_route
        event = self.logic.observe_route(
            int(route.route_sequence), int(route.route_start_id), int(route.route_goal_id))
        if event is not Event.ROUTE_ACCEPTED:
            return
        if self.arrival_timeout_s > 0.0:
            self.phase_deadline = time.monotonic() + float(self.arrival_timeout_s)
        else:
            self.phase_deadline = None
        self.last_status_detail = (
            f'sliced route {self.logic.active_route_sequence} matched; '
            'waiting for explicit controller FINISHED signal')
        self.publish_status()
        self._consider_last_controller_status()

    def _consider_last_controller_status(self) -> None:
        if self.last_controller_status is None:
            return
        status = self.last_controller_status
        event = self.logic.observe_controller(
            int(status.get('route_sequence', 0)), str(status.get('state', '')),
            bool(status.get('active', False)))
        if event is Event.ARRIVED:
            pair = self.logic.current_pair()
            self.phase_deadline = time.monotonic() + float(self.dwell_time_s)
            self.last_status_detail = (
                f'explicit FINISHED received for {pair[0]}->{pair[1]}; dwelling before reverse')
            self.get_logger().info(self.last_status_detail)
            self.publish_status()
        elif event is Event.COMPLETED:
            self.phase_deadline = None
            self.last_status_detail = 'configured round-trip count completed'
            self.get_logger().info(self.last_status_detail)
            self.publish_status()
        elif event is Event.ERROR:
            self._report_error()

    def timer_callback(self) -> None:
        now = time.monotonic()
        if self.logic.phase is Phase.READY:
            if self.phase_deadline is not None and now < self.phase_deadline:
                return
            if (
                self.logic.completed_legs == 0
                and self.wait_for_idle_before_first_request
                and not self._controller_allows_first_request()
            ):
                detail = 'waiting for the current controller route to finish before first request'
                if self.last_status_detail != detail:
                    self.last_status_detail = detail
                    self.publish_status()
                return
            publisher = (
                self.goal_request_publisher
                if self.request_mode == 'goal_only'
                else self.request_publisher)
            if publisher.get_subscription_count() == 0:
                self.last_status_detail = 'waiting for Dijkstra request subscriber'
                return
            pair = self.logic.request_pair(self.last_controller_route_sequence)
            if self.request_mode == 'goal_only':
                message = Int32()
                message.data = pair.goal_id
                self.goal_request_publisher.publish(message)
            else:
                message = Int32MultiArray()
                message.data = [pair.start_id, pair.goal_id]
                self.request_publisher.publish(message)
            self.phase_deadline = now + float(self.plan_timeout_s)
            self.last_status_detail = (
                f'published goal-only patrol request ->{pair.goal_id}'
                if self.request_mode == 'goal_only'
                else f'published patrol request {pair.start_id}->{pair.goal_id}')
            self.get_logger().info(self.last_status_detail)
            self.publish_status()
            return
        if self.logic.phase is Phase.DWELL and self._deadline_elapsed(now):
            self.logic.finish_dwell()
            self.phase_deadline = None
            self.last_status_detail = 'dwell complete; reverse request is ready'
            self.publish_status()
            return
        waiting_for_setup = self.logic.phase in (
            Phase.WAITING_PLAN, Phase.WAITING_ROUTE)
        if waiting_for_setup and self._deadline_elapsed(now):
            stage = (
                'Dijkstra response'
                if self.logic.phase is Phase.WAITING_PLAN
                else 'controller route')
            self.logic.fail(f'timed out waiting for {stage}')
            self._report_error()
            return
        if self.logic.phase is Phase.WAITING_ARRIVAL and self._deadline_elapsed(now):
            # A timeout never advances to the next pair: explicit FINISHED remains mandatory.
            self.logic.fail('timed out waiting for explicit controller FINISHED signal')
            self._report_error()

    def _deadline_elapsed(self, now: float) -> bool:
        return self.phase_deadline is not None and now >= self.phase_deadline

    def _controller_allows_first_request(self) -> bool:
        if self.last_controller_status is None:
            return False
        state = str(self.last_controller_status.get('state', ''))
        active = bool(self.last_controller_status.get('active', False))
        return not active and state in ('IDLE', 'FINISHED', 'CANCELLED')

    def _report_error(self) -> None:
        self.phase_deadline = None
        self.last_status_detail = self.logic.error
        self.get_logger().error(self.last_status_detail)
        self.publish_status()

    def publish_status(self) -> None:
        pair = self.logic.current_pair()
        status = {
            'phase': self.logic.phase.value,
            'detail': self.last_status_detail,
            'configured_start_id': self.logic.start_id,
            'configured_goal_id': self.logic.goal_id,
            'current_start_id': pair[0] if pair else None,
            'current_goal_id': pair[1] if pair else None,
            'active_route_sequence': self.logic.active_route_sequence,
            'completed_legs': self.logic.completed_legs,
            'completed_round_trips': self.logic.completed_round_trips,
            'max_round_trips': self.logic.max_round_trips,
            'arrival_requires_finished': True,
            'request_mode': self.request_mode,
            'wait_for_idle_before_first_request': self.wait_for_idle_before_first_request,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LoopPatrolNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
