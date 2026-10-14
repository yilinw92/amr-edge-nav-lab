"""ROS adapter for event-sourced waypoint execution and bounded recovery."""
import json
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from .supervisor import RouteSupervisor

class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_manager')
        for name, value in {'goal_x': 6.85, 'goal_y': 4.85, 'timeout_s': 120.0, 'waypoints': [1.85, 1.05, 1.85, 2.85]}.items():
            self.declare_parameter(name, value)
        self.mission = RouteSupervisor(timeout=self.get_parameter('timeout_s').value)
        self.goal_handle = None
        self.cancel_sent = False
        self.distance = None
        self.sensor_healthy = False
        self.health_seen = 0.0
        self.lifecycle_seen = 0.0
        self.safety_detail = {}
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.navigator_active = False
        self.lifecycle_future = None
        self.pub = self.create_publisher(String, '/mission/status', 10)
        self.events = self.create_publisher(String, '/mission/events', 10)
        self.inhibit = self.create_publisher(Bool, '/mission/inhibit', 10)
        self.emitted = 0
        self.create_subscription(String, '/safety/status', self.health, 10)
        self.create_service(Trigger, '/mission/start', self.start)
        self.create_service(Trigger, '/mission/start_route', self.start_route)
        self.create_service(Trigger, '/mission/cancel', self.cancel)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.check_activation)

    def health(self, msg):
        self.safety_detail = json.loads(msg.data)
        self.sensor_healthy = self.safety_detail['sensor_healthy']
        self.health_seen = time.monotonic()

    def check_activation(self):
        if self.lifecycle.service_is_ready() and (self.lifecycle_future is None or self.lifecycle_future.done()):
            self.lifecycle_future = self.lifecycle.call_async(GetState.Request())
            self.lifecycle_future.add_done_callback(self.activation_result)

    def activation_result(self, future):
        try:
            self.navigator_active = future.result().current_state.id == 3
            self.lifecycle_seen = time.monotonic()
        except Exception:
            self.navigator_active = False

    def ready(self):
        return self.client.server_is_ready() and self.navigator_active and (time.monotonic() - self.lifecycle_seen < 3.0) and self.sensor_healthy and (time.monotonic() - self.health_seen < 0.5)

    def begin(self, route, response):
        if not self.ready():
            response.success = False
            response.message = 'Waiting for active Nav2 and fresh scan/depth/odometry.'
            return response
        try:
            self.mission.begin(route, time.monotonic())
            response.success = True
            response.message = 'Route queued; inspect /mission/events and /mission/status.'
        except (ValueError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
        return response

    def start(self, request, response):
        return self.begin([(self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)], response)

    def start_route(self, request, response):
        flat = self.get_parameter('waypoints').value
        if len(flat) % 2:
            response.success = False
            response.message = 'waypoints must contain XY pairs'
            return response
        return self.begin(list(zip(flat[::2], flat[1::2])), response)

    def cancel(self, request, response):
        response.success = self.mission.cancel('operator', time.monotonic())
        response.message = 'Await final action result; operator cancellation never auto-resumes.'
        return response

    def dispatch(self, token, xy):
        self.goal_handle = None
        self.cancel_sent = False
        self.distance = None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(xy[0])
        goal.pose.pose.position.y = float(xy[1])
        goal.pose.pose.orientation.w = 1.0
        future = self.client.send_goal_async(goal, feedback_callback=lambda msg: self.feedback(token, msg))
        future.add_done_callback(lambda f: self.accepted(token, f))

    def accepted(self, token, future):
        try:
            handle = future.result()
            self.mission.accepted(token, handle.accepted, time.monotonic())
            if token == self.mission.token:
                self.goal_handle = handle
                if handle.accepted:
                    handle.get_result_async().add_done_callback(lambda f: self.finished(token, f))
        except Exception as error:
            self.mission.state = 'FAULT'
            self.mission.event(time.monotonic(), 'ACCEPTANCE_TRANSPORT_ERROR', error=str(error))

    def feedback(self, token, message):
        if token == self.mission.token:
            self.distance = float(message.feedback.distance_remaining)

    def finished(self, token, future):
        try:
            status = future.result().status
            outcome = 'succeeded' if status == GoalStatus.STATUS_SUCCEEDED else 'canceled' if status == GoalStatus.STATUS_CANCELED else 'failed'
            self.mission.result(token, outcome, time.monotonic())
        except Exception as error:
            self.mission.state = 'FAULT'
            self.mission.event(time.monotonic(), 'RESULT_TRANSPORT_ERROR', error=str(error))

    def tick(self):
        previous = self.mission.state
        now = time.monotonic()
        unhealthy_reason = 'health_heartbeat_stale' if now - self.health_seen > 0.5 else 'sensor_stale' if not self.sensor_healthy else 'navigator_unavailable'
        command = self.mission.step(self.ready(), now, unhealthy_reason)
        if self.mission.state == 'CANCELING' and previous != 'CANCELING':
            self.mission.event(now, 'READINESS_AT_CANCEL', navigator_active=self.navigator_active, health_age=now - self.health_seen, lifecycle_age=now - self.lifecycle_seen, safety=self.safety_detail)
        if command:
            self.dispatch(*command)
        if self.mission.state == 'CANCELING' and self.goal_handle and self.goal_handle.accepted and (not self.cancel_sent):
            self.cancel_sent = True
            self.goal_handle.cancel_goal_async().add_done_callback(self.cancel_reply)
        self.inhibit.publish(Bool(data=self.mission.state in {'CANCELING', 'PAUSED', 'FAULT'}))
        self.pub.publish(String(data=json.dumps({'state': self.mission.state, 'waypoint_index': self.mission.index, 'waypoints': len(self.mission.route), 'token': self.mission.token, 'retry': self.mission.retry, 'pauses': self.mission.pauses, 'distance_remaining_m': self.distance, 'navigator_active': self.navigator_active, 'sensor_healthy': self.sensor_healthy, 'events': len(self.mission.events)})))
        for event in self.mission.events[self.emitted:]:
            self.events.publish(String(data=json.dumps(event)))
        self.emitted = len(self.mission.events)

    def cancel_reply(self, future):
        try:
            self.mission.event(time.monotonic(), 'CANCEL_REPLY', accepted=bool(future.result().goals_canceling))
        except Exception as error:
            self.mission.event(time.monotonic(), 'CANCEL_TRANSPORT_ERROR', error=str(error))

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
