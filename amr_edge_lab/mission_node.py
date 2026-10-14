"""NavigateToPose action client: feedback, cancellation, timeout and final result."""
import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import Mission, State


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_manager")
        self.declare_parameter("goal_x", 6.85)
        self.declare_parameter("goal_y", 4.85)
        self.declare_parameter("timeout_s", 120.0)
        self.mission = Mission()
        self.goal_handle = None
        self.cancel_sent = False
        self.started = None
        self.distance = None
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.lifecycle = self.create_client(GetState, "/bt_navigator/get_state")
        self.navigator_active = False
        self.lifecycle_future = None
        self.pub = self.create_publisher(String, "mission/status", 10)
        self.create_service(Trigger, "mission/start", self.start)
        self.create_service(Trigger, "mission/cancel", self.cancel)
        self.create_timer(0.25, self.tick)
        self.create_timer(1.0, self.check_activation)

    def check_activation(self):
        if self.lifecycle.service_is_ready() and (self.lifecycle_future is None or self.lifecycle_future.done()):
            self.lifecycle_future = self.lifecycle.call_async(GetState.Request())
            self.lifecycle_future.add_done_callback(self.activation_result)

    def activation_result(self, future):
        try:
            self.navigator_active = future.result().current_state.id == 3
        except Exception:
            self.navigator_active = False

    def start(self, request, response):
        if self.mission.active:
            response.success, response.message = False, "Existing goal must reach a final result."
            return response
        if not self.client.server_is_ready() or not self.navigator_active:
            response.success, response.message = False, "Nav2 action server is not ready; retry after activation."
            return response
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = "map", self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.get_parameter("goal_x").value)
        pose.pose.position.y = float(self.get_parameter("goal_y").value)
        pose.pose.orientation.w = 1.0
        goal.pose = pose
        self.mission.start()
        self.goal_handle, self.cancel_sent, self.distance = None, False, None
        self.started = self.get_clock().now()
        self.client.send_goal_async(goal, feedback_callback=self.feedback).add_done_callback(self.accepted)
        response.success, response.message = True, "Goal submitted; inspect /mission/status for acceptance and result."
        return response

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            self.mission.accepted(self.goal_handle.accepted)
            if self.goal_handle.accepted:
                self.goal_handle.get_result_async().add_done_callback(self.finished)
                self.send_cancel_if_needed()
        except Exception as error:
            # A transport error does not prove the server stopped the robot.
            self.get_logger().error(f"Action acceptance error: {error}; inspect Nav2 before restarting.")

    def feedback(self, message):
        self.distance = float(message.feedback.distance_remaining)

    def cancel(self, request, response):
        response.success = self.mission.request_cancel()
        response.message = "Cancellation requested; awaiting action result." if response.success else "No active goal."
        self.send_cancel_if_needed()
        return response

    def send_cancel_if_needed(self):
        if self.mission.cancel_requested and self.goal_handle and self.goal_handle.accepted and not self.cancel_sent:
            self.cancel_sent = True
            self.goal_handle.cancel_goal_async().add_done_callback(self.cancel_reply)

    def cancel_reply(self, future):
        try:
            if not future.result().goals_canceling:
                self.get_logger().warning("Server did not accept cancellation; await final result.")
        except Exception as error:
            self.get_logger().error(f"Cancel transport error: {error}")

    def finished(self, future):
        try:
            status = future.result().status
            outcome = "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "canceled" if status == GoalStatus.STATUS_CANCELED else "failed"
            self.mission.finished(outcome)
        except Exception as error:
            self.get_logger().error(f"Result transport error: {error}; state retained for inspection.")

    def tick(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9 if self.started else 0.0
        if self.mission.active and elapsed > self.get_parameter("timeout_s").value:
            self.mission.request_cancel()
            self.send_cancel_if_needed()
        msg = String()
        msg.data = json.dumps({"state": self.mission.state.value, "elapsed_s": round(elapsed, 2),
                               "distance_remaining_m": self.distance, "cancel_requested": self.mission.cancel_requested,
                               "navigator_active": self.navigator_active})
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
