"""Live ROS integration check: sensors, AMCL TF, action cancellation and arrival."""
import json
import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from amr_edge_lab.core import warehouse
from amr_edge_lab.offline import HTML


def wait(node, condition, seconds, label):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition(): return
    raise RuntimeError("Timed out: " + label)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--full", action="store_true", help="Run across the warehouse instead of a nearby goal")
    args = parser.parse_args()
    rclpy.init()
    node = Node("portfolio_smoke_check")
    state = {"cloud": False, "mission": "", "ready": False}
    tf = Buffer(); listener = TransformListener(tf, node)
    node.create_subscription(PointCloud2, "/perception/obstacles", lambda msg: state.update(cloud=msg.width > 0), qos_profile_sensor_data)
    def mission_status(msg):
        data = json.loads(msg.data)
        state.update(mission=data["state"], ready=data["navigator_active"])
    node.create_subscription(String, "/mission/status", mission_status, 10)
    samples, planned = [], []
    def odometry(msg):
        p = msg.pose.pose.position
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if not samples or timestamp-samples[-1][0] >= 0.2:
            samples.append((timestamp, p.x, p.y, list(planned)))
    def plan(msg):
        planned[:] = [[p.pose.position.x/0.1-0.5,p.pose.position.y/0.1-0.5] for p in msg.poses]
    node.create_subscription(Odometry, "/odom", odometry, 10)
    node.create_subscription(NavPath, "/plan", plan, 10)
    start = node.create_client(Trigger, "/mission/start")
    cancel = node.create_client(Trigger, "/mission/cancel")
    action = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    wait(node, lambda: state["cloud"] and state["ready"] and action.server_is_ready() and tf.can_transform("map", "base_link", rclpy.time.Time()), 75, "sensor + active navigator + AMCL transform readiness")
    wait(node, lambda: start.service_is_ready() and cancel.service_is_ready(), 10, "mission services")
    started = start.call_async(Trigger.Request())
    wait(node, started.done, 5, "mission start")
    assert started.result().success, started.result().message
    wait(node, lambda: state["mission"] == "EXECUTING", 10, "accepted action")
    canceled = cancel.call_async(Trigger.Request())
    wait(node, canceled.done, 5, "cancel request")
    assert canceled.result().success
    wait(node, lambda: state["mission"] == "CANCELED", 20, "confirmed canceled result")
    # A fresh nearby goal must complete after the previous goal reached a final state.
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id, goal.pose.header.stamp = "map", node.get_clock().now().to_msg()
    destination = (6.85, 4.85) if args.full else (1.85, 1.05)
    goal.pose.pose.position.x, goal.pose.pose.position.y = destination
    goal.pose.pose.orientation.w = 1.0
    sent = action.send_goal_async(goal)
    wait(node, sent.done, 10, "nearby goal acceptance")
    assert sent.result().accepted
    result = sent.result().get_result_async()
    wait(node, result.done, 120 if args.full else 60, "goal arrival")
    assert result.result().status == GoalStatus.STATUS_SUCCEEDED, result.result().status
    evidence = {"mode": "ros2_nav2_kinematic_simulation", "depth_cloud": True, "amcl_map_to_base_tf": True,
                "mission_cancel_result": "CANCELED", "goal_result": "SUCCEEDED", "destination_m": destination,
                "hardware": "synthetic x86_64 ROS 2 Humble simulation",
                "recorded_path_length_m": round(sum(math.dist(a[1:3], b[1:3]) for a, b in zip(samples, samples[1:])),3)}
    Path("artifacts/ros").mkdir(parents=True, exist_ok=True)
    Path("artifacts/ros/smoke.json").write_text(json.dumps(evidence, indent=2)+"\n")
    grid = warehouse()
    frames = [{"tick":i,"position":[x/0.1-0.5,y/0.1-0.5],"path":path,"dynamic":[],"state":"ROS_NAV2_REPLAY"} for i,(_,x,y,path) in enumerate(samples)]
    if frames: frames[-1]["state"]="SUCCEEDED"
    payload = {"width":grid.width,"height":grid.height,"occupied":sorted(grid.occupied),
               "goal":[destination[0]/0.1-0.5,destination[1]/0.1-0.5],"frames":frames,"summary":evidence}
    Path("artifacts/ros/replay.html").write_text(HTML.replace("__DATA__",json.dumps(payload)).replace("__LABEL__","Recorded ROS 2 / Nav2 run · Kinematic simulator and synthetic sensors · No Jetson/ZED hardware"),encoding="utf-8")
    with Path("artifacts/ros/trajectory.csv").open("w",newline="") as handle:
        writer=csv.writer(handle);writer.writerow(["stamp_s","x_m","y_m"])
        writer.writerows((t,x,y) for t,x,y,_ in samples)
    print(json.dumps(evidence))
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__": main()
