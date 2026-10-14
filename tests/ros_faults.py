"""Live fault injection: moving robot loses depth, stops, cancels and resumes."""
import json
import math
import statistics
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from ros_smoke import wait

def main():
    rclpy.init();node=Node("portfolio_fault_injection")
    state={"mission":{},"safety":{},"velocity":(0,0),"position":(0,0)};events=[];frames=[];processing=[]
    out=Path("artifacts/ros");out.mkdir(parents=True,exist_ok=True);event_path=out/"mission_events.jsonl";event_path.write_text("")
    def status(msg):state["mission"]=json.loads(msg.data)
    def safety(msg):state["safety"]=json.loads(msg.data);frames.append({"t":time.monotonic(),"state":state["mission"].get("state"),**state["safety"],"position":state["position"]})
    def velocity(msg):state["velocity"]=(msg.linear.x,msg.angular.z)
    def odom(msg):state["position"]=(msg.pose.pose.position.x,msg.pose.pose.position.y)
    def event(msg):
        events.append(json.loads(msg.data))
        with event_path.open("a") as handle:handle.write(msg.data+"\n")
    def perception(msg):
        d=json.loads(msg.data)
        if d.get("healthy"):processing.append(d["processing_ms"])
    node.create_subscription(String,"/mission/status",status,10);node.create_subscription(String,"/mission/events",event,10)
    node.create_subscription(String,"/safety/status",safety,10);node.create_subscription(String,"/perception/status",perception,10)
    node.create_subscription(Twist,"/cmd_vel_safe",velocity,10);node.create_subscription(Odometry,"/odom",odom,10)
    start=node.create_client(Trigger,"/mission/start_route");cancel=node.create_client(Trigger,"/mission/cancel");depth=node.create_client(SetBool,"/demo/depth_enabled")
    wait(node,lambda:state["mission"].get("navigator_active") and state["safety"].get("sensor_healthy") and start.service_is_ready() and depth.service_is_ready(),75,"fault scenario readiness")
    def call(client,request):
        future=client.call_async(request);wait(node,future.done,5,"service reply");assert future.result().success,future.result().message
    call(start,Trigger.Request())
    wait(node,lambda:state["mission"].get("state")=="EXECUTING" and abs(state["velocity"][0])>0.05,30,"robot moving")
    injected=time.monotonic();call(depth,SetBool.Request(data=False))
    wait(node,lambda:state["safety"].get("reason")=="SENSOR_STALE" and state["velocity"]==(0.0,0.0),2,"sensor watchdog stop")
    stop_latency=time.monotonic()-injected;assert stop_latency<0.9,stop_latency
    wait(node,lambda:state["mission"].get("state")=="PAUSED",15,"confirmed cancel before pause")
    stopped_position=state["position"];hold_until=time.monotonic()+1.0
    wait(node,lambda:time.monotonic()>hold_until,2,"stationary dropout hold")
    drift=math.dist(stopped_position,state["position"]);assert drift<0.03,drift
    call(depth,SetBool.Request(data=True))
    wait(node,lambda:state["mission"].get("state")=="SUCCEEDED",100,"recovered two-waypoint route")
    # Host scheduling stalls are real readiness losses too. Count them explicitly;
    # the bounded supervisor must still confirm each cancellation before retrying.
    assert 1<=state["mission"]["pauses"]<=3,state["mission"]
    recovery_pauses=state["mission"]["pauses"]
    canceled=[e for e in events if e["event"]=="GOAL_RESULT" and e.get("outcome")=="canceled"]
    resumed=[e for e in events if e["event"]=="GOAL_SENT" and e["token"]>1]
    assert canceled and resumed and canceled[0]["seq"]<resumed[0]["seq"],events
    for sent in resumed:
        prior=[e for e in events if e["event"]=="GOAL_RESULT" and e["token"]==sent["token"]-1]
        assert prior and prior[-1]["seq"]<sent["seq"],sent
    # Start another route and verify explicit user cancellation is terminal.
    call(start,Trigger.Request());wait(node,lambda:state["mission"].get("state")=="EXECUTING",15,"operator-cancel route")
    call(cancel,Trigger.Request());wait(node,lambda:state["mission"].get("state")=="CANCELED",15,"operator cancel result")
    token=state["mission"]["token"];hold_until=time.monotonic()+1.5;wait(node,lambda:time.monotonic()>hold_until,3,"no auto-resume after operator cancel")
    assert state["mission"]["state"]=="CANCELED" and state["mission"]["token"]==token
    summary={"mode":"live_ros2_nav2_synthetic_fault_injection","depth_dropout_stop_latency_s":round(stop_latency,4),"stationary_drift_m":round(drift,5),"recovery_route":"SUCCEEDED","confirmed_cancellation_precedes_retry":True,"operator_cancel":"CANCELED_no_auto_resume","waypoints_completed":2,"recovery_pauses":recovery_pauses,"additional_readiness_losses":recovery_pauses-1,"perception_ms_p50":round(statistics.median(processing),3),"perception_ms_p95":round(sorted(processing)[int(0.95*(len(processing)-1))],3),"hardware":"x86_64 Docker / WSL; synthetic sensors"}
    out=Path("artifacts/ros");out.mkdir(parents=True,exist_ok=True)
    (out/"fault_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    (out/"mission_events.jsonl").write_text("".join(json.dumps(e)+"\n" for e in events))
    (out/"safety_trace.json").write_text(json.dumps(frames))
    print(json.dumps(summary));node.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
