"""Final software command arbiter: /cmd_vel -> /cmd_vel_safe."""
import json
import math
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String, Bool
from .safety import VelocityGate

class SafetyNode(Node):

    def __init__(self):
        super().__init__('velocity_gate')
        self.gate = VelocityGate()
        self.scan_points = []
        self.depth_points = []
        self.inhibit_seen = 0.0
        self.inhibit_value = True
        self.output = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.status = self.create_publisher(String, '/safety/status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/perception/obstacles', self.cloud, qos_profile_sensor_data)
        self.create_subscription(String, '/perception/status', self.depth_health, 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(Bool, '/mission/inhibit', self.inhibit, 10)
        self.create_timer(0.05, self.tick)

    def fresh(self, stamp):
        age = self.get_clock().now().nanoseconds / 1000000000.0 - (stamp.sec + stamp.nanosec / 1000000000.0)
        return -0.05 <= age <= self.gate.config.sensor_timeout

    def command(self, msg):
        try:
            self.gate.request(msg.linear.x, msg.angular.z, time.monotonic())
        except ValueError:
            self.gate.command_stamp = -math.inf

    def inhibit(self, msg):
        self.inhibit_value = msg.data
        self.inhibit_seen = time.monotonic()

    def odom(self, msg):
        if self.fresh(msg.header.stamp):
            self.gate.observe('odom', time.monotonic())

    def scan(self, msg):
        if not self.fresh(msg.header.stamp) or msg.header.frame_id != 'base_link':
            return
        self.scan_points = [(r * math.cos(msg.angle_min + i * msg.angle_increment), r * math.sin(msg.angle_min + i * msg.angle_increment)) for i, r in enumerate(msg.ranges) if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if self.scan_points:
            self.gate.observe('scan', time.monotonic())

    def cloud(self, msg):
        if not self.fresh(msg.header.stamp) or msg.header.frame_id != 'base_link' or msg.point_step != 12 or (len(msg.data) != msg.row_step * msg.height):
            return
        self.depth_points = [(x, y) for x, y, z in struct.iter_unpack('<fff', bytes(msg.data)) if all((math.isfinite(v) for v in (x, y, z)))]

    def depth_health(self, msg):
        data = json.loads(msg.data)
        age = self.get_clock().now().nanoseconds / 1000000000.0 - data.get('stamp_s', 0.0)
        if data.get('healthy') and -0.05 <= age <= self.gate.config.sensor_timeout:
            self.gate.observe('depth', time.monotonic())
        else:
            self.gate.stamps.pop('depth', None)

    def tick(self):
        self.gate.points = self.scan_points + self.depth_points
        self.gate.inhibit = self.inhibit_value or time.monotonic() - self.inhibit_seen > 0.5
        result = self.gate.evaluate(time.monotonic())
        out = Twist()
        out.linear.x = result['linear']
        out.angular.z = result['angular']
        self.output.publish(out)
        self.status.publish(String(data=json.dumps(result)))

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
