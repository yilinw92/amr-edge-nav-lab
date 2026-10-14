"""Small deterministic ROS 2 kinematic simulator, not a Gazebo replacement."""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from .core import raycast, warehouse


class SimWorld(Node):
    def __init__(self):
        super().__init__("synthetic_world")
        self.grid = warehouse()
        self.collision_grid = self.grid.inflated(0.2)
        self.x, self.y, self.yaw = 1.05, 1.05, 0.0
        self.v, self.w = 0.0, 0.0
        self.last_command = self.get_clock().now()
        self.blocker = False
        self.blocked_cells = [(x, y) for x in range(36, 43) for y in range(25, 33)]
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "camera/depth/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "camera/depth/camera_info", qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self.command, 10)
        self.create_service(SetBool, "demo/toggle_blocker", self.toggle)
        self.create_timer(0.05, self.tick)
        self.sensor_ticks = 0
        self.get_logger().info("Synthetic simulator: ideal wheel odometry; AMCL receives map + laser scan.")

    def command(self, msg):
        self.v = max(-0.2, min(0.4, msg.linear.x))
        self.w = max(-1.5, min(1.5, msg.angular.z))
        self.last_command = self.get_clock().now()

    def toggle(self, request, response):
        self.blocker = request.data
        response.success, response.message = True, f"dynamic blocker={self.blocker}"
        return response

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        if (self.get_clock().now() - self.last_command).nanoseconds > 500_000_000:
            self.v, self.w = 0.0, 0.0
        obstacles = self.blocked_cells if self.blocker else []
        collision = self.collision_grid if not obstacles else self.grid.with_obstacles(obstacles).inflated(0.2)
        yaw = self.yaw + self.w * 0.05
        x, y = self.x + self.v * math.cos(yaw) * 0.05, self.y + self.v * math.sin(yaw) * 0.05
        moving = self.grid.cell(x, y) not in collision.occupied and self.grid.inside(self.grid.cell(x, y))
        if moving:
            self.x, self.y = x, y
        self.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
        self.tf.sendTransform(tf)
        odom = Odometry()
        odom.header, odom.child_frame_id = tf.header, "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v if moving else 0.0, self.w
        self.odom_pub.publish(odom)
        self.sensor_ticks += 1
        if self.sensor_ticks % 2:
            return
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, "base_link"
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2 * math.pi / 180
        scan.range_min, scan.range_max, scan.scan_time = 0.05, 5.0, 0.1
        scan.ranges = [raycast(self.grid, self.x, self.y, self.yaw + scan.angle_min + i*scan.angle_increment, extra=obstacles) for i in range(181)]
        self.scan_pub.publish(scan)
        width, height, fx = 64, 24, 32.0
        # Simplified extruded-wall depth. Pinhole Z = planar ray range * cos(angle).
        row = []
        for u in range(width):
            angle = -math.atan((u - 31.5) / fx)
            row.append(raycast(self.grid, self.x, self.y, self.yaw + angle, extra=obstacles) * math.cos(angle))
        image = Image()
        image.header.stamp, image.header.frame_id = stamp, "camera_optical_frame"
        image.width, image.height, image.encoding, image.step = width, height, "32FC1", width * 4
        image.is_bigendian, image.data = 0, struct.pack("<" + "f"*(width*height), *(row*height))
        info = CameraInfo()
        info.header, info.width, info.height = image.header, width, height
        info.k = [fx, 0.0, 31.5, 0.0, fx, 11.5, 0.0, 0.0, 1.0]
        self.info_pub.publish(info)
        self.depth_pub.publish(image)


def main(args=None):
    rclpy.init(args=args)
    node = SimWorld()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
