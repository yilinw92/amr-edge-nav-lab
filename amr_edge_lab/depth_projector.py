"""Project depth images into a filtered point cloud for Nav2 obstacle layers."""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

from .core import project_depth


class DepthProjector(Node):
    def __init__(self):
        super().__init__("depth_projector")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("info_topic", "/camera/depth/camera_info")
        self.info = None
        self.create_subscription(CameraInfo, self.get_parameter("info_topic").value, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("depth_topic").value, self.on_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, "/perception/obstacles", qos_profile_sensor_data)

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, msg):
        if self.info is None:
            return
        if msg.encoding != "32FC1" or msg.width != self.info.width or msg.height != self.info.height:
            self.get_logger().warning("Need matching CameraInfo and 32FC1 depth in metres.")
            return
        if msg.step < msg.width*4 or len(msg.data) < msg.step*msg.height:
            self.get_logger().warning("Malformed depth payload.")
            return
        # Honour row padding and endianness rather than assuming contiguous pixels.
        fmt = (">" if msg.is_bigendian else "<") + "f"*msg.width
        values = []
        for row in range(msg.height):
            values.extend(struct.unpack_from(fmt, bytes(msg.data), row*msg.step))
        k = self.info.k
        try:
            points = project_depth(values, msg.width, msg.height, k[0], k[4], k[2], k[5])
        except ValueError as error:
            self.get_logger().warning(str(error)); return
        points = [p for p in points if 0.08 <= p[2] <= 1.2]
        cloud = PointCloud2()
        cloud.header = Header(stamp=msg.header.stamp, frame_id="base_link")
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, name in enumerate("xyz")]
        cloud.is_bigendian, cloud.point_step, cloud.row_step = False, 12, len(points)*12
        cloud.data = b"".join(struct.pack("<fff", *p) for p in points)
        cloud.is_dense = True
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProjector()
    try: rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()
