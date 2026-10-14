"""Calibrated depth adapter with bounded latency, filtering and diagnostics."""
import json
import struct
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header, String
from .perception import Calibration, decode_depth, process_depth, cluster_obstacles
try:
    from .perception_numpy import process_depth_numpy
except ImportError:
    process_depth_numpy = None

class DepthProjector(Node):

    def __init__(self):
        super().__init__('depth_projector')
        for name, value in {'depth_topic': '/camera/depth/image_raw', 'info_topic': '/camera/depth/camera_info', 'translation': [0.0, 0.0, 0.35], 'rpy': [0.0, 0.0, 0.0], 'voxel_m': 0.08, 'stride': 2, 'min_points': 1, 'max_age_s': 0.5, 'info_skew_s': 0.25}.items():
            self.declare_parameter(name, value)
        self.info = None
        self.sequence = 0
        self.processor = process_depth_numpy or process_depth
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.on_depth, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, '/perception/obstacles', qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(String, '/perception/status', 10)

    def on_info(self, msg):
        self.info = msg

    def seconds(self, stamp):
        return stamp.sec + stamp.nanosec / 1000000000.0

    def on_depth(self, msg):
        started = time.perf_counter()
        self.sequence += 1
        try:
            if self.info is None:
                raise ValueError('missing CameraInfo')
            age = self.get_clock().now().nanoseconds / 1000000000.0 - self.seconds(msg.header.stamp)
            if age < -0.05 or age > self.get_parameter('max_age_s').value:
                raise ValueError('depth timestamp outside freshness window')
            if abs(self.seconds(msg.header.stamp) - self.seconds(self.info.header.stamp)) > self.get_parameter('info_skew_s').value:
                raise ValueError('CameraInfo/depth timestamp skew')
            if msg.encoding != '32FC1' or msg.width != self.info.width or msg.height != self.info.height or (msg.header.frame_id != self.info.header.frame_id):
                raise ValueError('depth/calibration encoding, dimensions or frame mismatch')
            values = decode_depth(bytes(msg.data), msg.width, msg.height, msg.step, bool(msg.is_bigendian))
            k = self.info.k
            calibration = Calibration(k[0], k[4], k[2], k[5], tuple(self.get_parameter('translation').value), tuple(self.get_parameter('rpy').value))
            points, stats = self.processor(values, msg.width, msg.height, calibration, stride=self.get_parameter('stride').value, voxel=self.get_parameter('voxel_m').value, min_points=self.get_parameter('min_points').value)
            if stats['valid_ratio'] < 0.1:
                raise ValueError('insufficient valid depth coverage')
            cloud = PointCloud2(header=Header(stamp=msg.header.stamp, frame_id='base_link'), height=1, width=len(points))
            cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1) for i, name in enumerate('xyz')]
            cloud.is_bigendian, cloud.point_step, cloud.row_step = (False, 12, len(points) * 12)
            cloud.data = b''.join((struct.pack('<fff', *p) for p in points))
            cloud.is_dense = True
            self.pub.publish(cloud)
            report = {'healthy': True, 'backend': 'numpy' if process_depth_numpy else 'portable_python', 'stamp_s': self.seconds(msg.header.stamp), 'image_age_s': round(age, 4), **stats, 'clusters': cluster_obstacles(points)}
        except (ValueError, struct.error) as error:
            report = {'healthy': False, 'reason': str(error)}
        report.update(sequence=self.sequence, processing_ms=round((time.perf_counter() - started) * 1000, 3))
        self.diagnostics.publish(String(data=json.dumps(report)))

def main(args=None):
    rclpy.init(args=args)
    node = DepthProjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
