from collections import deque
import importlib
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from .mapping import Occupancy, compose, inverse


def planar(tr):
    q = tr.rotation
    return np.array([tr.translation.x, tr.translation.y,
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])


def load_estimator(name):
    modules = {
        'student': 'rawseeds_slam.student_algorithm',
        'demo': 'rawseeds_slam.slam.scan_matcher',
        'odom': 'rawseeds_slam.slam.odometry_baseline',
    }
    if name not in modules:
        raise ValueError('algorithm must be student, demo, or odom')
    module = importlib.import_module(modules[name])
    return module.PoseEstimator()


class SlamMapper(Node):
    def __init__(self):
        super().__init__('slam_mapper')
        algorithm = self.declare_parameter('algorithm', 'student').value
        resolution = float(self.declare_parameter('resolution', .1).value)
        size = int(self.declare_parameter('size', 2400).value)
        self.grid = Occupancy(resolution, size)
        self.algorithm = load_estimator(algorithm)
        self.get_logger().info(f'Using pose estimator: {algorithm}')
        self.pending = deque(maxlen=100)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.correction = np.zeros(3)
        self.last = None
        self.publisher = self.create_publisher(
            OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan, '/scan_front', self.receive, qos_profile_sensor_data)
        self.create_timer(.05, self.process)
        self.create_timer(.05, self.broadcast)
        self.create_timer(5., self.publish_map)

    def receive(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds
        if self.last is None or t - self.last >= 200_000_000:
            self.last = t
            self.pending.append(msg)

    def process(self):
        if not self.pending:
            return
        msg = self.pending[0]
        stamp = Time.from_msg(msg.header.stamp)
        try:
            odom = planar(self.buffer.lookup_transform('odom', 'base_link', stamp).transform)
            extrinsic = planar(self.buffer.lookup_transform('base_link', msg.header.frame_id, stamp).transform)
        except TransformException:
            if self.get_clock().now().nanoseconds - stamp.nanoseconds > 2_000_000_000:
                self.pending.popleft()
                self.get_logger().warning('Dropped scan: no TF within 2 s')
            return
        self.pending.popleft()
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges < 30.)
        points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])))
        pose = np.asarray(self.algorithm.update(points, odom, extrinsic, stamp.nanoseconds / 1e9))
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError('PoseEstimator must return finite shape-(3,) [x,y,yaw]')
        self.correction = compose(pose, inverse(odom))
        self.grid.integrate(compose(pose, extrinsic), ranges, msg.angle_min, msg.angle_increment)

    def broadcast(self):
        tr = TransformStamped()
        tr.header.stamp = self.get_clock().now().to_msg()
        tr.header.frame_id = 'map'
        tr.child_frame_id = 'odom'
        x, y, yaw = map(float, self.correction)
        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.rotation.z = math.sin(yaw / 2)
        tr.transform.rotation.w = math.cos(yaw / 2)
        self.broadcaster.sendTransform(tr)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.grid.resolution
        msg.info.width = self.grid.size
        msg.info.height = self.grid.size
        msg.info.origin.position.x = float(self.grid.origin[0])
        msg.info.origin.position.y = float(self.grid.origin[1])
        msg.info.origin.orientation.w = 1.
        msg.data = self.grid.values().ravel().tolist()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
