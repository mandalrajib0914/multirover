"""Shared part of the victim detectors (ArUco S7, YOLO S7b): hits -> estimate -> topics.

A detector subclasses VictimNode, turns its detections into world positions, and calls
  add_candidate(xyz, yaw, stamp, robot, rng)   every sighting up to candidate_range
  add_hit(xyz, stamp, robot, rng)              sightings within max_range (accurate enough)
VictimNode keeps the running median of the hits and publishes (same for every detector, so
the coordinator, monitor and analysis don't care which one runs):
  /victim/pose        geometry_msgs/PoseStamped  the median, on every hit
  /victim/confirmed   geometry_msgs/PointStamped latched, once confirm_hits hits
  /victim/candidate   geometry_msgs/PoseStamped  every sighting until confirmed; yaw = the
                      direction to look at the victim from (where it faces / where it was seen)
  /victim/markers     visualization_msgs/MarkerArray  victim cube + label
"""
import math
import threading

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_to_rt(tf):
    """(R 3x3, t 3) of a geometry_msgs TransformStamped."""
    tr = tf.transform
    return quat_to_matrix(tr.rotation), np.array([tr.translation.x, tr.translation.y,
                                                  tr.translation.z])


class VictimNode(Node):
    def __init__(self, name, label):
        super().__init__(name)
        p = self.declare_parameter
        self.robots = list(p('robots', ['robot1']).value)
        self.max_range = p('max_range', 2.0).value
        self.candidate_range = p('candidate_range', 4.0).value
        self.confirm_hits = p('confirm_hits', 3).value
        self.world = p('world_frame', 'map').value
        self.max_tf_lag = p('max_tf_lag', 0.1).value
        self.label = label
        self.info = {}              # robot -> (K, D) from camera_info
        self.hits = []              # [(x, y, z, robot, range, stamp_s)]
        self.confirmed = False
        self.tf = Buffer()
        # TF on its own node and thread: on the detector's executor, slow image callbacks (a
        # throttled CPU on battery, or YOLO) starved TF and ~95 % of detections were dropped as
        # "no TF" (image up to 2.2 s newer than the latest transform). rclpy's
        # TransformListener(spin_thread=True) doesn't help: it adds *this* node to a second
        # executor, and the two then steal each other's callbacks.
        self._tf_node = rclpy.create_node(f'{name}_tf', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=self.get_parameter('use_sim_time').value)])
        self._tf_listener = TransformListener(self.tf, self._tf_node)
        self._tf_executor = SingleThreadedExecutor()
        self._tf_executor.add_node(self._tf_node)
        threading.Thread(target=self._tf_executor.spin, daemon=True).start()
        for r in self.robots:
            self.create_subscription(CameraInfo, f'{self.ns(r)}camera/camera_info',
                                     lambda m, r=r: self.on_info(r, m), qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, '/victim/pose', 10)
        self.confirmed_pub = self.create_publisher(PointStamped, '/victim/confirmed', latched)
        self.candidate_pub = self.create_publisher(PoseStamped, '/victim/candidate', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/victim/markers', latched)

    @staticmethod
    def ns(robot):
        return f'/{robot}/' if robot else '/'

    def on_info(self, robot, msg):
        if robot not in self.info:
            self.info[robot] = (np.array(msg.k, np.float64).reshape(3, 3),
                                np.array(msg.d, np.float64) if len(msg.d) else np.zeros(5))

    def lookup(self, frame, stamp):
        """TF world <- frame at the image time. Images can be a frame (~30 ms) newer than the
        latest TF (seen in sim: "extrapolation into the future"), so fall back to the latest
        transform if it is at most max_tf_lag older than the image."""
        return self.lookup_between(self.world, frame, stamp)

    def lookup_between(self, target, frame, stamp):
        try:
            return self.tf.lookup_transform(target, frame, stamp, timeout=Duration(seconds=0.05))
        except TransformException:
            pass
        try:
            tf = self.tf.lookup_transform(target, frame, Time())
        except TransformException:
            return None
        lag = (stamp - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        return tf if lag <= self.max_tf_lag else None

    def add_candidate(self, w, yaw, stamp, robot, rng):
        """/victim/candidate: where the victim probably is and the direction to look at it
        from. The coordinator sends a robot there to look close up."""
        if self.confirmed:
            return
        c = PoseStamped()
        c.header.frame_id, c.header.stamp = self.world, stamp
        c.pose.position.x, c.pose.position.y, c.pose.position.z = (float(v) for v in w)
        c.pose.orientation.z, c.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.candidate_pub.publish(c)
        self.get_logger().info(f'{robot}: sighting at ({w[0]:.2f}, {w[1]:.2f}), {rng:.2f} m away',
                               throttle_duration_sec=2.0)

    def add_hit(self, w, stamp, robot, rng):
        self.hits.append((*w, robot, rng, Time.from_msg(stamp).nanoseconds * 1e-9))
        self.publish(stamp)

    def estimate(self):
        a = np.array([h[:3] for h in self.hits])
        return np.median(a, axis=0)

    def publish(self, stamp):
        x, y, z = self.estimate()
        n = len(self.hits)
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = self.world, stamp
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
        ps.pose.orientation.w = 1.0
        self.pose_pub.publish(ps)
        if not self.confirmed and n >= self.confirm_hits:
            self.confirmed = True
            pt = PointStamped()
            pt.header = ps.header
            pt.point = ps.pose.position
            self.confirmed_pub.publish(pt)
            robots = sorted({h[3] for h in self.hits})
            self.get_logger().info(f'VICTIM CONFIRMED ({self.label}) at ({x:.3f}, {y:.3f}) after '
                                   f'{n} detections by {robots}, ranges '
                                   f'{min(h[4] for h in self.hits):.2f}-{max(h[4] for h in self.hits):.2f} m')
        ma = MarkerArray()
        cube = Marker(type=Marker.CUBE, ns='victim', id=0, action=Marker.ADD)
        cube.header = ps.header
        cube.pose = ps.pose
        cube.scale.x = cube.scale.y = cube.scale.z = 0.2
        cube.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9 if self.confirmed else 0.4)
        label = Marker(type=Marker.TEXT_VIEW_FACING, ns='victim', id=1, action=Marker.ADD)
        label.header = ps.header
        label.pose.position.x, label.pose.position.y, label.pose.position.z = float(x), float(y), 0.45
        label.pose.orientation.w = 1.0
        label.scale.z = 0.15
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        label.text = f'victim ({n} hits){"" if self.confirmed else "?"}'
        ma.markers = [cube, label]
        self.marker_pub.publish(ma)
