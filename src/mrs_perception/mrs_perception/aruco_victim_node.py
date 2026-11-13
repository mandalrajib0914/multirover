"""Find the victim's ArUco marker in every robot's camera and place it on the world map (S7).

  ros2 run mrs_perception aruco_victim_node --ros-args --params-file config/aruco.yaml \
      -p robots:="['robot1','robot2']"

One node on the base station for all robots (on hardware the images come over Wi-Fi).
For each robotN it subscribes /robotN/<image_topic> and /robotN/camera/camera_info, and per
image:
  1. detects markers (OpenCV aruco; the old detectMarkers API on OpenCV 4.6, ArucoDetector on
     4.7+)
  2. solvePnPGeneric (IPPE_SQUARE) on the 4 corners -> both candidate marker poses in the
     camera optical frame. A marker 20-40 px wide has two near-equal solutions (planar pose
     ambiguity); in an offline test IPPE's first pick tilted the normal 16 deg up/down while the
     marker was really turned 35 deg sideways. The marker is on a vertical face, so the pose
     whose normal is most horizontal in the world frame is used
  3. victim centre = marker centre moved `marker_to_center` m behind the marker, along the
     horizontal part of that normal
  4. sightings up to `candidate_range` go out on /victim/candidate (so the coordinator can
     send a robot to look close up); only those within `max_range` count as detections
  5. transforms it into `world_frame` with TF at the image time
The victim estimate is the running median of all accepted detections (all robots);
victim_base.VictimNode does that part and the publishing, shared with the YOLO detector.
Publishes:
  /victim/pose        geometry_msgs/PoseStamped  the median, on every accepted detection
  /victim/confirmed   geometry_msgs/PointStamped latched, once `confirm_hits` detections
  /victim/candidate   geometry_msgs/PoseStamped  every sighting until confirmed; yaw = the
                      direction the marker faces
  /victim/markers     visualization_msgs/MarkerArray  victim cube + label
"""

import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image

from mrs_perception.victim_base import quat_to_matrix, VictimNode


def make_detector(dict_name):
    """Return detect(gray) -> (corners, ids) for either OpenCV aruco API. Corners are
    refined to subpixel: a small marker's corners are otherwise biased inwards by blur, which
    over-estimates the range (+6 % at 2.5 m in the offline test)."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    if hasattr(cv2.aruco, 'ArucoDetector'):   # OpenCV >= 4.7
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        det = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda gray: det.detectMarkers(gray)[:2]
    params = cv2.aruco.DetectorParameters_create()   # OpenCV 4.6 (Ubuntu 24.04)
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)[:2]


def marker_poses(corners, size, K, D):
    """Both IPPE solutions for one marker's corners (TL, TR, BR, BL), as [(R, t)] in the camera
    frame: t = marker centre, R[:, 2] = marker normal, pointing out of the marker (towards
    the camera)."""
    h = size / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, corners.reshape(4, 2).astype(np.float64), K, D,
                                             flags=cv2.SOLVEPNP_IPPE_SQUARE)
    return [(cv2.Rodrigues(r)[0], t.ravel()) for r, t in zip(rvecs[:n], tvecs[:n])]


def victim_centre(poses, Rw, tw, to_center):
    """World position of the victim centre from the candidate marker poses.

    Rw, tw: camera -> world rotation and translation. Picks the pose whose normal is most
    horizontal in the world (the marker is on a vertical face) and steps `to_center` behind
    the marker along the normal's horizontal part. Returns (centre_world, normal_world)."""
    best = min(poses, key=lambda p: abs((Rw @ p[0][:, 2])[2]))
    R, t = best
    marker_w = Rw @ t + tw
    n = Rw @ R[:, 2]
    cam_w = tw
    if n @ (cam_w - marker_w) < 0:   # make it point out of the marker, towards the camera
        n = -n
    nh = np.array([n[0], n[1], 0.0])
    nh /= max(np.linalg.norm(nh), 1e-9)
    return marker_w - nh * to_center, nh


class ArucoVictimNode(VictimNode):
    def __init__(self):
        super().__init__('aruco_victim_node', 'aruco')
        p = self.declare_parameter
        image_topic = p('image_topic', 'camera/image_raw').value
        self.marker_id = p('marker_id', 0).value
        self.size = p('marker_size', 0.10).value
        self.to_center = p('marker_to_center', 0.10).value
        self.detections = 0   # raw marker detections before range/TF checks (for diagnostics)
        self.detect = make_detector(p('dictionary', 'DICT_4X4_50').value)
        self.bridge = CvBridge()
        for r in self.robots:
            self.create_subscription(Image, f'{self.ns(r)}{image_topic}',
                                     lambda m, r=r: self.on_image(r, m), qos_profile_sensor_data)
        self.get_logger().info(f'watching {self.robots} for ArUco id {self.marker_id} '
                               f'({self.size} m), confirm after {self.confirm_hits} hits')

    def on_image(self, robot, msg):
        if robot not in self.info:
            return
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        corners, ids = self.detect(gray)
        if ids is None:
            return
        self.detections += 1
        K, D = self.info[robot]
        for c, i in zip(corners, ids.ravel()):
            if self.marker_id >= 0 and i != self.marker_id:
                continue
            poses = marker_poses(c, self.size, K, D)
            if not poses:
                continue
            rng = float(np.linalg.norm(poses[0][1]))
            if rng > self.candidate_range:
                continue
            tf = self.lookup(msg.header.frame_id, Time.from_msg(msg.header.stamp))
            if tf is None:
                self.get_logger().warn(f'{robot}: no TF for a detection', throttle_duration_sec=5.0)
                continue
            tr = tf.transform
            w, nh = victim_centre(poses, quat_to_matrix(tr.rotation),
                                  np.array([tr.translation.x, tr.translation.y, tr.translation.z]),
                                  self.to_center)
            # look at it from where the marker faces
            self.add_candidate(w, math.atan2(nh[1], nh[0]), msg.header.stamp, robot, rng)
            if rng > self.max_range:
                continue   # a sighting to investigate, not accurate enough to count
            self.add_hit(w, msg.header.stamp, robot, rng)


def main():
    rclpy.init()
    node = ArucoVictimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
