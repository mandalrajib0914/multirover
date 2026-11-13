"""Find the victim (a person) with YOLO in every robot's camera; range it with the lidar (S7b).

  ~/Desktop/multirover/.venv/bin/python -m mrs_perception.yolo_victim_node --ros-args \
      --params-file config/yolo.yaml -p robots:="['robot1','robot2']"

Runs from the project venv (ultralytics + torch; ROS comes in via --system-site-packages), so
sim_multi_robot.launch.py starts it with the venv's python, not as a ros2 run executable.
Per robot, images are processed at up to `rate` Hz (YOLO on the GPU, class "person", confidence
>= `conf`). For the best box:
  1. bearing: the ray through the box centre (camera intrinsics), made horizontal in the world
  2. range: the lidar points (latest /robotN/scan, into the world frame with TF) inside the
     central `box_fraction` of the box's horizontal span, as seen from the camera; of those,
     the ones within `cluster_depth` of the nearest (the person, not what is behind it);
     median distance from the camera
  3. check: the person is `person_height` m tall, so the box height gives a second distance
     (fy * h / box_px). If lidar and box distance disagree by more than `size_tolerance`
     (relative), the lidar probably hit something else (a wall, another robot): rejected
  4. victim = camera + (range + `body_offset`) along the ray (the lidar sees the near surface)
Sightings up to `candidate_range` go to /victim/candidate (yaw: back towards the camera, so a
robot investigates from the side it was seen from); within `max_range` they count as hits.
Topics are the same as the ArUco detector's (victim_base.VictimNode).
"""
import math
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan

from mrs_perception.victim_base import tf_to_rt, VictimNode

PERSON = 0   # COCO class id


def range_in_box(points_w, cam_w, yaw_a, yaw_b, depth):
    """Distance from cam_w to the nearest object between azimuths yaw_a..yaw_b: the lidar
    points in that sector, keeping only those within `depth` m of the closest (the person is
    in front of whatever is behind it); their median. A fixed angular window was wider than
    the 0.3 m doll at 4 m and took in the wall behind: +28 cm range bias."""
    d = points_w[:, :2] - cam_w[:2]
    az = np.arctan2(d[:, 1], d[:, 0])
    mid = math.atan2(math.sin(yaw_a) + math.sin(yaw_b), math.cos(yaw_a) + math.cos(yaw_b))
    half = abs(math.atan2(math.sin(yaw_a - yaw_b), math.cos(yaw_a - yaw_b))) / 2
    off = np.abs(np.arctan2(np.sin(az - mid), np.cos(az - mid)))
    dist = np.hypot(d[:, 0], d[:, 1])[off <= half]
    if dist.size == 0:
        return None
    return float(np.median(dist[dist <= dist.min() + depth]))


class YoloVictimNode(VictimNode):
    def __init__(self):
        super().__init__('yolo_victim_node', 'yolo')
        p = self.declare_parameter
        image_topic = p('image_topic', 'camera/image_raw').value
        self.conf = p('conf', 0.5).value
        self.period = 1.0 / p('rate', 5.0).value
        self.box_fraction = p('box_fraction', 0.6).value
        self.cluster_depth = p('cluster_depth', 0.15).value
        self.person_h = p('person_height', 0.29).value
        self.size_tol = p('size_tolerance', 0.5).value
        self.body_offset = p('body_offset', 0.07).value
        weights = p('weights', '').value
        device = p('device', '0').value
        from ultralytics import YOLO   # the venv's; slow import, so only here
        self.model = YOLO(weights)
        self.device = device
        self.model.predict(np.zeros((480, 640, 3), np.uint8), verbose=False, device=device)  # warm up
        self.bridge = CvBridge()
        self.scans = {}      # robot -> latest LaserScan
        self.last = {}       # robot -> wall time of the last processed image
        self.stats = {'frames': 0, 'person': 0, 'no_range': 0, 'size_reject': 0, 'hits': 0}
        for r in self.robots:
            self.create_subscription(LaserScan, f'{self.ns(r)}scan',
                                     lambda m, r=r: self.scans.__setitem__(r, m), qos_profile_sensor_data)
            self.create_subscription(Image, f'{self.ns(r)}{image_topic}',
                                     lambda m, r=r: self.on_image(r, m), qos_profile_sensor_data)
        self.create_timer(30.0, lambda: self.get_logger().info(f'stats {self.stats}'))
        self.get_logger().info(f'watching {self.robots} with {weights} on device {device}, '
                               f'conf {self.conf}, {1 / self.period:.0f} Hz per robot')

    def scan_points_world(self, scan):
        """The scan's valid returns as world-frame xyz (latest TF)."""
        tf = self.lookup(scan.header.frame_id, Time.from_msg(scan.header.stamp))
        if tf is None:
            return None
        r = np.asarray(scan.ranges, np.float64)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        pts = np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]), np.zeros(ok.sum())], 1)
        R, t = tf_to_rt(tf)
        return pts @ R.T + t

    def on_image(self, robot, msg):
        now = time.monotonic()
        if robot not in self.info or robot not in self.scans or now - self.last.get(robot, 0) < self.period:
            return
        self.last[robot] = now
        self.stats['frames'] += 1
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.model.predict(img, verbose=False, conf=self.conf, classes=[PERSON], device=self.device)[0]
        if len(res.boxes) == 0:
            return
        self.stats['person'] += 1
        b = max(res.boxes, key=lambda b: float(b.conf))
        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
        K, _ = self.info[robot]
        tf = self.lookup(msg.header.frame_id, Time.from_msg(msg.header.stamp))
        pts = self.scan_points_world(self.scans[robot])
        if tf is None or pts is None:
            return
        R, cam = tf_to_rt(tf)
        def ray_yaw(u):   # horizontal world direction of image column u
            ray = R @ np.array([(u - K[0, 2]) / K[0, 0], 0.0, 1.0])    # optical frame -> world
            return math.atan2(ray[1], ray[0])
        yaw = ray_yaw((x1 + x2) / 2)
        # the central `box_fraction` of the box, so its edges (background) don't count
        f = (1 - self.box_fraction) / 2 * (x2 - x1)
        rng = range_in_box(pts, cam, ray_yaw(x1 + f), ray_yaw(x2 - f), self.cluster_depth)
        if rng is None:
            self.stats['no_range'] += 1
            return
        box_rng = K[1, 1] * self.person_h / max(y2 - y1, 1.0)
        if abs(rng - box_rng) > self.size_tol * box_rng:
            self.stats['size_reject'] += 1
            self.get_logger().info(f'{robot}: person at {box_rng:.2f} m by size but lidar says '
                                   f'{rng:.2f} m: not ranged', throttle_duration_sec=5.0)
            return
        d = rng + self.body_offset
        w = np.array([cam[0] + d * math.cos(yaw), cam[1] + d * math.sin(yaw), 0.0])
        if rng > self.candidate_range:
            return
        self.add_candidate(w, yaw + math.pi, msg.header.stamp, robot, rng)
        if rng > self.max_range:
            return
        self.stats['hits'] += 1
        self.add_hit(w, msg.header.stamp, robot, rng)


def main():
    rclpy.init()
    node = YoloVictimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
