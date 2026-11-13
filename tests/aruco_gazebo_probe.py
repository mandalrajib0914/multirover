"""Count ArUco detections from one robot's real Gazebo camera for a few seconds."""
import os
import sys
import time

import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Image

sys.path.insert(0, 'src/mrs_perception')
from mrs_perception.aruco_victim_node import ArucoVictimNode  # noqa: E402

rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', "robots:=['']", '-p',
                 'world_frame:=odom', '-p', 'max_range:=5.0', '-p', 'confirm_hits:=1'])
n = ArucoVictimNode()
frames = [0]
lags = []
orig = n.lookup


def lookup(frame, stamp):
    tf = orig(frame, stamp)
    try:
        latest = n.tf.lookup_transform(n.world, frame, Time())
        lags.append((stamp - Time.from_msg(latest.header.stamp)).nanoseconds * 1e-9)
    except Exception:
        lags.append(None)
    return tf


n.lookup = lookup
n.create_subscription(Image, '/camera/image_raw', lambda m: frames.__setitem__(0, frames[0] + 1), 1)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)
est = n.estimate() if n.hits else None
good = [l for l in lags if l is not None]
print(f'frames {frames[0]}, raw detections {n.detections}, accepted {len(n.hits)}, '
      f'image-minus-latest-TF lag: min {min(good, default=0):.3f} max {max(good, default=0):.3f} s '
      f'(None: {lags.count(None)}), est_odom {None if est is None else est.round(3).tolist()}',
      flush=True)
os._exit(0)
