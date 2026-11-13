"""Capture camera frames of a person model at several distances (for the YOLO test, S7b).

  (sim_one_robot running with world floor_novictim.sdf and a person model spawned)
  python3 tests/yolo_capture.py <out_dir> <person_x> <person_y> d1 d2 ...

Teleports robot1 (gz set_pose) to distance d on the -x side of the person, facing it, waits
for fresh frames and saves /camera/image_raw as <out_dir>/d<d>.png. Only the camera matters
here, so TF/odometry being wrong after a teleport is irrelevant.
"""
import os
import subprocess
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def set_pose(x, y, yaw):
    import math
    req = (f'name: "robot1" position: {{x: {x} y: {y} z: 0.01}} '
           f'orientation: {{z: {math.sin(yaw / 2)} w: {math.cos(yaw / 2)}}}')
    subprocess.run(['gz', 'service', '-s', '/world/floor/set_pose', '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                   capture_output=True, text=True)


def main():
    out, px, py = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    dists = [float(d) for d in sys.argv[4:]]
    os.makedirs(out, exist_ok=True)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = rclpy.create_node('yolo_capture')
    last = {}
    node.create_subscription(Image, '/camera/image_raw', lambda m: last.__setitem__('img', m),
                             qos_profile_sensor_data)
    bridge = CvBridge()
    for d in dists:
        set_pose(px - d, py, 0.0)
        t0 = time.time()
        last.pop('img', None)
        while time.time() - t0 < 2.5:            # let the camera render the new view
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'img' in last:
            cv2.imwrite(os.path.join(out, f'd{d:.1f}.png'), bridge.imgmsg_to_cv2(last['img'], 'bgr8'))
            print(f'saved d={d}', flush=True)
        else:
            print(f'no image at d={d}', flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
