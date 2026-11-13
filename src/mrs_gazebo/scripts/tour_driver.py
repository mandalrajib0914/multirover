#!/usr/bin/env python3
"""Drive a robot along the `tour` waypoints in floor_plan.yaml (a repeatable mapping run).

  ros2 run mrs_gazebo tour_driver.py [--model robot1] [--speed 0.15]

Steers on Gazebo ground truth (gz-transport /world/<world>/pose/info), so the tour is the
same every run no matter how good odometry or SLAM are. It only publishes cmd_vel;
use it for mapping and experiments, not as a stand-in for Nav2.
"""
import argparse
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
import yaml


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GroundTruth:
    def __init__(self, world, model):
        self.model, self.pose, self.lock = model, None, threading.Lock()
        self.node = GzNode()
        if not self.node.subscribe(Pose_V, f'/world/{world}/pose/info', self._cb):
            raise RuntimeError('cannot subscribe to gz pose/info')

    def _cb(self, msg):
        for p in msg.pose:
            if p.name == self.model:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                with self.lock:
                    self.pose = (p.position.x, p.position.y, yaw)

    def get(self):
        with self.lock:
            return self.pose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='robot1')
    ap.add_argument('--world', default='floor')
    ap.add_argument('--speed', type=float, default=0.15, help='m/s')
    ap.add_argument('--turn', type=float, default=0.5, help='max rad/s')
    ap.add_argument('--plan', default=os.path.join(
        get_package_share_directory('mrs_gazebo'), 'config', 'floor_plan.yaml'))
    args, ros_args = ap.parse_known_args()

    with open(args.plan) as f:
        tour = yaml.safe_load(f)['tour']

    rclpy.init(args=ros_args)
    node = rclpy.create_node('tour_driver')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    gt = GroundTruth(args.world, args.model)
    while gt.get() is None:
        time.sleep(0.1)

    t0, dist, last = time.monotonic(), 0.0, gt.get()
    try:
        for i, (wx, wy) in enumerate(tour):
            node.get_logger().info(f'waypoint {i + 1}/{len(tour)}: ({wx:.2f}, {wy:.2f})')
            while True:
                x, y, yaw = gt.get()
                dist += math.hypot(x - last[0], y - last[1])
                last = (x, y, yaw)
                d = math.hypot(wx - x, wy - y)
                if d < 0.06:
                    break
                err = wrap(math.atan2(wy - y, wx - x) - yaw)
                cmd = Twist()
                cmd.angular.z = max(-args.turn, min(args.turn, 2.0 * err))
                # turn in place first, then drive with heading correction
                if abs(err) < 0.25:
                    cmd.linear.x = min(args.speed, 0.8 * d) * math.cos(err)
                pub.publish(cmd)
                time.sleep(0.05)
    finally:
        pub.publish(Twist())
        time.sleep(0.2)
    node.get_logger().info(f'tour done: {dist:.1f} m in {time.monotonic() - t0:.0f} s (wall)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
