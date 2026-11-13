#!/usr/bin/env python3
"""S2.6 odometry sanity checks against Gazebo ground truth.

Run with the sim up (and nothing else driving the robot):
  ros2 run mrs_gazebo odom_sanity_check.py [--model robot1] [--distance 1.0]

1. Drive straight until odom says `distance` m; compare with the true displacement (±2%).
2. Spin 360° in place by odom; compare with the true yaw change (±3°).
3. While turning left, imu/data angular_velocity.z must be positive.
Ground truth comes from `gz model -m <model> -p`.
"""
import argparse
import math
import re
import subprocess
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def gz_pose(model):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True, text=True).stdout
    rows = re.findall(r'\[([-\d.e]+) ([-\d.e]+) ([-\d.e]+)\]', out)
    (x, y, _), (_, _, yaw) = rows[-2], rows[-1]
    return float(x), float(y), float(yaw)


class Checker(Node):
    def __init__(self):
        super().__init__('odom_sanity_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom = None
        self.imu_wz = []
        self.create_subscription(Odometry, 'odom', self._odom, 10)
        self.create_subscription(Imu, 'imu/data', lambda m: self.imu_wz.append(m.angular_velocity.z), 50)

    def _odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def drive(self, v, w, done):
        """Publish (v, w) until done(odom) is true, then stop and settle."""
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        while not done(self.odom):
            self.pub.publish(cmd)
            self.spin_for(0.02)
        self.pub.publish(Twist())
        self.spin_for(1.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='robot1')
    ap.add_argument('--distance', type=float, default=1.0)
    ap.add_argument('--speed', type=float, default=0.2)
    args = ap.parse_args()

    rclpy.init()
    n = Checker()
    while n.odom is None:
        n.spin_for(0.1)
    ok = True

    # 1. straight line
    o0, g0 = n.odom, gz_pose(args.model)
    n.drive(args.speed, 0.0, lambda o: math.hypot(o[0] - o0[0], o[1] - o0[1]) >= args.distance)
    o1, g1 = n.odom, gz_pose(args.model)
    d_odom = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    d_true = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
    err = abs(d_odom - d_true) / d_true * 100
    ok &= err <= 2.0
    print(f'[{"PASS" if err <= 2.0 else "FAIL"}] straight: odom {d_odom:.3f} m, true {d_true:.3f} m, error {err:.2f}% (limit 2%)')

    # 2. + 3. spin 360 deg left, integrating odom yaw
    state = {'prev': n.odom[2], 'acc': 0.0}

    def turned(o):
        state['acc'] += wrap(o[2] - state['prev'])
        state['prev'] = o[2]
        return state['acc'] >= 2 * math.pi

    g0 = gz_pose(args.model)
    n.imu_wz.clear()
    n.drive(0.0, 0.8, turned)
    g1 = gz_pose(args.model)
    wz = sorted(n.imu_wz)[len(n.imu_wz) // 2] if n.imu_wz else float('nan')
    yaw_err = math.degrees(abs(wrap(g1[2] - g0[2])))
    ok &= yaw_err <= 3.0 and wz > 0
    print(f'[{"PASS" if yaw_err <= 3.0 else "FAIL"}] spin 360: true yaw error {yaw_err:.2f} deg (limit 3 deg)')
    print(f'[{"PASS" if wz > 0 else "FAIL"}] left turn: median imu angular_velocity.z = {wz:+.3f} rad/s (must be > 0)')

    print('ALL PASS' if ok else 'SOME CHECKS FAILED')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
