#!/usr/bin/env python3
"""Sim only: turn Gazebo's perfect DiffDrive odom into realistic wheel odometry.

  odom (gz DiffDrive, zero covariance)  ->  wheel/odom (what the EKF fuses)

Gazebo's DiffDrive odom has all-zero covariances and no error, so robot_localization
would trust it completely and ignore the IMU. This node
  * scales the twist (wheel_slip: turning over-reports rotation, like skid on a real base),
  * adds gaussian noise to vx and wz,
  * sets twist covariances the EKF can weigh against the IMU,
  * integrates its own pose from the degraded twist, so wheel/odom drifts like real odometry
    (the EKF only uses the twist; the pose is there to compare against odometry/filtered).
On the real robot the base driver publishes wheel/odom instead and this node is not run.
"""
import math

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SimWheelOdom(Node):
    def __init__(self):
        super().__init__('sim_wheel_odom')
        self.vx_scale = self.declare_parameter('vx_scale', 1.0).value
        self.wz_scale = self.declare_parameter('wz_scale', 1.04).value   # 4% over-rotation
        self.vx_std = self.declare_parameter('vx_noise_std', 0.005).value
        self.wz_std = self.declare_parameter('wz_noise_std', 0.02).value
        # Reported variances (not the injected noise): the EKF uses these as weights.
        self.cov_vx = self.declare_parameter('cov_vx', 1e-3).value
        self.cov_vy = self.declare_parameter('cov_vy', 1e-4).value
        self.cov_wz = self.declare_parameter('cov_wz', 1e-2).value
        self.rng = np.random.default_rng(self.declare_parameter('seed', 0).value)

        self.x = self.y = self.yaw = 0.0
        self.last_t = None
        self.pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        tw = msg.twist.twist
        moving = abs(tw.linear.x) > 1e-4 or abs(tw.angular.z) > 1e-4
        # No noise while standing still, so the estimate doesn't wander at rest.
        vx = tw.linear.x * self.vx_scale + (self.rng.normal(0, self.vx_std) if moving else 0.0)
        wz = tw.angular.z * self.wz_scale + (self.rng.normal(0, self.wz_std) if moving else 0.0)

        if self.last_t is not None and t > self.last_t:
            dt = t - self.last_t
            self.x += vx * math.cos(self.yaw) * dt
            self.y += vx * math.sin(self.yaw) * dt
            self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        self.last_t = t

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        p = out.pose.pose
        p.position.x, p.position.y = self.x, self.y
        p.orientation.z, p.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        pc = [0.0] * 36
        pc[0] = pc[7] = pc[35] = 1e3            # pose: dead reckoning, not to be trusted
        pc[14] = pc[21] = pc[28] = 1e6
        out.pose.covariance = pc
        out.twist.twist.linear.x = vx
        out.twist.twist.angular.z = wz
        tc = [0.0] * 36
        tc[0], tc[7], tc[35] = self.cov_vx, self.cov_vy, self.cov_wz
        tc[14] = tc[21] = tc[28] = 1e6
        out.twist.covariance = tc
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(SimWheelOdom())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
