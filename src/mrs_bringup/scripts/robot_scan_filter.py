#!/usr/bin/env python3
"""Remove the other robots from this robot's lidar scan before SLAM sees it (S6).

  scan -> [robot_scan_filter] -> scan_filtered -> slam_toolbox
                (costmaps keep the raw scan, so robots still avoid each other)

Without it, robot2's SLAM map records robot1's body as a wall. After map merging, every
robot sits inside an obstacle drawn by its neighbours ("Start occupied" in the planner), and
wherever one robot saw another, a ghost obstacle stays in the static map until that spot is
rescanned. Beams whose end point lies within `radius` of another robot's base_footprint are
set to +inf (no return). Other robots' positions come from TF (all robots share /tf), so this
works the same on real robots as long as their poses reach the shared TF tree.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RobotScanFilter(Node):
    def __init__(self):
        super().__init__('robot_scan_filter')
        self.others = [r for r in self.declare_parameter('other_robots', ['']).value if r]
        # footprint circumradius 0.19 m + TF latency at 0.2 m/s + lidar noise
        self.radius = self.declare_parameter('radius', 0.27).value
        self.tracked = set()   # other robots whose base_footprint has been in TF
        ns = self.get_namespace().strip('/')
        self.prefix = f'{ns}/' if ns else ''
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(LaserScan, 'scan_filtered', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(f'filtering {self.others} (radius {self.radius} m) out of scan')

    def lookup_xy_yaw(self, target, source):
        """(x, y, yaw) of frame `source` in frame `target`, latest available."""
        t = self.tf.lookup_transform(target, source, Time()).transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def other_in_laser(self, laser, r):
        """Other robot r's position in our laser frame, or None if not known yet."""
        try:
            # latest, not at msg.header.stamp: a robot moves < 3 cm in the typical lag
            x, y, _ = self.lookup_xy_yaw(laser, f'{r}/base_footprint')
            self.tracked.add(r)
            return x, y
        except TransformException:
            pass
        if r in self.tracked:
            return None   # was tracked, now gone from TF (stopped/dead): skip it
        # Start-up: its full chain needs its SLAM's map -> odom, which needs scans, which we
        # would be holding back: a deadlock. Before anyone has moved, though, robot r is at
        # its start pose (the origin of r/map, from the static map -> r/map) and our own map
        # and odom frames still coincide.
        try:
            ox, oy, _ = self.lookup_xy_yaw(f'{self.prefix}map', f'{r}/map')
            lx, ly, lyaw = self.lookup_xy_yaw(laser, f'{self.prefix}odom')
        except TransformException:
            return None
        c, s = math.cos(lyaw), math.sin(lyaw)
        return lx + c * ox - s * oy, ly + s * ox + c * oy

    def on_scan(self, msg):
        centres = [self.other_in_laser(msg.header.frame_id, r) for r in self.others]
        if any(c is None and r not in self.tracked for c, r in zip(centres, self.others)):
            # Hold scans until every other robot can be placed at least by its start pose. A
            # scan let through now would put a neighbour into the SLAM map for good (later
            # scans with no return there don't clear it). This only waits for static TFs.
            self.get_logger().info('waiting for the other robots\' start poses in TF',
                                   throttle_duration_sec=5.0)
            return
        centres = [c for c in centres if c is not None]
        if centres:
            ranges = np.asarray(msg.ranges, np.float32)
            ang = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
            with np.errstate(invalid='ignore'):
                px, py = ranges * np.cos(ang), ranges * np.sin(ang)
                hit = np.zeros(len(ranges), bool)
                for cx, cy in centres:
                    hit |= np.hypot(px - cx, py - cy) < self.radius
            if hit.any():
                ranges[hit] = math.inf
                msg.ranges = ranges.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RobotScanFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
