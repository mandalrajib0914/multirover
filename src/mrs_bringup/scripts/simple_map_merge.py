#!/usr/bin/env python3
"""Merge the robots' SLAM maps into one world map, using their known start poses (S6.2).

  ros2 run mrs_bringup simple_map_merge.py --ros-args -p use_sim_time:=true \
      -p robots:="['robot1','robot2']"

Subscribes /robotN/map (slam_toolbox, frame robotN/map) and publishes /map in the world
frame "map". The pose of each robotN/map in "map" comes from TF (the static
map -> robotN/map that sim_multi_nav.launch.py publishes at the spawn pose), so nothing
here is sim-specific.

Why not multirobot_map_merge: with known_init_poses it warps each grid by its init pose in
cell units, ignores the robot map's own origin (slam_toolbox moves the origin as the map
grows), and then puts the merged origin at the grid centre. The result is not anchored to
any frame, and the Nav2 costmaps and the coordinator need one that is.

Merge rule per world cell: occupied beats free beats unknown. The output grid's cell centres
sit on multiples of the resolution in "map", and it only ever grows, so Nav2's static layer resizes as
little as possible.
"""
import math

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def merge_grids(maps, poses, res, bounds=None):
    """Merge OccupancyGrids into one world grid.

    maps:   {robot: OccupancyGrid in robot map frame}
    poses:  {robot: (x, y, yaw) of that frame in the world frame}
    bounds: optional (x0, y0, x1, y1) the result must at least cover (grow-only)
    Returns (grid int8 [H, W] row 0 = min y, origin_x, origin_y).
    """
    # World extent of every input grid: its four corners through its pose.
    xs, ys = [], []
    for r, m in maps.items():
        px, py, pyaw = poses[r]
        c, s = math.cos(pyaw), math.sin(pyaw)
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        w, h = m.info.width * m.info.resolution, m.info.height * m.info.resolution
        for cx, cy in ((ox, oy), (ox + w, oy), (ox, oy + h), (ox + w, oy + h)):
            xs.append(px + c * cx - s * cy)
            ys.append(py + s * cx + c * cy)
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
    if bounds is not None:
        x0, y0, x1, y1 = min(x0, bounds[0]), min(y0, bounds[1]), max(x1, bounds[2]), max(y1, bounds[3])
    # Snap to the resolution grid in the world frame, so cells line up between updates. Cell
    # centres sit on multiples of res (edges at k*res - res/2), the same convention as
    # floor_gt from make_floor.py, so comparing against the ground truth is cell-exact.
    x0 = math.floor(x0 / res + 0.5) * res - res / 2
    y0 = math.floor(y0 / res + 0.5) * res - res / 2
    width, height = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))

    out = np.full((height, width), -1, np.int8)
    rows, cols = np.indices((height, width))
    wx, wy = x0 + (cols + 0.5) * res, y0 + (rows + 0.5) * res
    for r, m in maps.items():
        px, py, pyaw = poses[r]
        c, s = math.cos(-pyaw), math.sin(-pyaw)
        # world cell centres -> robot map frame -> robot map cell (nearest)
        mx, my = c * (wx - px) - s * (wy - py), s * (wx - px) + c * (wy - py)
        i = np.floor((my - m.info.origin.position.y) / m.info.resolution).astype(np.int64)
        j = np.floor((mx - m.info.origin.position.x) / m.info.resolution).astype(np.int64)
        inside = (i >= 0) & (i < m.info.height) & (j >= 0) & (j < m.info.width)
        src = np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width)
        vals = np.full(out.shape, -1, np.int8)
        vals[inside] = src[i[inside], j[inside]]
        # occupied > free > unknown: a known cell turns unknown into free, occupied always wins
        known = vals >= 0
        occupied = vals >= 50
        out[known & (out < 0)] = 0
        out[occupied] = 100
    return out, x0, y0


class SimpleMapMerge(Node):
    def __init__(self):
        super().__init__('simple_map_merge')
        self.robots = list(self.declare_parameter('robots', ['robot1', 'robot2']).value)
        self.world_frame = self.declare_parameter('world_frame', 'map').value
        self.res = self.declare_parameter('resolution', 0.05).value
        rate = self.declare_parameter('rate', 1.0).value

        self.maps, self.poses, self.bounds = {}, {}, None
        self.tf = Buffer()
        TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        for r in self.robots:
            self.create_subscription(OccupancyGrid, f'/{r}/map',
                                     lambda msg, r=r: self.maps.__setitem__(r, msg), latched)
        self.pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.create_timer(1.0 / rate, self.merge)
        self.get_logger().info(f'merging {self.robots} into /map ({self.world_frame})')

    def pose_of(self, r):
        if r not in self.poses:
            try:
                t = self.tf.lookup_transform(self.world_frame, f'{r}/map', Time())
            except TransformException:
                return None
            tr = t.transform
            # map -> robotN/map is static, so look it up once
            self.poses[r] = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        return self.poses[r]

    def merge(self):
        maps = {r: m for r, m in self.maps.items() if self.pose_of(r) is not None}
        if not maps:
            return
        grid, ox, oy = merge_grids(maps, self.poses, self.res, self.bounds)
        h, w = grid.shape
        self.bounds = (ox, oy, ox + w * self.res, oy + h * self.res)
        msg = OccupancyGrid()
        msg.header.frame_id = self.world_frame
        msg.header.stamp = max((m.header.stamp for m in maps.values()),
                               key=lambda s: (s.sec, s.nanosec))
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.res
        msg.info.width, msg.info.height = w, h
        msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimpleMapMerge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
