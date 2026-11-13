#!/usr/bin/env python3
"""S4 acceptance check: Nav2 goals on the live SLAM map, scored on Gazebo ground truth.

  ros2 launch mrs_bringup sim_nav.launch.py headless:=true rviz:=false   # terminal 1
  ros2 run mrs_navigation nav_check.py                                  # terminal 2

Tests, with the robot starting at the robot1 spawn pose (the map-frame origin):
  1. goal     hall -> room B through the door; final position error vs gz ground truth
  2. obstacle drive across room B; as soon as the first global plan arrives, a 0.3 m box
              is spawned on the plan 0.8 m ahead of the robot. Pass means it replans around
              the box, reaches the goal, and the footprint never touches the box.
  3. unreachable  (a) a goal off the map: FAILED straight away; (b) a goal inside a fixed
              obstacle: Nav2 tries, runs its recoveries, then gives up (FAILED) without
              touching the obstacle
Goals are given in world coordinates and converted to the map frame with the spawn pose
(slam_toolbox starts its map at the robot's first pose).
"""
import argparse
import math
import os
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
import rclpy
import yaml

# Must match nav2_params.yaml (base_footprint frame, metres).
FOOTPRINT = [(0.09, 0.12), (0.09, -0.12), (-0.14, -0.12), (-0.14, 0.12)]
BOX = 0.30
MIN_CLEARANCE = 0.05   # footprint to box, metres; below this counts as a (near) collision


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


def footprint_box_clearance(pose, box_xy, half=BOX / 2):
    """Smallest distance from the robot footprint (sampled edges) to an axis-aligned box."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    best = float('inf')
    for i in range(len(FOOTPRINT)):
        (ax, ay), (bx, by) = FOOTPRINT[i], FOOTPRINT[(i + 1) % len(FOOTPRINT)]
        for k in range(11):
            px, py = ax + (bx - ax) * k / 10, ay + (by - ay) * k / 10
            wx, wy = x + c * px - s * py, y + s * px + c * py
            dx = max(abs(wx - box_xy[0]) - half, 0.0)
            dy = max(abs(wy - box_xy[1]) - half, 0.0)
            best = min(best, math.hypot(dx, dy))
    return best


def gz(world, service, reqtype, req):
    out = subprocess.run(
        ['gz', 'service', '-s', f'/world/{world}/{service}', '--reqtype', reqtype,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
        capture_output=True, text=True)
    return 'data: true' in out.stdout


def spawn_box(world, name, x, y):
    sdf = (f"<sdf version='1.9'><model name='{name}'><static>true</static><link name='l'>"
           f"<collision name='c'><geometry><box><size>{BOX} {BOX} 0.3</size></box></geometry>"
           f"</collision><visual name='v'><geometry><box><size>{BOX} {BOX} 0.3</size></box>"
           f"</geometry><material><diffuse>0.9 0.4 0.1 1</diffuse></material></visual>"
           f"</link></model></sdf>")
    sdf = sdf.replace('"', '\\"')
    return gz(world, 'create', 'gz.msgs.EntityFactory',
              f'sdf: "{sdf}" pose: {{position: {{x: {x} y: {y} z: 0.15}}}}')


def remove_box(world, name):
    return gz(world, 'remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')


class Checker:
    def __init__(self, args):
        self.args = args
        with open(os.path.join(get_package_share_directory('mrs_gazebo'),
                               'config', 'floor_plan.yaml')) as f:
            self.spawn = yaml.safe_load(f)['spawn_poses'][args.model]
        self.nav = BasicNavigator()
        self.gt = GroundTruth(args.world, args.model)
        self.plan = None
        self.nav.create_subscription(Path, 'plan', self._plan_cb, 10)
        self.results = []
        self.trace = []   # (wall time, test, x, y, yaw) for --trace
        self.boxes = []   # (name, world xy) of spawned boxes, for --trace

    def _plan_cb(self, msg):
        self.plan = msg

    # world <-> map (map origin = spawn pose)
    def world_to_map(self, x, y, yaw=0.0):
        sx, sy, syaw = self.spawn
        dx, dy = x - sx, y - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        return c * dx - s * dy, s * dx + c * dy, wrap(yaw - syaw)

    def map_to_world(self, x, y):
        sx, sy, syaw = self.spawn
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + c * x - s * y, sy + s * x + c * y

    def goal(self, wx, wy, wyaw):
        mx, my, myaw = self.world_to_map(wx, wy, wyaw)
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = mx, my
        p.pose.orientation.z, p.pose.orientation.w = math.sin(myaw / 2), math.cos(myaw / 2)
        return p

    def run_goal(self, name, wx, wy, wyaw, timeout, on_tick=None):
        self.plan = None
        self.nav.goToPose(self.goal(wx, wy, wyaw))
        t0, recoveries, path_len, last = time.monotonic(), 0, 0.0, self.gt.get()
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb:
                recoveries = max(recoveries, fb.number_of_recoveries)
            pose = self.gt.get()
            path_len += math.hypot(pose[0] - last[0], pose[1] - last[1])
            last = pose
            self.trace.append((time.monotonic(), name, *pose))
            if on_tick:
                on_tick(pose)
            if time.monotonic() - t0 > timeout:
                self.nav.cancelTask()
                break
        result = self.nav.getResult()
        x, y, yaw = self.gt.get()
        err = math.hypot(wx - x, wy - y)
        yaw_err = math.degrees(abs(wrap(wyaw - yaw)))
        dt = time.monotonic() - t0
        self.nav.get_logger().info(
            f'[{name}] {result.name} in {dt:.0f} s, driven {path_len:.2f} m, '
            f'final gt error {err:.3f} m / {yaw_err:.1f} deg, recoveries {recoveries}')
        return result, err, yaw_err, recoveries

    def plan_point_ahead(self, pose, ahead):
        """World point on the current global plan about `ahead` metres past the robot."""
        if self.plan is None or len(self.plan.poses) < 2:
            return None
        pts = [self.map_to_world(p.pose.position.x, p.pose.position.y) for p in self.plan.poses]
        i0 = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - pose[0], pts[i][1] - pose[1]))
        acc = 0.0
        for i in range(i0 + 1, len(pts)):
            acc += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            if acc >= ahead:
                return pts[i]
        return None

    def run(self):
        while self.gt.get() is None:
            time.sleep(0.1)
        self.nav.waitUntilNav2Active(localizer='robot_localization')
        ok = True

        # 1. hall -> room B
        res, err, yaw_err, _ = self.run_goal('goal', 3.0, 0.7, 0.0, 120)
        passed = res == TaskResult.SUCCEEDED and err < 0.20
        ok &= passed
        self.results.append(('1 goal hall -> room B', passed,
                             f'{res.name}, gt error {err:.3f} m, {yaw_err:.1f} deg'))

        # 2. obstacle spawned in the path across room B
        state = {'box': None, 'clear': float('inf')}
        box_name = 'nav_check_box'

        def tick(pose):
            if state['box'] is None and self.plan is not None:
                pt = self.plan_point_ahead(pose, 0.8)
                if pt is not None:
                    state['box'] = pt
                    self.boxes.append((box_name, pt))
                    spawned = spawn_box(self.args.world, box_name, *pt)
                    self.nav.get_logger().info(
                        f'spawned box at ({pt[0]:.2f}, {pt[1]:.2f}) world: {spawned}')
            if state['box'] is not None:
                state['clear'] = min(state['clear'], footprint_box_clearance(pose, state['box']))

        # First part of the leg, so the box lands mid-room (not in a doorway).
        res, err, _, rec = self.run_goal('obstacle', 5.4, 1.0, 0.0, 180, on_tick=tick)
        remove_box(self.args.world, box_name)
        passed = (res == TaskResult.SUCCEEDED and err < 0.20 and state['box'] is not None
                  and state['clear'] > MIN_CLEARANCE)
        ok &= passed
        self.results.append(('2 obstacle spawned in path', passed,
                             f'{res.name}, gt error {err:.3f} m, min footprint-box clearance '
                             f'{state["clear"]:.3f} m, recoveries {rec}'))

        # 3a. unreachable: outside the arena, off the map -> the planner rejects it at once
        res, _, _, rec = self.run_goal('off_map', 7.5, 1.0, 0.0, 120)
        passed = res == TaskResult.FAILED
        ok &= passed
        self.results.append(('3a goal off the map', passed, f'{res.name} (expected FAILED)'))

        # 3b. unreachable: inside the fixed 0.3 m box at (4.6, 1.2), on the map. Nav2 has to
        #     try, run its recoveries, and give up without driving into the box.
        box = (4.6, 1.2)
        clear = [float('inf')]

        def tick_b(pose):
            clear[0] = min(clear[0], footprint_box_clearance(pose, box))

        res, _, _, rec = self.run_goal('in_obstacle', box[0], box[1], 0.0, 300, on_tick=tick_b)
        x, y, _ = self.gt.get()
        inside = 0.0 < x < 6.0 and 0.0 < y < 5.0
        passed = res == TaskResult.FAILED and inside and clear[0] > MIN_CLEARANCE
        ok &= passed
        self.results.append(('3b goal inside an obstacle', passed,
                             f'{res.name} (expected FAILED), recoveries {rec}, min footprint-box '
                             f'clearance {clear[0]:.3f} m'))

        print('\n==== S4 nav_check ====')
        for name, p, info in self.results:
            print(f'{"PASS" if p else "FAIL"}  {name}: {info}')
        print('ALL PASS' if ok else 'SOME FAILED')
        return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', default='robot1')
    ap.add_argument('--world', default='floor')
    ap.add_argument('--trace', default='', help='write the gt trajectory to this CSV')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    checker = Checker(args)
    ok = checker.run()
    if args.trace:
        with open(args.trace, 'w') as f:
            f.write('t,test,x,y,yaw\n')
            for t, name, x, y, yaw in checker.trace:
                f.write(f'{t:.3f},{name},{x:.4f},{y:.4f},{yaw:.4f}\n')
            for name, pt in checker.boxes:
                f.write(f'nan,box:{name},{pt[0]:.4f},{pt[1]:.4f},nan\n')
    checker.nav.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
