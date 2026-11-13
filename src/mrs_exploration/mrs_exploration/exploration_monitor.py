"""Record one exploration run: time, coverage vs time, distance, final map score.

  ros2 run mrs_exploration exploration_monitor --ros-args -p use_sim_time:=true \
      [-p robot:=robot1] [-p results_dir:=results/s5] [-p timeout:=900.0]
  multi-robot (S6): -p robots:="['robot1','robot2']" -p map_frame:=world \
      -p status_topics:="['/coordinator/status']"      (or one explore/status per robot)

Started by sim_explore.launch.py / sim_multi_robot.launch.py. It watches explore/status
(explore_lite, or the coordinator's status in the same message type) and /map, which is the
robot's SLAM map (map_frame robot: origin at its spawn pose) or the merged map (world):
  t = 0          first EXPLORATION_STARTED (sim time)
  every period   coverage of floor_gt free space (same definition as map_vs_gt.py), driven
                 distance from gz ground truth -> coverage.csv
  done           EXPLORATION_COMPLETE, timeout, or "stalled" (no coverage gain for
                 stall_timeout s and no robot moving for 60 s). explore_lite sends the same status for
                 "no frontiers left" and "all remaining frontiers blacklisted"; the log says which
  victim         first /victim/confirmed: time and error against floor_plan.yaml's victim
                 (and the last /victim/pose median at the end)
Then it saves the map (map_saver_cli), scores it with map_vs_gt.py and writes summary.yaml,
all into <results_dir>/<YYYYmmdd_HHMMSS>/. It waits up to return_timeout for
RETURNED_TO_ORIGIN, and then exits (sim_explore.launch.py can shut the launch down then).
"""
import datetime
import math
import os
import re
import subprocess
import threading

from ament_index_python.packages import get_package_share_directory
import cv2
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
import yaml

try:  # gz ground truth is optional (a real robot has none)
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
except ImportError:
    GzNode = None


def load_map_yaml(path):
    """Return (grid, res, origin_xy); grid: 1 occupied, 0 free, -1 unknown, row 0 = min y."""
    with open(path) as f:
        meta = yaml.safe_load(f)
    img = cv2.imread(os.path.join(os.path.dirname(path), meta['image']), cv2.IMREAD_GRAYSCALE)
    p = (255 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    grid = np.full(img.shape, -1, np.int8)
    grid[p > meta['occupied_thresh']] = 1
    grid[p < meta['free_thresh']] = 0
    return np.flipud(grid), meta['resolution'], meta['origin'][:2]


class Coverage:
    """Share of true free cells that a live OccupancyGrid marks free (as in map_vs_gt.py)."""

    def __init__(self, gt_yaml, spawn):
        self.gt, self.gres, (self.gox, self.goy) = load_map_yaml(gt_yaml)
        self.spawn = spawn
        self.free = self.gt == 0
        rows, cols = np.indices(self.gt.shape)
        sx, sy, syaw = spawn
        wx = self.gox + (cols + 0.5) * self.gres - sx
        wy = self.goy + (rows + 0.5) * self.gres - sy
        c, s = math.cos(-syaw), math.sin(-syaw)
        # GT cell centres in the SLAM map frame (map origin = spawn pose)
        self.mx, self.my = c * wx - s * wy, s * wx + c * wy

    def __call__(self, msg):
        info = msg.info
        grid = np.asarray(msg.data, np.int8).reshape(info.height, info.width)
        si = np.floor((self.my - info.origin.position.y) / info.resolution).astype(int)
        sj = np.floor((self.mx - info.origin.position.x) / info.resolution).astype(int)
        inside = (si >= 0) & (si < info.height) & (sj >= 0) & (sj < info.width)
        slam = np.full(self.gt.shape, -1, np.int8)
        slam[inside] = grid[si[inside], sj[inside]]
        # OccupancyGrid: 0 free, 100 occupied, -1 unknown; slam_toolbox only writes 0/100/-1
        return float(((slam == 0) & self.free).sum() / max(self.free.sum(), 1))


class GroundTruth:
    def __init__(self, world, models):
        self.models, self.poses, self.lock = set(models), {}, threading.Lock()
        self.node = GzNode()
        self.node.subscribe(Pose_V, f'/world/{world}/pose/info', self._cb)

    def _cb(self, msg):
        for p in msg.pose:
            if p.name in self.models:
                with self.lock:
                    self.poses[p.name] = (p.position.x, p.position.y)

    def get(self, model):
        with self.lock:
            return self.poses.get(model)


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        gz_pkg = get_package_share_directory('mrs_gazebo')
        self.robot = self.declare_parameter('robot', 'robot1').value
        self.robots = list(self.declare_parameter('robots', ['']).value)
        self.robots = [r for r in self.robots if r] or [self.robot]
        self.map_frame = self.declare_parameter('map_frame', 'robot').value   # robot | world
        self.strategy = self.declare_parameter('strategy', '').value          # for the record
        world = self.declare_parameter('world', 'floor').value
        results = self.declare_parameter('results_dir', 'results/s5').value
        self.timeout = self.declare_parameter('timeout', 900.0).value
        self.period = self.declare_parameter('period', 5.0).value
        self.return_timeout = self.declare_parameter('return_timeout', 120.0).value
        self.stall_timeout = self.declare_parameter('stall_timeout', 120.0).value   # 0: off
        self.tag = self.declare_parameter('tag', '').value
        gt_yaml = os.path.join(gz_pkg, 'maps', 'floor_gt.yaml')
        plan_file = self.declare_parameter('plan_file', '').value or os.path.join(
            gz_pkg, 'config', 'floor_plan.yaml')
        with open(plan_file) as f:
            plan = yaml.safe_load(f)
        self.victim_true = plan['victim'][:2]
        spawn = (0.0, 0.0, 0.0) if self.map_frame == 'world' else plan['spawn_poses'][self.robot]

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        run_dir = self.declare_parameter('run_dir', '').value
        self.out = os.path.abspath(run_dir or os.path.join(
            results, stamp + (f'_{self.tag}' if self.tag else '')))
        os.makedirs(self.out, exist_ok=True)
        self.coverage = Coverage(gt_yaml, spawn)
        self.gt = GroundTruth(world, self.robots) if GzNode else None

        self.map = None
        self.t0 = None
        self.t_done = None
        self.done_reason = None
        self.dist = {r: 0.0 for r in self.robots}
        self.last_xy = {}
        self.traj = []     # (t, robot, x, y) at 2 Hz, for path length / overlap analysis
        self._traj_tick = 0
        self.samples = []   # (t, coverage, distance)
        self.returned = False
        self.finished = False
        self.victim_t, self.victim_confirmed, self.victim_last = None, None, None
        self.mission_end = None
        self.victim_status_t = None

        # Several topics (independent explore_lite per robot): t = 0 at the first STARTED,
        # done when every topic has said COMPLETE, returned when every one says RETURNED.
        topics = [t for t in self.declare_parameter('status_topics', ['explore/status']).value if t]
        self.complete, self.returned_set = set(), set()
        self.n_status = len(topics)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for t in topics:
            self.create_subscription(ExploreStatus, t, lambda m, t=t: self._status(m, t), latched)
        self.create_subscription(OccupancyGrid, 'map', self._map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointStamped, '/victim/confirmed', self._victim_confirmed,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseStamped, '/victim/pose', self._victim_pose, 10)
        self.create_timer(0.2, self._odometer)
        self.create_timer(self.period, self._sample)
        self.get_logger().info(f'recording to {self.out}')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        return self.now_s() - self.t0 if self.t0 is not None else 0.0

    def _status(self, msg, topic):
        s = msg.status
        self.get_logger().info(f'{topic}: {s} (t = {self.elapsed():.1f} s)')
        if s == ExploreStatus.EXPLORATION_STARTED and self.t0 is None:
            self.t0 = self.now_s()
        elif s == 'victim_confirmed':
            self.victim_status_t = self.now_s()
        elif s == ExploreStatus.EXPLORATION_COMPLETE and self.t0 is not None:
            # The mission ended *because of* the victim only if COMPLETE follows the
            # confirmation at once (stop_on_victim); otherwise exploration went on.
            if self.victim_status_t is not None and self.now_s() - self.victim_status_t < 1.0:
                self.mission_end = 'victim_confirmed'
            self.complete.add(topic)
            if len(self.complete) == self.n_status and self.t_done is None:
                self._finish('exploration_complete')
        elif s == ExploreStatus.RETURNED_TO_ORIGIN:
            self.returned_set.add(topic)
            self.returned = len(self.returned_set) == self.n_status

    def _victim_confirmed(self, msg):
        if self.victim_t is not None or self.t0 is None:
            return
        self.victim_t = self.elapsed()
        self.victim_confirmed = (msg.point.x, msg.point.y)
        err = math.hypot(msg.point.x - self.victim_true[0], msg.point.y - self.victim_true[1])
        self.get_logger().info(f'victim confirmed at t = {self.victim_t:.1f} s, '
                               f'({msg.point.x:.3f}, {msg.point.y:.3f}), error {err:.3f} m')

    def _victim_pose(self, msg):
        self.victim_last = (msg.pose.position.x, msg.pose.position.y)

    def _map(self, msg):
        # Ignore a stale latched /map from a previous run (stamped in the future; sim time
        # restarts at 0 every run).
        if Time.from_msg(msg.header.stamp) > self.get_clock().now() + Duration(seconds=2.0):
            return
        self.map = msg

    def _odometer(self):
        if self.gt is None or self.t0 is None or self.t_done is not None:
            return
        self._traj_tick += 1
        for r in self.robots:
            xy = self.gt.get(r)
            if xy is None:
                continue
            if r in self.last_xy:
                self.dist[r] += math.hypot(xy[0] - self.last_xy[r][0], xy[1] - self.last_xy[r][1])
            self.last_xy[r] = xy
            if self._traj_tick % 2 == 0:   # odometer runs at 5 Hz; trajectories at 2.5 Hz
                self.traj.append((self.elapsed(), r, xy[0], xy[1]))

    def stalled(self):
        """Coverage up < 0.2 % in stall_timeout s and all robots together moved < 0.2 m in
        the last 60 s: the explorer is stuck (it would otherwise run into `timeout`)."""
        t, cov, dist = self.samples[-1]
        old = [s for s in self.samples if s[0] <= t - self.stall_timeout]
        recent = [s for s in self.samples if s[0] <= t - 60.0]
        if not old or not recent:
            return False
        return cov - old[-1][1] < 0.002 and dist - recent[-1][2] < 0.2

    @property
    def distance(self):
        return sum(self.dist.values())

    def _sample(self):
        if self.t0 is None:
            return
        if self.map is not None and self.t_done is None:
            self.samples.append((self.elapsed(), self.coverage(self.map), self.distance))
            t, cov, d = self.samples[-1]
            self.get_logger().info(f't {t:6.1f} s  coverage {cov * 100:5.1f}%  driven {d:5.1f} m')
        if self.t_done is None and self.elapsed() > self.timeout:
            self._finish('timeout')
        if self.t_done is None and self.stall_timeout > 0 and self.stalled():
            self.get_logger().warn(f'no coverage gain for {self.stall_timeout:.0f} s and no robot '
                                   'moving for 60 s: ending the run as stalled')
            self._finish('stalled')
        if self.t_done is not None and not self.finished and (
                self.returned or self.now_s() - self.t_done > self.return_timeout):
            self.finished = True

    def victim_summary(self):
        def err(p):
            return None if p is None else round(math.hypot(p[0] - self.victim_true[0],
                                                           p[1] - self.victim_true[1]), 3)
        return {
            'true_xy': [float(v) for v in self.victim_true],
            'time_to_confirm_s': None if self.victim_t is None else round(self.victim_t, 1),
            'confirmed_xy': None if self.victim_confirmed is None else
            [round(float(v), 3) for v in self.victim_confirmed],
            'confirmed_error_m': err(self.victim_confirmed),
            'final_xy': None if self.victim_last is None else [round(float(v), 3) for v in self.victim_last],
            'final_error_m': err(self.victim_last),
        }

    def _finish(self, reason):
        self.t_done, self.done_reason = self.now_s(), reason
        duration = self.t_done - self.t0
        final_cov = self.coverage(self.map) if self.map is not None else 0.0
        self.samples.append((duration, final_cov, self.distance))
        self.get_logger().info(f'exploration done ({reason}) after {duration:.1f} s sim, '
                               f'coverage {final_cov * 100:.1f}%, driven {self.distance:.1f} m')

        with open(os.path.join(self.out, 'trajectories.csv'), 'w') as f:
            f.write('t_s,robot,x,y\n')
            for t, r, x, y in self.traj:
                f.write(f'{t:.2f},{r},{x:.3f},{y:.3f}\n')
        with open(os.path.join(self.out, 'coverage.csv'), 'w') as f:
            f.write('t_s,coverage,distance_m\n')
            for t, c, d in self.samples:
                f.write(f'{t:.2f},{c:.4f},{d:.3f}\n')

        map_base = os.path.join(self.out, 'map')
        subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_base,
                        '--ros-args', '-p', 'use_sim_time:=true'],
                       capture_output=True, text=True, timeout=60)
        score = subprocess.run(['ros2', 'run', 'mrs_gazebo', 'map_vs_gt.py', map_base + '.yaml',
                                '--robot', 'world' if self.map_frame == 'world' else self.robot,
                                '--victim', str(self.victim_true[0]), str(self.victim_true[1])],
                               capture_output=True, text=True, timeout=60)
        metrics = {}
        for m in re.finditer(r'^(wall precision|wall recall|wall F1|free IoU|coverage)\s+([\d.]+)%',
                             score.stdout, re.MULTILINE):
            metrics[m.group(1).replace(' ', '_')] = float(m.group(2)) / 100
        self.get_logger().info('map_vs_gt: ' + ' | '.join(score.stdout.splitlines()[:3]))

        # first sample at which live coverage reached 90 % / 95 %
        def first_time(level):
            return next((t for t, c, _ in self.samples if c >= level), None)

        final_saved = metrics.get('coverage', final_cov)

        summary = {
            'robots': self.robots,
            'n_robots': len(self.robots),
            'strategy': self.strategy,
            'tag': self.tag,
            'end_reason': reason if self.mission_end is None else self.mission_end,
            'exploration_time_s': round(duration, 1),
            'distance_m': round(self.distance, 2),
            'distance_per_robot_m': {r: round(d, 2) for r, d in self.dist.items()},
            'coverage_live': round(final_cov, 4),
            'time_to_90pct_s': first_time(0.90),
            'time_to_95pct_s': first_time(0.95),
            # coverage plateaus a little below 100 % (wall-adjacent cells, obstacle shadows),
            # so this is the more comparable "done" time between runs and strategies
            'time_to_99pct_of_final_s': first_time(0.99 * final_saved),
            'victim': self.victim_summary(),
            'saved_map': map_base + '.yaml',
            'map_vs_gt': metrics,
        }
        with open(os.path.join(self.out, 'summary.yaml'), 'w') as f:
            yaml.safe_dump(summary, f, sort_keys=False)
        self.get_logger().info(f'summary: {summary}')


def main():
    rclpy.init()
    node = ExplorationMonitor()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.get_logger().info(f'returned to start: {node.returned}. Results in {node.out}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
