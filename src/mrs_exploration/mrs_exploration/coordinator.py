"""Centralized frontier allocation on the merged map (S6.3, the paper's contribution).

  ros2 run mrs_exploration coordinator --ros-args --params-file config/coordinator.yaml \
      -p robots:="['robot1','robot2']" -p strategy:=greedy

Every `period` seconds, on the merged /map (frame "map", from simple_map_merge):
  1. frontier cells: free cells with an unknown 4-neighbour
  2. clusters (8-connected); clusters shorter than `min_size` m are dropped. Each cluster's
     goal is the cell nearest its centroid that is at least `goal_clearance` m from any
     occupied cell (a goal against a wall is in the planner's lethal zone and fails)
  3. robot poses from TF (map -> robotN/base_footprint)
  4. cost(r, f): shortest 8-connected path over cells with `path_clearance` from obstacles
     (cost_mode: path), or straight-line distance (cost_mode: euclidean).
     score(r, f) = gain(f) - lambda * cost(r, f), gain = frontier length in m
  5. assignment, `strategy`:
       nearest    each robot takes its cheapest frontier; robots may pick the same one
       greedy     best (r, f) pair first; frontiers within R of f lose gain in proportion
                  (d / R) for the remaining robots; repeat while a frontier has gain left
       hungarian  optimal one-to-one assignment (scipy linear_sum_assignment) on -score
     greedy and hungarian add `hysteresis` m to the score of a robot's current target (if
     it is still a frontier), so a robot only switches when another frontier is clearly
     better. Without it, re-planning every T s sent a robot back and forth across the
     building between near-equal leftovers (seen in the first 2-robot runs).
     With fewer useful frontiers than robots, greedy and hungarian leave the rest without a
     new goal (they finish their current one) rather than doubling up on a frontier.
  6. NavigateToPose goals to /robotN/navigate_to_pose, in frame "map". A goal is re-sent
     only if the target moved more than `resend_dist` (D). A frontier is blacklisted after
     `max_failures` aborted or stalled goals (no progress for `progress_timeout` s), or once
     it is still there `reached_grace` s after a robot reached its goal (can't be cleared).
  7. no frontiers for `done_cycles` periods in a row, or frontiers that no robot can reach for
     `unreachable_timeout` s: exploration complete, robots go home
     (their start poses), status on coordinator/status (explore_lite_msgs/ExploreStatus, so
     exploration_monitor can record the run exactly like a single explore_lite run)
  8. markers on coordinator/markers: frontiers (size ~ gain), goals, robot -> goal lines
Mission (S7): EXPLORING -> VICTIM_CONFIRMED (/victim/confirmed from the victim detector)
-> FINISH. Exploration alone rarely brings a camera within the detector's 2 m, facing the
marker (a 2-robot run mapped everything and saw the marker once, far off). So every sighting
(/victim/candidate, up to 4 m) triggers an INVESTIGATION: the robot with the cheapest path
goes to a viewpoint `view_dist` m in front of the marker, facing it, and waits `view_dwell` s;
other viewpoints are tried if that fails. That robot leaves frontier assignment meanwhile,
and exploration is not declared finished while an investigation is pending. With stop_on_victim the mission ends there and the robots go home; without it
(exploration experiments) the confirmation is only reported and exploration goes on.
The status topic then carries "victim_confirmed" before the usual end statuses.
"""
import math

from action_msgs.msg import GoalStatus
from explore_lite_msgs.msg import ExploreStatus
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy import ndimage, optimize
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

STRATEGIES = ('nearest', 'greedy', 'hungarian')
ROBOT_COLOURS = [(0.12, 0.47, 0.71), (1.0, 0.5, 0.05), (0.17, 0.63, 0.17), (0.84, 0.15, 0.16)]


# ---------------------------------------------------------------- frontier detection

def find_frontiers(grid, res, origin, min_size, goal_clearance):
    """Frontier clusters of an occupancy grid (int8, row 0 = min y).

    Returns a list of dicts: goal (x, y) in the grid frame, gain (m), cells (N, 2) world xy.
    """
    free, unknown, occupied = grid == 0, grid < 0, grid >= 50
    unk = np.pad(unknown, 1)
    near_unknown = unk[:-2, 1:-1] | unk[2:, 1:-1] | unk[1:-1, :-2] | unk[1:-1, 2:]
    frontier = free & near_unknown
    labels, n = ndimage.label(frontier, structure=np.ones((3, 3), bool))
    if n == 0:
        return []
    clearance = ndimage.distance_transform_edt(~occupied) * res
    out = []
    for k, sl in enumerate(ndimage.find_objects(labels), start=1):
        rows, cols = np.nonzero(labels[sl] == k)
        rows, cols = rows + sl[0].start, cols + sl[1].start
        gain = len(rows) * res
        if gain < min_size:
            continue
        ok = clearance[rows, cols] >= goal_clearance
        if not ok.any():
            continue
        cr, cc = rows.mean(), cols.mean()
        i = np.argmin(np.where(ok, (rows - cr) ** 2 + (cols - cc) ** 2, np.inf))
        out.append({
            'goal': (origin[0] + (cols[i] + 0.5) * res, origin[1] + (rows[i] + 0.5) * res),
            'gain': gain,
            'cells': np.stack([origin[0] + (cols + 0.5) * res, origin[1] + (rows + 0.5) * res], 1),
        })
    return out


# ---------------------------------------------------------------- path costs

def path_costs(grid, res, origin, starts, goals, clearance_min):
    """Shortest 8-connected path length (m) from each start to each goal over cells that are
    free (or unknown) and at least clearance_min from occupied cells. inf if unreachable.
    A start or goal off the traversable set joins it at its nearest traversable cell."""
    h, w = grid.shape
    occupied = grid >= 50
    clearance = ndimage.distance_transform_edt(~occupied) * res
    trav = (clearance >= clearance_min) & ~occupied
    idx = -np.ones(grid.shape, np.int64)
    ti = np.flatnonzero(trav)
    idx.flat[ti] = np.arange(len(ti))
    if len(ti) == 0:
        return np.full((len(starts), len(goals)), np.inf)
    src, dst, wts = [], [], []
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):   # each undirected edge once
        # cell (r, c) and its neighbour (r + dr, c + dc), both inside the grid
        c0, c1 = (0, w - dc) if dc >= 0 else (-dc, w)
        a, b = trav[0:h - dr, c0:c1], trav[dr:h, c0 + dc:c1 + dc]
        m = a & b
        src.append(idx[0:h - dr, c0:c1][m])
        dst.append(idx[dr:h, c0 + dc:c1 + dc][m])
        wts.append(np.full(m.sum(), res * math.hypot(dr, dc)))
    src, dst, wts = np.concatenate(src), np.concatenate(dst), np.concatenate(wts)
    graph = csr_matrix((wts, (src, dst)), shape=(len(ti), len(ti)))
    tr, tc = np.divmod(ti, w)

    def snap(p):
        r = int(np.clip((p[1] - origin[1]) / res, 0, h - 1))
        c = int(np.clip((p[0] - origin[0]) / res, 0, w - 1))
        if trav[r, c]:
            return idx[r, c], 0.0
        j = np.argmin((tr - r) ** 2 + (tc - c) ** 2)
        return j, math.hypot(tr[j] - r, tc[j] - c) * res

    s_nodes = [snap(p) for p in starts]
    g_nodes = [snap(p) for p in goals]
    dist = dijkstra(graph, directed=False, indices=[s for s, _ in s_nodes])
    out = np.empty((len(starts), len(goals)))
    for i, (_, s_off) in enumerate(s_nodes):
        for j, (g, g_off) in enumerate(g_nodes):
            out[i, j] = dist[i, g] + s_off + g_off
    return out


def euclidean_costs(starts, goals):
    s, g = np.asarray(starts, float), np.asarray(goals, float)
    return np.hypot(s[:, None, 0] - g[None, :, 0], s[:, None, 1] - g[None, :, 1])


# ---------------------------------------------------------------- assignment

def assign(strategy, gains, cost, lam, radius, goals, bonus=None):
    """Return {robot index: frontier index} for the robots that get a frontier.

    bonus: optional (robots x frontiers) score bonus in m, used for hysteresis (a robot's
    current target); ignored by `nearest`, the plain baseline."""
    n_r, n_f = cost.shape
    if n_f == 0:
        return {}
    if bonus is None:
        bonus = np.zeros(cost.shape)
    score = gains[None, :] - lam * cost + bonus
    score[~np.isfinite(cost)] = -np.inf
    result = {}
    if strategy == 'nearest':
        for r in range(n_r):
            if np.isfinite(cost[r]).any():
                result[r] = int(np.argmin(cost[r]))
    elif strategy == 'greedy':
        g = gains.astype(float).copy()
        free_r = set(range(n_r))
        gx = np.asarray(goals, float)
        while free_r:
            s = g[None, :] - lam * cost + bonus
            s[~np.isfinite(cost)] = -np.inf
            s[:, g <= 0] = -np.inf     # already claimed (discounted to nothing): not worth a robot
            s[[r for r in range(n_r) if r not in free_r], :] = -np.inf
            if not np.isfinite(s).any():
                break
            r, f = np.unravel_index(np.argmax(s), s.shape)
            result[int(r)] = int(f)
            free_r.discard(int(r))
            d = np.hypot(gx[:, 0] - gx[f, 0], gx[:, 1] - gx[f, 1])
            g *= np.clip(d / radius, 0.0, 1.0)   # f itself -> 0 gain
    elif strategy == 'hungarian':
        big = 1e6
        c = np.where(np.isfinite(score), -score, big)
        rows, cols = optimize.linear_sum_assignment(c)
        for r, f in zip(rows, cols):
            if c[r, f] < big:
                result[int(r)] = int(f)
    else:
        raise ValueError(f'unknown strategy {strategy}')
    return result


# ---------------------------------------------------------------- node

class RobotState:
    def __init__(self, name):
        self.name = name
        self.target = None          # (x, y) in map of the active goal
        self.goal_handle = None
        self.goal_seq = 0           # bumps on every new goal; stale callbacks are ignored
        self.best_dist = math.inf   # progress tracking toward target
        self.last_progress = None
        self.home = None
        self.at_home = False


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        p = self.declare_parameter
        self.robots = [RobotState(r) for r in p('robots', ['robot1', 'robot2']).value]
        self.strategy = p('strategy', 'greedy').value
        self.period = p('period', 2.0).value                 # T
        self.lam = p('lambda', 0.5).value                    # λ
        self.radius = p('discount_radius', 1.5).value        # R
        self.resend = p('resend_dist', 0.5).value            # D
        self.hysteresis = p('hysteresis', 2.0).value         # score bonus for the current target, m
        self.min_size = p('min_size', 0.5).value
        self.goal_clearance = p('goal_clearance', 0.20).value
        self.path_clearance = p('path_clearance', 0.15).value
        self.cost_mode = p('cost_mode', 'path').value
        self.max_failures = p('max_failures', 2).value
        # A frontier still there this long after a robot reached its goal can't be cleared
        # from there (lidar shadow behind a box): skip it. Without this, a goal at the robot's
        # position is re-sent and "succeeds" in a loop (explore_lite did that in S8).
        self.reached_grace = p('reached_grace', 5.0).value
        self.unreachable_timeout = p('unreachable_timeout', 15.0).value
        self.unreachable_since = None
        self.progress_timeout = p('progress_timeout', 30.0).value
        self.done_cycles = p('done_cycles', 2).value
        self.return_home = p('return_home', True).value
        self.world_frame = p('world_frame', 'map').value
        self.debug_dir = p('debug_dir', '').value   # dump the grid when no frontiers are found
        self.stop_on_victim = p('stop_on_victim', True).value
        self.view_dists = list(p('view_dists', [1.2, 1.0, 1.5]).value)
        self.view_angles = list(p('view_angles', [0.0, 0.35, -0.35]).value)   # rad off the normal
        self.view_dwell = p('view_dwell', 3.0).value
        self.victim = None
        self.candidate = None        # (x, y, marker facing yaw)
        self.investigator = None     # RobotState sent to look, or None
        self.inv_tried = []          # viewpoints already used for this candidate
        self.inv_arrived = None      # time the investigator reached its viewpoint
        if self.strategy not in STRATEGIES:
            raise ValueError(f'strategy must be one of {STRATEGIES}')

        self.map = None
        self.failures = []           # [(x, y, count)]
        self.reached = []            # [(x, y, sim time first reached)] of frontier goals
        self.empty_cycles = 0
        self.started = False
        self.finished = False
        self.frontiers = []

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.nav_clients = {r.name: ActionClient(self, NavigateToPose, f'/{r.name}/navigate_to_pose')
                        for r in self.robots}
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(ExploreStatus, 'coordinator/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'coordinator/markers', 10)
        self.create_subscription(PointStamped, '/victim/confirmed', self.on_victim,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseStamped, '/victim/candidate', self.on_candidate, 10)
        self.timer = self.create_timer(self.period, self.step)
        self.get_logger().info(
            f'robots {[r.name for r in self.robots]}, strategy {self.strategy}, T {self.period} s, '
            f'lambda {self.lam}, R {self.radius} m, D {self.resend} m, hysteresis {self.hysteresis} m, '
            f'cost {self.cost_mode}')

    # ------------------------------------------------------------ helpers
    def on_map(self, msg):
        # A latched /map stamped in the future is from a previous run (sim time restarts at 0
        # every run) whose publisher hasn't died yet; it once ended a run early at 84 %.
        if Time.from_msg(msg.header.stamp) > self.get_clock().now() + Duration(seconds=2.0):
            self.get_logger().warn('ignoring a /map from the future (a stale publisher?)',
                                   throttle_duration_sec=5.0)
            return
        self.map = msg

    def on_victim(self, msg):
        if self.victim is not None or not self.started:
            return
        self.victim = (msg.point.x, msg.point.y)
        self.get_logger().info(f'victim confirmed at ({msg.point.x:.2f}, {msg.point.y:.2f})')
        self.publish_status('victim_confirmed')
        self.end_investigation()
        if self.stop_on_victim and not self.finished:
            self.finish()

    def on_candidate(self, msg):
        if self.victim is not None or self.finished:
            return
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        first = self.candidate is None
        self.candidate = (msg.pose.position.x, msg.pose.position.y, yaw)
        if first:
            self.get_logger().info(f'victim sighting at ({self.candidate[0]:.2f}, '
                                   f'{self.candidate[1]:.2f}), facing {math.degrees(yaw):.0f} deg: '
                                   'investigating')

    def end_investigation(self):
        if self.investigator is not None:
            self.investigator.target = None
            self.investigator.goal_seq += 1
            self.investigator = None
        self.candidate, self.inv_tried, self.inv_arrived = None, [], None

    def viewpoints(self, grid, res, origin):
        """Free viewpoints in front of the candidate, in preference order: (x, y, yaw to face it)."""
        cx, cy, nyaw = self.candidate
        clearance = ndimage.distance_transform_edt(grid < 50) * res
        out = []
        for d in self.view_dists:
            for da in self.view_angles:
                a = nyaw + da
                x, y = cx + d * math.cos(a), cy + d * math.sin(a)
                r, c = int((y - origin[1]) / res), int((x - origin[0]) / res)
                if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
                    continue
                if grid[r, c] != 0 or clearance[r, c] < self.goal_clearance:
                    continue
                if any(math.hypot(x - tx, y - ty) < 0.1 for tx, ty in self.inv_tried):
                    continue
                out.append((x, y, a + math.pi))
        return out

    def investigate(self, robots, grid, res, origin):
        """Drive the investigation; returns the robot busy with it (or None)."""
        if self.candidate is None or self.victim is not None:
            return None
        r = self.investigator
        if r is not None and r.target is not None:
            return r                                  # on its way
        now = self.get_clock().now()
        if r is not None and self.inv_arrived is not None:
            if (now - self.inv_arrived).nanoseconds * 1e-9 < self.view_dwell:
                return r                              # looking
            self.get_logger().info(f'{r.name}: no confirmation from this viewpoint')
        vps = self.viewpoints(grid, res, origin)
        if not vps:
            self.get_logger().info('victim sighting: no viewpoint left, back to exploring')
            self.end_investigation()
            return None
        if r is None:   # pick the robot with the cheapest path to the first viewpoint
            cost = path_costs(grid, res, origin, [xy for _, xy in robots], [vps[0][:2]],
                              self.path_clearance)[:, 0]
            if not np.isfinite(cost).any():
                self.end_investigation()
                return None
            r = robots[int(np.argmin(cost))][0]
            self.investigator = r
        x, y, yaw = vps[0]
        self.inv_tried.append((x, y))
        self.inv_arrived = None
        if self.send_goal(r, (x, y), yaw):
            self.get_logger().info(f'{r.name} -> viewpoint ({x:.2f}, {y:.2f}) to look at the victim')
        return r

    def pose(self, name):
        try:
            t = self.tf.lookup_transform(self.world_frame, f'{name}/base_footprint', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def home_of(self, r):
        if r.home is None:
            try:
                t = self.tf.lookup_transform(self.world_frame, f'{r.name}/map', Time())
                q = t.transform.rotation
                r.home = (t.transform.translation.x, t.transform.translation.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
            except TransformException:
                pass
        return r.home

    def failure_count(self, xy):
        return max((c for x, y, c in self.failures if math.hypot(x - xy[0], y - xy[1]) < 0.3),
                   default=0)

    def add_failure(self, xy):
        for i, (x, y, c) in enumerate(self.failures):
            if math.hypot(x - xy[0], y - xy[1]) < 0.3:
                self.failures[i] = (x, y, c + 1)
                return
        self.failures.append((xy[0], xy[1], 1))

    def publish_status(self, status):
        self.status_pub.publish(ExploreStatus(status=status))
        self.get_logger().info(f'status: {status}')

    # ------------------------------------------------------------ goals
    def send_goal(self, r, xy, yaw=0.0):
        client = self.nav_clients[r.name]
        if not client.server_is_ready():
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.world_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = xy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r.goal_seq += 1
        seq = r.goal_seq
        r.target = xy
        r.best_dist = math.inf
        r.last_progress = self.get_clock().now()
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f, r=r, seq=seq: self.on_goal_response(r, seq, f))
        return True

    def on_goal_response(self, r, seq, future):
        handle = future.result()
        if seq != r.goal_seq:
            return
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'{r.name}: goal rejected')
            r.target = None
            return
        r.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, r=r, seq=seq: self.on_result(r, seq, f))

    def on_result(self, r, seq, future):
        if seq != r.goal_seq:
            return   # superseded by a newer goal
        status = future.result().status
        target = r.target
        r.target, r.goal_handle = None, None
        if self.finished:
            if status == GoalStatus.STATUS_SUCCEEDED:
                r.at_home = True
                if all(x.at_home for x in self.robots):
                    self.publish_status(ExploreStatus.RETURNED_TO_ORIGIN)
            return
        if status == GoalStatus.STATUS_SUCCEEDED and target is not None:
            if not any(math.hypot(x - target[0], y - target[1]) < 0.3 for x, y, _ in self.reached):
                self.reached.append((target[0], target[1], self.get_clock().now()))
        if status == GoalStatus.STATUS_ABORTED and target is not None:
            self.add_failure(target)
            self.get_logger().info(f'{r.name}: goal ({target[0]:.2f}, {target[1]:.2f}) aborted '
                                   f'({self.failure_count(target)}x)')
        if r is self.investigator:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.inv_arrived = self.get_clock().now()
            return   # the next step() decides: dwell, next viewpoint or give up
        if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED):
            self.step()   # reassign now instead of idling until the next period

    # ------------------------------------------------------------ main loop
    def step(self):
        if self.finished or self.map is None:
            return
        m = self.map
        grid = np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width)
        origin = (m.info.origin.position.x, m.info.origin.position.y)
        res = m.info.resolution
        found = find_frontiers(grid, res, origin, self.min_size, self.goal_clearance)
        now = self.get_clock().now()
        stale = [(x, y) for x, y, t in self.reached
                 if (now - t).nanoseconds * 1e-9 > self.reached_grace]
        frontiers = [f for f in found if self.failure_count(f['goal']) < self.max_failures
                     and not any(math.hypot(f['goal'][0] - x, f['goal'][1] - y) < 0.3
                                 for x, y in stale)]
        self.get_logger().debug(f'frontiers: {len(found)} found, {len(frontiers)} not blacklisted')
        if not frontiers and self.debug_dir:
            path = f'{self.debug_dir}/no_frontiers_{self.get_clock().now().nanoseconds}.npz'
            np.savez(path, grid=grid, res=res, origin=origin, stamp=m.header.stamp.sec,
                     poses=np.array([xy for _, xy in [(r, self.pose(r.name)) for r in self.robots]
                                     if xy is not None]))
            self.get_logger().info(f'no frontiers: grid {grid.shape}, saved {path}')
        self.frontiers = frontiers
        robots = [(r, self.pose(r.name)) for r in self.robots]
        robots = [(r, xy) for r, xy in robots if xy is not None and self.nav_clients[r.name].server_is_ready()]
        if not robots:
            return
        if not self.started:
            self.started = True
            self.publish_status(ExploreStatus.EXPLORATION_STARTED)

        now = self.get_clock().now()
        for r, xy in robots:   # stalled goals count as failures
            if r.target is None:
                continue
            d = math.hypot(r.target[0] - xy[0], r.target[1] - xy[1])
            if d < r.best_dist - 0.1:
                r.best_dist, r.last_progress = d, now
            elif (now - r.last_progress).nanoseconds * 1e-9 > self.progress_timeout:
                self.get_logger().info(f'{r.name}: no progress toward '
                                       f'({r.target[0]:.2f}, {r.target[1]:.2f}), giving up')
                self.add_failure(r.target)
                r.target = None
                r.goal_seq += 1
                if r.goal_handle is not None:
                    r.goal_handle.cancel_goal_async()
                    r.goal_handle = None

        busy = self.investigate(robots, grid, res, origin)
        robots = [(r, xy) for r, xy in robots if r is not busy]

        if not frontiers:
            self.empty_cycles += 1
            if self.empty_cycles >= self.done_cycles and busy is None:
                self.finish()
            self.publish_markers(robots, {})
            return
        self.empty_cycles = 0
        if not robots:
            return

        starts = [xy for _, xy in robots]
        goals = [f['goal'] for f in frontiers]
        if self.cost_mode == 'path':
            cost = path_costs(grid, res, origin, starts, goals, self.path_clearance)
        else:
            cost = euclidean_costs(starts, goals)
        gains = np.array([f['gain'] for f in frontiers])
        bonus = np.zeros(cost.shape)
        for i, (r, _) in enumerate(robots):
            if r.target is not None:
                for j, g in enumerate(goals):
                    if math.hypot(g[0] - r.target[0], g[1] - r.target[1]) <= self.resend:
                        bonus[i, j] = self.hysteresis
        if not np.isfinite(cost).any():
            # Frontiers left, but no robot has a path to any of them (e.g. behind a gap under
            # path_clearance). No strategy can assign them, so without this the run waits forever
            # (seen in S8). Wait unreachable_timeout first: while the map updates, a frontier can
            # look unreachable for a moment.
            if self.unreachable_since is None:
                self.unreachable_since = now
                self.get_logger().info(f'{len(frontiers)} frontiers left, none reachable')
            elif (now - self.unreachable_since).nanoseconds * 1e-9 > self.unreachable_timeout \
                    and busy is None:
                self.get_logger().info('frontiers unreachable for '
                                       f'{self.unreachable_timeout:.0f} s: done')
                self.finish()
            self.publish_markers(robots, {})
            return
        self.unreachable_since = None
        plan = assign(self.strategy, gains, cost, self.lam, self.radius, goals, bonus)

        for i, (r, _) in enumerate(robots):
            if i not in plan:
                continue
            new = goals[plan[i]]
            if r.target is not None and math.hypot(new[0] - r.target[0], new[1] - r.target[1]) <= self.resend:
                continue
            if self.send_goal(r, new):
                self.get_logger().info(f'{r.name} -> frontier ({new[0]:.2f}, {new[1]:.2f}) '
                                       f'gain {gains[plan[i]]:.2f} m cost {cost[i, plan[i]]:.2f} m')
        self.publish_markers(robots, plan)

    def finish(self):
        self.finished = True
        self.publish_status(ExploreStatus.EXPLORATION_COMPLETE)
        self.publish_markers([], {})
        if not self.return_home:
            return
        self.publish_status(ExploreStatus.RETURNING_TO_ORIGIN)
        for r in self.robots:
            home = self.home_of(r)
            if home is not None:
                self.send_goal(r, home[:2], home[2])

    # ------------------------------------------------------------ markers
    def publish_markers(self, robots, plan):
        ma = MarkerArray()
        clear = Marker(action=Marker.DELETEALL)
        ma.markers.append(clear)
        stamp = self.get_clock().now().to_msg()
        for k, f in enumerate(self.frontiers):
            mk = Marker(type=Marker.POINTS, ns='frontier_cells', id=k, action=Marker.ADD)
            mk.header.frame_id, mk.header.stamp = self.world_frame, stamp
            mk.scale.x = mk.scale.y = 0.05
            mk.color = ColorRGBA(r=0.6, g=0.2, b=0.8, a=0.8)
            mk.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in f['cells']]
            ma.markers.append(mk)
            g = Marker(type=Marker.SPHERE, ns='frontier_goals', id=k, action=Marker.ADD)
            g.header.frame_id, g.header.stamp = self.world_frame, stamp
            g.pose.position.x, g.pose.position.y = f['goal']
            g.pose.orientation.w = 1.0
            g.scale.x = g.scale.y = g.scale.z = min(0.1 + 0.05 * f['gain'], 0.35)
            g.color = ColorRGBA(r=0.6, g=0.2, b=0.8, a=1.0)
            ma.markers.append(g)
        for i, (r, xy) in enumerate(robots):
            if r.target is None:
                continue
            c = ROBOT_COLOURS[self.robots.index(r) % len(ROBOT_COLOURS)]
            ln = Marker(type=Marker.LINE_STRIP, ns='assignment', id=i, action=Marker.ADD)
            ln.header.frame_id, ln.header.stamp = self.world_frame, stamp
            ln.scale.x = 0.03
            ln.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0)
            ln.points = [Point(x=xy[0], y=xy[1], z=0.05), Point(x=r.target[0], y=r.target[1], z=0.05)]
            ma.markers.append(ln)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
