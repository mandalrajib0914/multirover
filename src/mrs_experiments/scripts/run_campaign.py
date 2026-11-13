#!/usr/bin/env python3
"""Run the S8 experiment campaign unattended (resumable).

  cd ~/Desktop/multirover && ros2 run mrs_experiments run_campaign.py [--trials 10]
      [--only n2_greedy] [--dry-run] [--retry-failed] [--timeout 900]

For every trial t and condition (robots x strategy) in config/trial_sets.yaml it writes a
trial plan (start set + victim) and runs trial.launch.py headless into
data/s8/<tag>/, tag = n<robots>_<strategy>_t<tt>_<start>_<victim>. The loop is trial-major
(all conditions for trial 0, then trial 1, ...), so a campaign stopped early is still
balanced across conditions. Trials with a summary.yaml are skipped; a trial that ends
without one gets failed.txt and is skipped next time unless --retry-failed.
Between trials every leftover sim/ROS process is killed: a stale latched /map from a dying
process once corrupted a run (S6).
"""
import argparse
import datetime
import gzip
import math
import os
import shutil
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import yaml

LEFTOVERS = ['gz sim', 'ros2 launch mrs_', 'ros2 bag record', 'controller_server', 'planner_server',
             'slam_toolbox', 'mrs_exploration', 'mrs_bringup', 'mrs_perception', 'parameter_bridge',
             'image_bridge', 'ekf_node', 'robot_state_publisher', 'bt_navigator', 'lifecycle_manager',
             'behavior_server', 'velocity_smoother', 'collision_monitor', 'smoother_server',
             'waypoint_follower', 'static_transform', 'explore_lite', 'map_saver']


def ancestors():
    """This process and every parent up to init: never killed, even if a parent's command
    line happens to contain one of the patterns (it did: a shell with a path to mrs_bringup)."""
    pids, pid = set(), os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            with open(f'/proc/{pid}/stat') as f:
                pid = int(f.read().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def leftover_pids():
    keep = ancestors()
    out = subprocess.run(['pgrep', '-f', '|'.join(LEFTOVERS)], capture_output=True, text=True).stdout
    return [int(p) for p in out.split() if int(p) not in keep]


def kill_leftovers():
    for p in leftover_pids():
        try:
            os.kill(p, signal.SIGINT)
        except ProcessLookupError:
            pass
    for _ in range(15):
        if not leftover_pids():
            return 0
        time.sleep(1)
    left = leftover_pids()
    for p in left:
        try:
            os.kill(p, signal.SIGKILL)
        except ProcessLookupError:
            pass
    time.sleep(1)
    return len(left)


# The victim poses in trial_sets.yaml were chosen for the 0.2 m ArUco box (back 3 cm from the
# wall). The YOLO doll (models/victim_person) is 0.41 m long along its facing direction, so at
# the same centre its back went 8 cm into the wall at V2/V3: the camera saw a shifted, partly
# hidden doll and the "truth" was a buried centre (errors up to 28 cm). For the doll, the centre
# moves forward so its back is where the box's back was: 0.41/2 - 0.2/2 along the facing yaw.
DOLL_SHIFT = 0.41 / 2 - 0.20 / 2


def trial_plan(sets, t, n, victim_kind='aruco'):
    start = sets['start_order'][t % len(sets['start_order'])]
    victim = sets['victim_order'][t % len(sets['victim_order'])]
    with open(os.path.join(get_package_share_directory('mrs_gazebo'), 'config', 'floor_plan.yaml')) as f:
        plan = yaml.safe_load(f)
    plan['spawn_poses'] = {r: p for r, p in sets['start_sets'][start].items()
                           if int(r.replace('robot', '')) <= n}
    x, y, yaw = sets['victims'][victim]
    if victim_kind == 'yolo':
        x, y = x + DOLL_SHIFT * math.cos(yaw), y + DOLL_SHIFT * math.sin(yaw)
    plan['victim'] = [round(x, 4), round(y, 4), yaw]
    return start, victim, plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sets', default=os.path.join(
        get_package_share_directory('mrs_experiments'), 'config', 'trial_sets.yaml'))
    ap.add_argument('--results', default='', help='default: data/s8, or data/s8_yolo')
    ap.add_argument('--victim', default='aruco', choices=['aruco', 'yolo'],
                    help='aruco: box + ArUco detector (S8); yolo: doll + YOLO detector (S7b)')
    ap.add_argument('--stop-on-victim', action='store_true', help='end each trial at the victim')
    ap.add_argument('--trials', type=int, default=None, help='default: trials in the sets file')
    ap.add_argument('--only', default='',
                    help='run only tags containing one of these (comma-separated), e.g. n2_greedy,n3_greedy')
    ap.add_argument('--timeout', type=int, default=900, help='wall seconds per trial')
    ap.add_argument('--retry-failed', action='store_true')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    with open(args.sets) as f:
        sets = yaml.safe_load(f)
    trials = args.trials or sets['trials']
    args.results = args.results or ('data/s8_yolo' if args.victim == 'yolo' else 'data/s8')
    only = [o for o in args.only.split(',') if o]
    os.makedirs(args.results, exist_ok=True)
    todo = []
    for t in range(trials):
        for c in sets['conditions']:
            n, strat = c['robots'], c['strategy']
            start, victim, plan = trial_plan(sets, t, n, args.victim)
            tag = f'n{n}_{strat}_t{t:02d}_{start}_{victim}'
            if only and not any(o in tag for o in only):
                continue
            todo.append((tag, n, strat, plan))

    done = sum(os.path.exists(os.path.join(args.results, t, 'summary.yaml')) for t, *_ in todo)
    print(f'{len(todo)} trials, {done} already done', flush=True)
    for i, (tag, n, strat, plan) in enumerate(todo):
        run_dir = os.path.abspath(os.path.join(args.results, tag))
        if os.path.exists(os.path.join(run_dir, 'summary.yaml')):
            continue
        if os.path.exists(os.path.join(run_dir, 'failed.txt')) and not args.retry_failed:
            continue
        print(f'[{i + 1}/{len(todo)}] {tag} start {datetime.datetime.now():%H:%M:%S}', flush=True)
        if args.dry_run:
            continue
        if os.path.exists(run_dir):
            shutil.rmtree(run_dir)   # a half-written earlier attempt
        os.makedirs(run_dir)
        plan_file = os.path.join(run_dir, 'trial_plan.yaml')
        with open(plan_file, 'w') as f:
            yaml.safe_dump(plan, f, sort_keys=False)
        t0 = time.time()
        log_path = os.path.join(run_dir, 'launch.log')
        with open(log_path, 'w') as log:
            try:
                rc = subprocess.run(
                    ['ros2', 'launch', 'mrs_experiments', 'trial.launch.py', f'n:={n}',
                     f'strategy:={strat}', f'plan_file:={plan_file}', f'run_dir:={run_dir}',
                     f'tag:={tag}', f'victim:={args.victim}',
                     f'stop_on_victim:={str(args.stop_on_victim).lower()}'],
                    stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout,
                    start_new_session=True).returncode
            except subprocess.TimeoutExpired:
                rc = 'timeout'
        left = kill_leftovers()
        with open(log_path, 'rb') as src, gzip.open(log_path + '.gz', 'wb') as dst:
            shutil.copyfileobj(src, dst)
        os.remove(log_path)
        ok = os.path.exists(os.path.join(run_dir, 'summary.yaml'))
        if not ok:
            with open(os.path.join(run_dir, 'failed.txt'), 'w') as f:
                f.write(f'launch returned {rc}; no summary.yaml\n')
        s = yaml.safe_load(open(os.path.join(run_dir, 'summary.yaml'))) if ok else {}
        v = (s.get('victim') or {})
        print(f'    {"ok" if ok else "FAILED"} in {time.time() - t0:.0f} s wall: '
              f'{s.get("end_reason")}, time {s.get("exploration_time_s")} s, '
              f'coverage {s.get("map_vs_gt", {}).get("coverage")}, '
              f'victim {v.get("time_to_confirm_s")} s / {v.get("confirmed_error_m")} m, '
              f'leftovers killed {left}', flush=True)
    print('CAMPAIGN DONE', flush=True)


if __name__ == '__main__':
    main()
