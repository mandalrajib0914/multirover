#!/usr/bin/env python3
"""Localization error of one S8 trial: the robots' pose from the recorded /tf vs gz ground truth.

  python3 result/loc_error.py results/s8/<tag> [--plot out.png]

For each robot the pose in the world frame "map" is rebuilt from the bag's TF chain
map -> robotN/map (static, spawn pose) -> robotN/odom (slam_toolbox) -> robotN/base_footprint
(EKF) and compared, at every ground-truth sample in trajectories.csv, with Gazebo's pose.
Prints max and final xy error per robot. Used for the S8 map-failure analysis.
"""
import argparse
import bisect
import csv
import glob
import math
import os
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    return ax + math.cos(ath) * bx - math.sin(ath) * by, ay + math.sin(ath) * bx + math.cos(ath) * by, ath + bth


def read_tf(bag_dir):
    """{(parent, child): ([t], [(x, y, yaw)])} from /tf and /tf_static."""
    # the trial bags are file-compressed (zstd), so the compression-aware reader is needed
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static']))
    series = {}
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tr in deserialize_message(data, TFMessage).transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            p = tr.transform
            series.setdefault(key, ([], []))
            series[key][0].append(t)
            series[key][1].append((p.translation.x, p.translation.y, yaw_of(p.rotation)))
    for key, (ts, ps) in series.items():   # bags are not strictly time ordered across topics
        order = np.argsort(ts)
        series[key] = ([ts[i] for i in order], [ps[i] for i in order])
    return series


def at(series, key, t):
    ts, ps = series[key]
    i = bisect.bisect_right(ts, t) - 1
    return ps[max(i, 0)]


def trial_errors(run_dir):
    """{robot: [(t, xy error m)]} for one trial folder (computed from its bag)."""
    series = read_tf(os.path.join(run_dir, 'bag'))
    gt = {}
    with open(os.path.join(run_dir, 'trajectories.csv')) as f:
        for r in csv.DictReader(f):
            gt.setdefault(r['robot'], []).append((float(r['t_s']), float(r['x']), float(r['y'])))
    out = {}
    for robot, samples in gt.items():
        keys = [('map', f'{robot}/map'), (f'{robot}/map', f'{robot}/odom'),
                (f'{robot}/odom', f'{robot}/base_footprint')]
        if any(k not in series for k in keys):
            continue

        def pose(t):
            return compose(compose(at(series, keys[0], t), at(series, keys[1], t)),
                           at(series, keys[2], t))
        # trajectories.csv time is relative to exploration start; find the bag-clock offset
        # that best matches the first 20 samples (the robot starts well localized).
        t0 = series[keys[1]][0][0]
        best = min(np.arange(t0, t0 + 60, 0.5), key=lambda o: np.mean(
            [math.hypot(pose(t + o)[0] - x, pose(t + o)[1] - y) for t, x, y in samples[:20]]))
        out[robot] = [(t, math.hypot(pose(t + best)[0] - x, pose(t + best)[1] - y))
                      for t, x, y in samples]
    return out


def cached_max_error(run_dir):
    """Max localization error over robots and time (m), cached in <run_dir>/loc_error.yaml.
    Reading a zstd bag leaves a decompressed bag_0.mcap (~20 MB) next to it; that copy is
    deleted afterwards (3.5 GB for the S8 campaign otherwise)."""
    import yaml
    cache = os.path.join(run_dir, 'loc_error.yaml')
    if os.path.exists(cache):
        return yaml.safe_load(open(cache))['max_error_m']
    errs = trial_errors(run_dir)
    for f in glob.glob(os.path.join(run_dir, 'bag', '*.mcap')):
        if os.path.exists(f + '.zstd'):
            os.remove(f)
    per = {r: round(max(v for _, v in e), 3) for r, e in errs.items()}
    worst = max(per.values()) if per else None
    with open(cache, 'w') as f:
        yaml.safe_dump({'max_error_m': worst, 'per_robot_max_m': per}, f)
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--plot', default='')
    args = ap.parse_args()
    out = trial_errors(args.run_dir)
    for robot, errs in out.items():
        e = [v for _, v in errs]
        worst = max(errs, key=lambda z: z[1])
        print(f'{robot}: localization error mean {np.mean(e):.3f} m, max {worst[1]:.3f} m at '
              f't={worst[0]:.0f} s, final {e[-1]:.3f} m ({len(e)} samples)')
    if args.plot and out:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 3.5))
        for robot, errs in out.items():
            ax.plot([t for t, _ in errs], [v for _, v in errs], label=robot)
        ax.set_xlabel('time since start (s)')
        ax.set_ylabel('localization error (m)')
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)


if __name__ == '__main__':
    sys.exit(main())
