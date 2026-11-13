"""Method and illustration figures for the report (called by make_result.py).

  figures/arena.png                    floor plan, start sets A/B, victims V1-V3
  figures/s4_nav_check_trajectories.png  S4 Nav2 tests (from data/s4/nav_check_trace.csv)
  figures/s8/trajectories_n3.png       3 robots, same layout: coordinated vs independent paths
  figures/s8/localization_failures.png localization error over time, the S8 failure trials
                                       (needs ROS sourced: reads the bags; skipped otherwise)
  figures/maps/s8/                     every S8 trial's map-vs-ground-truth overlay
"""
import csv
import glob
import math
import os
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrow, Rectangle  # noqa: E402
import yaml  # noqa: E402

INK, INK_2, INK_3, GRID, SURFACE = '#0b0b0b', '#52514e', '#8a8984', '#e6e5e1', '#fcfcfb'
# Robot identity: reference palette slots 1-3 (light mode); victims use slot 8 (red).
ROBOT = {'robot1': '#2a78d6', 'robot2': '#eb6834', 'robot3': '#1baf7a'}
VICTIM = '#e34948'
BOX = '#b9b8b2'


def draw_arena(ax, plan):
    for x1, y1, x2, y2 in plan['walls']:
        ax.plot([x1, x2], [y1, y2], color=INK, linewidth=2.5, solid_capstyle='butt', zorder=3)
    for cx, cy, sx, sy in plan.get('obstacles', []):
        ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, color=BOX, zorder=2))
    ax.set_aspect('equal')
    ax.set_xlim(-0.3, 6.3)
    ax.set_ylim(-0.3, 5.3)
    ax.set_facecolor(SURFACE)
    for side in ax.spines.values():
        side.set_visible(False)
    ax.tick_params(colors=INK_3, labelsize=8)


def fig_arena(root, out):
    plan = yaml.safe_load(open(os.path.join(root, 'src', 'mrs_gazebo', 'config', 'floor_plan.yaml')))
    sets = yaml.safe_load(open(os.path.join(root, 'src', 'mrs_experiments', 'config', 'trial_sets.yaml')))
    fig, ax = plt.subplots(figsize=(7.5, 6.6), facecolor=SURFACE)
    draw_arena(ax, plan)
    for name, (x, y) in {'hall': (0.75, 2.5), 'room B': (3.75, 0.25), 'room C': (2.1, 4.75),
                         'room D': (4.4, 4.75)}.items():
        ax.text(x, y, name, ha='center', va='center', fontsize=10, color=INK_2, style='italic')
    markers = {'A': 'o', 'B': 's'}
    for sname, poses in sets['start_sets'].items():
        for r, (x, y, yaw) in poses.items():
            ax.scatter(x, y, s=90, marker=markers[sname], color=ROBOT[r], edgecolor=SURFACE,
                       linewidth=1.5, zorder=5)
            ax.add_patch(FancyArrow(x, y, 0.22 * math.cos(yaw), 0.22 * math.sin(yaw), width=0.02,
                                    head_width=0.08, head_length=0.07, color=ROBOT[r], zorder=5))
        x0, y0 = list(poses.values())[0][:2]
        ax.text(x0 + (0.18 if sname == 'A' else -0.18), y0 - 0.22, f'start {sname}', fontsize=8.5,
                color=INK, ha='left' if sname == 'A' else 'right')
    for vname, (x, y, yaw) in sets['victims'].items():
        ax.add_patch(Rectangle((x - 0.1, y - 0.1), 0.2, 0.2, color=VICTIM, zorder=6))
        ax.add_patch(FancyArrow(x, y, 0.35 * math.cos(yaw), 0.35 * math.sin(yaw), width=0.02,
                                head_width=0.09, head_length=0.08, color=VICTIM, zorder=6))
        ax.text(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw) + (0.12 if abs(math.sin(yaw)) < 0.5 else 0),
                vname, fontsize=9, color=INK, ha='center', va='center', fontweight='bold')
    handles = [plt.Line2D([], [], marker='o', ls='', color=ROBOT[r], label=r) for r in ROBOT]
    handles += [plt.Line2D([], [], marker='s', ls='', color=VICTIM, label='victim (arrow: ArUco face)'),
                plt.Line2D([], [], marker='s', ls='', color=BOX, label='box obstacle (0.3 m tall)')]
    ax.legend(handles=handles, loc='upper center', bbox_to_anchor=(0.5, -0.06), ncol=3,
              fontsize=8.5, frameon=False)
    ax.set_title('Arena (6 × 5 m, 0.8 m doors), S8 start sets and victim positions', loc='left',
                 fontsize=11, color=INK)
    fig.text(0.01, 0.01, 'Circles: start set A (hall), squares: start set B (room B); arrows show '
             'the initial heading. Victim V1 is the one used in S2–S7.', fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(out, 'figures', 'arena.png'), dpi=160, facecolor=SURFACE)
    plt.close(fig)


def load_traj(run_dir):
    tr = {}
    with open(os.path.join(run_dir, 'trajectories.csv')) as f:
        for r in csv.DictReader(f):
            tr.setdefault(r['robot'], []).append((float(r['x']), float(r['y'])))
    return tr


def fig_trajectories(root, out, layout='t00_A_V1', n=3):
    plan = yaml.safe_load(open(os.path.join(root, 'src', 'mrs_gazebo', 'config', 'floor_plan.yaml')))
    pairs = [('greedy', 'coordinated (greedy)'), ('independent', 'independent explore_lite')]
    dirs = [os.path.join(root, 'results', 's8', f'n{n}_{s}_{layout}') for s, _ in pairs]
    if not all(os.path.exists(os.path.join(d, 'trajectories.csv')) for d in dirs):
        return
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.0), facecolor=SURFACE)
    for ax, d, (s, title) in zip(axes, dirs, pairs):
        draw_arena(ax, plan)
        summ = yaml.safe_load(open(os.path.join(d, 'summary.yaml')))
        for r, pts in sorted(load_traj(d).items()):
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=ROBOT[r], linewidth=1.6, alpha=0.85,
                    label=f"{r}: {summ['distance_per_robot_m'][r]:.1f} m", zorder=4)
            ax.scatter(*pts[0], s=40, color=ROBOT[r], edgecolor=SURFACE, zorder=5)
        ax.set_title(f"{title}: {summ['exploration_time_s']:.0f} s, {summ['distance_m']:.1f} m total",
                     loc='left', fontsize=10, color=INK)
        ax.legend(fontsize=8, frameon=False, loc='upper center', bbox_to_anchor=(0.5, -0.07), ncol=3)
    fig.text(0.01, 0.01, f'Same layout ({layout.replace("_", ", ")}), 3 robots, Gazebo ground-truth '
             'paths from start until exploration ended (the drive home is not included).', fontsize=7.5, color=INK_3)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.92, bottom=0.16, wspace=0.08)
    os.makedirs(os.path.join(out, 'figures', 's8'), exist_ok=True)
    fig.savefig(os.path.join(out, 'figures', 's8', f'trajectories_n{n}.png'), dpi=160, facecolor=SURFACE)
    plt.close(fig)


def fig_localization(root, out, threshold=0.30):
    try:
        import loc_error   # needs ROS (rosbag2_py) sourced
    except ImportError:
        print('localization figure skipped (source ROS to build it)')
        return
    fails = []
    for f in sorted(glob.glob(os.path.join(root, 'results', 's8', '*', 'loc_error.yaml'))):
        y = yaml.safe_load(open(f))
        if y['max_error_m'] and y['max_error_m'] > threshold:
            fails.append((os.path.dirname(f), max(y['per_robot_max_m'], key=y['per_robot_max_m'].get)))
    if not fails:
        return
    if any(not os.path.isdir(os.path.join(d, 'bag')) for d, _ in fails):
        print('localization figure skipped (recorded ROS bags are not available)')
        return
    fig, ax = plt.subplots(figsize=(8, 4.2), facecolor=SURFACE)
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.grid(color=GRID, linewidth=0.8)
    colours = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4']
    for (d, robot), c in zip(fails, colours):
        errs = loc_error.trial_errors(d)[robot]
        for mc in glob.glob(os.path.join(d, 'bag', '*.mcap')):   # decompressed copy
            if os.path.exists(mc + '.zstd'):
                os.remove(mc)
        ax.plot([t for t, _ in errs], [e for _, e in errs], color=c, linewidth=2,
                label=f'{os.path.basename(d)} ({robot})')
    ax.axhline(threshold, color=INK_3, linewidth=1, linestyle='--')
    ax.text(1, threshold + 0.04, f'failure threshold {threshold:.2f} m', fontsize=8, color=INK_3)
    ax.set_xlabel('Time since exploration start (s)', color=INK_2, fontsize=9)
    ax.set_ylabel('Localization error (m)', color=INK_2, fontsize=9)
    ax.set_title('S8 localization failures: sudden SLAM pose jumps', loc='left', fontsize=11, color=INK)
    ax.legend(fontsize=7.5, frameon=False, loc='upper left')
    fig.text(0.01, 0.01, 'Error = |pose from TF (map -> robotN/map -> robotN/odom -> base) - Gazebo '
             'ground truth|, for the worst robot of each failing trial.', fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(os.path.join(out, 'figures', 's8', 'localization_failures.png'), dpi=160, facecolor=SURFACE)
    plt.close(fig)


def copy_s8_maps(root, out):
    dst = os.path.join(out, 'figures', 'maps', 's8')
    os.makedirs(dst, exist_ok=True)
    for f in glob.glob(os.path.join(root, 'results', 's8', '*', 'map_vs_gt.png')):
        shutil.copy(f, os.path.join(dst, os.path.basename(os.path.dirname(f)) + '.png'))
    for f in glob.glob(os.path.join(root, 'results', 's7', '*', 'map_vs_gt.png')):
        os.makedirs(os.path.join(out, 'figures', 'maps', 's7'), exist_ok=True)
        shutil.copy(f, os.path.join(out, 'figures', 'maps', 's7',
                                    os.path.basename(os.path.dirname(f)) + '.png'))


def fig_s4(root, out):
    """S4 nav_check trajectories, from the saved trace (result/data/s4)."""
    import subprocess
    import sys
    data = os.path.join(out, 'data', 's4')
    if os.path.exists(os.path.join(data, 'nav_check_trace.csv')):
        subprocess.run([sys.executable, os.path.join(data, 'plot_nav_trace.py'),
                        os.path.join(data, 'nav_check_trace.csv'),
                        os.path.join(out, 'figures', 's4_nav_check_trajectories.png'),
                        '0.2', '5.8', '0', '2.4'], cwd=root, check=True)


def run(root, out):
    os.makedirs(os.path.join(out, 'figures'), exist_ok=True)
    fig_arena(root, out)
    fig_s4(root, out)
    fig_trajectories(root, out)
    copy_s8_maps(root, out)
    fig_localization(root, out)
