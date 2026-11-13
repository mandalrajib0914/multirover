#!/usr/bin/env python3
"""Rebuild the report-ready tables and figures in result/ from the raw runs in results/.

  cd ~/Desktop/multirover && python3 result/make_result.py

Reads results/s5/*/ and results/s6/*/ (results/excluded/ is ignored; one folder per run, written by exploration_monitor:
summary.yaml, coverage.csv, map.pgm/.yaml, map_vs_gt.png) and writes:
  result/tables/all_runs.csv            one row per run
  result/tables/summary_by_config.csv   mean / sd / min / max per (robots, strategy)
  result/tables/summary_by_config.md    the same, as a Markdown table for the report
  result/figures/exploration_time_by_config.png
  result/figures/distance_by_config.png
  result/figures/coverage_vs_time.png
  result/tables/victim_missions.csv/.md S7: time to confirm the victim and its position error
  result/figures/maps/                  map-vs-ground-truth overlays of selected runs
  result/tables/s8_*, result/figures/s8/ S8 campaign (results/s8, see s8_analysis.py)
  result/figures/arena.png, figures/s8/trajectories_n3.png, figures/s8/localization_failures.png,
  result/figures/maps/{s7,s8}/          (report_figures.py; the localization figure needs ROS sourced)
New runs are picked up automatically, so re-run this after S8.

Colours: the categorical palette, light mode, fixed order by
robot count (1, 2, 3 robots) and slot 4 for the uncoordinated baseline.
"""
import csv
import glob
import os
import shutil
import statistics as st

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

import report_figures  # noqa: E402  (result/report_figures.py)
import s8_analysis  # noqa: E402  (result/s8_analysis.py)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'result')

# Categorical palette, light mode, slots 1-4 in order.
C_1ROBOT, C_2ROBOTS, C_3ROBOTS, C_BASELINE = '#2a78d6', '#eb6834', '#1baf7a', '#eda100'
INK, INK_2, INK_3, GRID, SURFACE = '#0b0b0b', '#52514e', '#8a8984', '#e6e5e1', '#fcfcfb'

# Tags of runs made before a behaviour change, kept apart so tables stay honest.
HYSTERESIS_TAGS = ('gh', 'hh')      # S6 runs with hysteresis 2.0 (earlier ones had none)


def config_of(d):
    """(robots, label) of one run's summary."""
    n = d.get('n_robots', 1)
    strategy = d.get('strategy') or 'explore_lite'
    tag = d.get('tag') or ''
    if strategy == 'independent':
        label = 'independent explore_lite'
    elif strategy == 'explore_lite':
        label = 'explore_lite'
    else:
        label = strategy + (' + hysteresis' if tag.startswith(HYSTERESIS_TAGS) else '')
    return n, label


def load_runs():
    runs = []
    for step in ('s5', 's6'):
        for f in sorted(glob.glob(os.path.join(ROOT, 'results', step, '*', 'summary.yaml'))):
            d = yaml.safe_load(open(f))
            folder = os.path.dirname(f)
            n, label = config_of(d)
            cov = []
            cov_file = os.path.join(folder, 'coverage.csv')
            if os.path.exists(cov_file):
                with open(cov_file) as fh:
                    cov = [(float(r['t_s']), float(r['coverage'])) for r in csv.DictReader(fh)]
            m = d.get('map_vs_gt', {})
            runs.append({
                'step': step, 'run': os.path.basename(folder), 'folder': folder,
                'robots': n, 'config': label, 'tag': d.get('tag') or '',
                'end_reason': d['end_reason'],
                'time_s': d['exploration_time_s'], 'distance_m': d['distance_m'],
                'time_to_90pct_s': d.get('time_to_90pct_s'),
                'time_to_99pct_of_final_s': d.get('time_to_99pct_of_final_s'),
                'coverage': m.get('coverage'), 'wall_precision': m.get('wall_precision'),
                'wall_recall': m.get('wall_recall'), 'curve': cov,
            })
    return runs


def colour(n, label):
    if label.startswith('independent'):
        return C_BASELINE
    return {1: C_1ROBOT, 2: C_2ROBOTS, 3: C_3ROBOTS}.get(n, INK_2)


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_3)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def write_tables(runs, groups):
    os.makedirs(os.path.join(OUT, 'tables'), exist_ok=True)
    fields = ['step', 'run', 'robots', 'config', 'tag', 'end_reason', 'time_s', 'distance_m',
              'time_to_90pct_s', 'time_to_99pct_of_final_s', 'coverage', 'wall_precision',
              'wall_recall']
    with open(os.path.join(OUT, 'tables', 'all_runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in runs:
            w.writerow(r)

    rows = []
    for (n, label), g in groups:
        t = [r['time_s'] for r in g]
        t99 = [r['time_to_99pct_of_final_s'] for r in g if r['time_to_99pct_of_final_s']]
        rows.append({
            'robots': n, 'config': label, 'runs': len(g),
            'time_mean_s': round(st.mean(t), 1),
            'time_sd_s': round(st.stdev(t), 1) if len(t) > 1 else '',
            'time_min_s': min(t), 'time_max_s': max(t),
            't99_mean_s': round(st.mean(t99), 1) if t99 else '',
            'distance_mean_m': round(st.mean(r['distance_m'] for r in g), 1),
            'coverage_mean_pct': round(100 * st.mean(r['coverage'] for r in g), 1),
            'wall_recall_mean_pct': round(100 * st.mean(r['wall_recall'] for r in g), 1),
        })
    with open(os.path.join(OUT, 'tables', 'summary_by_config.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(OUT, 'tables', 'summary_by_config.md'), 'w') as f:
        f.write('| Robots | Strategy | Runs | Time mean ± sd (s) | Min–max (s) | '
                'Time to 99 % of final coverage (s) | Distance (m) | Coverage | Wall recall |\n')
        f.write('|---|---|---|---|---|---|---|---|---|\n')
        for r in rows:
            sd = f" ± {r['time_sd_s']}" if r['time_sd_s'] != '' else ''
            f.write(f"| {r['robots']} | {r['config']} | {r['runs']} | {r['time_mean_s']}{sd} | "
                    f"{r['time_min_s']:.0f}–{r['time_max_s']:.0f} | {r['t99_mean_s']} | "
                    f"{r['distance_mean_m']} | {r['coverage_mean_pct']} % | "
                    f"{r['wall_recall_mean_pct']} % |\n")
    return rows


def bar_figure(groups, key, xlabel, title, path):
    """Horizontal bars (mean) with every run as a dot; one bar per configuration."""
    labels = [f'{n} robot{"s" if n > 1 else ""} · {label}' for (n, label), _ in groups]
    fig, ax = plt.subplots(figsize=(8, 0.45 * len(groups) + 1.4), facecolor=SURFACE)
    style(ax)
    ax.grid(axis='y', visible=False)
    xmax = max(r[key] for _, g in groups for r in g) * 1.25
    ax.set_xlim(0, xmax)
    y = np.arange(len(groups))[::-1]
    for yi, ((n, label), g) in zip(y, groups):
        vals = [r[key] for r in g]
        c = colour(n, label)
        ax.barh(yi, st.mean(vals), height=0.55, color=c, edgecolor=SURFACE, linewidth=2)
        ax.scatter(vals, [yi] * len(vals), s=22, color=SURFACE, edgecolor=INK, linewidth=1,
                   zorder=3)
        ax.text(max(vals) + 0.02 * xmax, yi, f'{st.mean(vals):.1f}  (n={len(vals)})',
                va='center', fontsize=8.5, color=INK_2)
    ax.set_yticks(y, labels, fontsize=9, color=INK)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9)
    ax.set_title(title, loc='left', fontsize=11, color=INK, pad=10)
    fig.text(0.01, 0.01, 'Bar = mean, dots = individual runs. Headless Gazebo, one 6 × 5 m arena.',
             fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    plt.close(fig)


def coverage_figure(runs, path):
    """Coverage vs time: thin line per run, thick mean per configuration (resampled)."""
    show = [(1, 'greedy'), (2, 'greedy'), (3, 'greedy'), (2, 'independent explore_lite')]
    fig, ax = plt.subplots(figsize=(8, 4.6), facecolor=SURFACE)
    style(ax)
    grid_t = np.arange(0, 141, 2.0)
    for n, label in show:
        sel = [r for r in runs if r['robots'] == n and (r['config'] == label or
               (label == 'greedy' and r['config'] == 'greedy + hysteresis')) and r['curve']]
        if not sel:
            continue
        c = colour(n, label)
        curves = []
        for r in sel:
            t, cv = zip(*r['curve'])
            ax.plot(t, np.array(cv) * 100, color=c, linewidth=0.8, alpha=0.3)
            curves.append(np.interp(grid_t, t, cv, right=cv[-1]))   # hold final value after done
        mean = np.mean(curves, axis=0) * 100
        ax.plot(grid_t, mean, color=c, linewidth=2)
    ax.set_xlim(0, grid_t[-1])
    ax.set_ylim(0, 100)
    ax.set_xlabel('Time since exploration start (s, sim time)', color=INK_2, fontsize=9)
    ax.set_ylabel('Floor mapped as free (%)', color=INK_2, fontsize=9)
    ax.set_title('Coverage of the true free space over time', loc='left', fontsize=11, color=INK, pad=10)
    handles = [plt.Line2D([], [], color=colour(n, l), linewidth=2) for n, l in show]
    ax.legend(handles, ['1 robot, greedy', '2 robots, greedy', '3 robots, greedy',
                        '2 robots, independent explore_lite'],
              loc='lower right', fontsize=8, frameon=False)
    fig.text(0.01, 0.01, 'Thin = each run, thick = mean (runs held at their final value after they '
             'finish). Greedy includes runs with and without hysteresis.', fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    plt.close(fig)


def victim_table():
    """S7 runs (results/s7): kept out of the exploration tables because a mission that stops
    at the victim doesn't explore everything."""
    rows = []
    for f in sorted(glob.glob(os.path.join(ROOT, 'results', 's7', '*', 'summary.yaml'))):
        d = yaml.safe_load(open(f))
        v = d.get('victim') or {}
        rows.append({
            'run': os.path.basename(os.path.dirname(f)), 'robots': d.get('n_robots', 1),
            'strategy': d.get('strategy', ''),
            'stops_at_victim': d['end_reason'] == 'victim_confirmed',
            'end_reason': d['end_reason'], 'mission_time_s': d['exploration_time_s'],
            'time_to_confirm_s': v.get('time_to_confirm_s'),
            'confirmed_error_m': v.get('confirmed_error_m'), 'final_error_m': v.get('final_error_m'),
            'coverage_at_end': d['map_vs_gt'].get('coverage'), 'distance_m': d['distance_m'],
        })
    if not rows:
        return
    with open(os.path.join(OUT, 'tables', 'victim_missions.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(OUT, 'tables', 'victim_missions.md'), 'w') as f:
        f.write('| Run | Robots | Ends at victim | Time to confirm (s) | Error at confirm (m) | '
                'Error at end (m) | Mission time (s) | Coverage at end |\n|---|---|---|---|---|---|---|---|\n')
        for r in rows:
            fmt = lambda v, p='': '–' if v is None else f'{v}{p}'   # noqa: E731
            cov = '–' if r['coverage_at_end'] is None else f"{100 * r['coverage_at_end']:.1f} %"
            f.write(f"| {r['run']} | {r['robots']} | {'yes' if r['stops_at_victim'] else 'no'} | "
                    f"{fmt(r['time_to_confirm_s'])} | {fmt(r['confirmed_error_m'])} | "
                    f"{fmt(r['final_error_m'])} | {r['mission_time_s']} | {cov} |\n")


def copy_maps(runs):
    """Map-vs-ground-truth overlays: the S3 mapping-tour map and each run's saved map."""
    dst = os.path.join(OUT, 'figures', 'maps')
    os.makedirs(dst, exist_ok=True)
    s3 = os.path.join(ROOT, 'src', 'mrs_navigation', 'maps', 'floor_v1_vs_gt.png')
    if os.path.exists(s3):
        shutil.copy(s3, os.path.join(dst, 's3_floor_v1_tour_vs_gt.png'))
    for r in runs:
        src = os.path.join(r['folder'], 'map_vs_gt.png')
        if os.path.exists(src):
            name = f"{r['step']}_{r['robots']}robot_{r['config'].replace(' ', '_').replace('+', 'plus')}_{r['run']}.png"
            shutil.copy(src, os.path.join(dst, name))


def main():
    runs = load_runs()
    keys = sorted({(r['robots'], r['config']) for r in runs})
    groups = [(k, [r for r in runs if (r['robots'], r['config']) == k]) for k in keys]
    os.makedirs(os.path.join(OUT, 'figures'), exist_ok=True)
    write_tables(runs, groups)
    bar_figure(groups, 'time_s', 'Exploration time (s, sim time): start until no frontiers remain',
               'Exploration time by robots and strategy',
               os.path.join(OUT, 'figures', 'exploration_time_by_config.png'))
    bar_figure(groups, 'distance_m', 'Total distance driven by all robots (m, Gazebo ground truth)',
               'Distance driven by robots and strategy',
               os.path.join(OUT, 'figures', 'distance_by_config.png'))
    coverage_figure(runs, os.path.join(OUT, 'figures', 'coverage_vs_time.png'))
    copy_maps(runs)
    victim_table()
    report_figures.run(ROOT, OUT)
    n8 = s8_analysis.run(ROOT, OUT)
    if n8:
        print(f'S8: {n8} trials -> tables/s8_*, figures/s8/')
    print(f'{len(runs)} runs, {len(groups)} configurations -> {OUT}')


if __name__ == '__main__':
    main()
