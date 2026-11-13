"""S8 campaign analysis: data/s8/<tag>/ -> results/tables/s8_* and results/figures/s8/.

Called by make_result.py. Each trial folder (written by exploration_monitor via
run_campaign.py) has summary.yaml, coverage.csv, trajectories.csv and trial_plan.yaml; the
tag is n<robots>_<strategy>_t<tt>_<start>_<victim>.

Metrics per condition (robots x strategy), over its trials:
  exploration time        start -> no frontiers (coordinator) / every explore_lite done
  time to 99 % of final   first coverage sample >= 99 % of the run's final coverage
  distance                all robots, Gazebo ground truth
  coverage, wall F1, free IoU   final merged map vs the ground truth (victim where the trial put it)
  victim                  confirmed or not; time to confirm; error at confirmation
  overlap                 share of visited 0.25 m cells visited by 2+ robots (n >= 2)
  localization            max over robots and time of |TF pose - gz pose| (loc_error.py, from
                          the bag); > LOC_FAIL counts as a localization failure
Time and distance use completed trials only (end_reason exploration_complete); "stalled"
(no coverage gain for 120 s, no robot moving for 60 s) and "timeout" are counted per
condition in the table. Intervals are 95 % t-intervals of the mean. Speed-ups are paired: the same trial layout
(start set, victim) with 1 robot vs n robots.
"""
import csv
import glob
import math
import os
import statistics as st

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402
import yaml  # noqa: E402

# Strategy colours: categorical palette, light mode, slots 1-4 in fixed order.
STRATEGIES = ['independent', 'nearest', 'greedy', 'hungarian']
COLOUR = {'independent': '#2a78d6', 'nearest': '#eb6834', 'greedy': '#1baf7a', 'hungarian': '#eda100'}
LABEL = {'independent': 'independent explore_lite (baseline)', 'nearest': 'nearest',
         'greedy': 'greedy', 'hungarian': 'hungarian'}
INK, INK_2, INK_3, GRID, SURFACE = '#0b0b0b', '#52514e', '#8a8984', '#e6e5e1', '#fcfcfb'
CELL = 0.25       # m, grid for trajectory overlap
LOC_FAIL = 0.30   # m, a robot's max localization error above this = a localization failure


def ci95(v):
    """(mean, half-width) of a 95 % t-interval; half-width 0 for a single value."""
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return float('nan'), float('nan')
    if len(v) == 1:
        return v[0], 0.0
    return st.mean(v), stats.t.ppf(0.975, len(v) - 1) * st.stdev(v) / math.sqrt(len(v))


def overlap(traj_file):
    """Share of visited cells (CELL m) that 2+ robots visited; None for a single robot."""
    cells = {}
    with open(traj_file) as f:
        for row in csv.DictReader(f):
            c = (int(float(row['x']) // CELL), int(float(row['y']) // CELL))
            cells.setdefault(c, set()).add(row['robot'])
    robots = set().union(*cells.values()) if cells else set()
    if len(robots) < 2 or not cells:
        return None
    return sum(len(v) >= 2 for v in cells.values()) / len(cells)


def load(root, folder='s8'):
    trials = []
    for f in sorted(glob.glob(os.path.join(root, 'data', folder, '*', 'summary.yaml'))):
        d = yaml.safe_load(open(f))
        folder = os.path.dirname(f)
        tag = os.path.basename(folder)
        n_s, strat, t_s, start, victim = tag.split('_', 4)
        m, v = d.get('map_vs_gt', {}), d.get('victim') or {}
        curve = []
        with open(os.path.join(folder, 'coverage.csv')) as fh:
            curve = [(float(r['t_s']), float(r['coverage'])) for r in csv.DictReader(fh)]
        traj = os.path.join(folder, 'trajectories.csv')
        loc_file = os.path.join(folder, 'loc_error.yaml')   # written by loc_error.py (from the bag)
        loc = yaml.safe_load(open(loc_file))['max_error_m'] if os.path.exists(loc_file) else None
        trials.append({
            'tag': tag, 'robots': int(n_s[1:]), 'strategy': strat, 'trial': int(t_s[1:]),
            'start': start, 'victim_id': victim, 'end_reason': d['end_reason'],
            'time_s': d['exploration_time_s'], 't99_s': d.get('time_to_99pct_of_final_s'),
            'distance_m': d['distance_m'], 'coverage': m.get('coverage'),
            'wall_f1': m.get('wall_F1'), 'free_iou': m.get('free_IoU'),
            'victim_confirmed': v.get('time_to_confirm_s') is not None,
            'victim_time_s': v.get('time_to_confirm_s'), 'victim_error_m': v.get('confirmed_error_m'),
            'overlap': overlap(traj) if os.path.exists(traj) else None,
            'loc_max_error_m': loc,
            'curve': curve,
        })
    return trials


def summarise(trials):
    rows = []
    for n in sorted({t['robots'] for t in trials}):
        for s in STRATEGIES:
            g = [t for t in trials if t['robots'] == n and t['strategy'] == s]
            if not g:
                continue
            # timeout / stalled (a robot stuck: see LOG) are not completed explorations
            done = [t for t in g if t['end_reason'] in ('exploration_complete', 'victim_confirmed')]
            tm, th = ci95([t['time_s'] for t in done])
            dm, dh = ci95([t['distance_m'] for t in done])
            vt, vh = ci95([t['victim_time_s'] for t in g if t['victim_confirmed']])
            errs = [t['victim_error_m'] for t in g if t['victim_confirmed']]
            ov = [t['overlap'] for t in g if t['overlap'] is not None]
            rows.append({
                'robots': n, 'strategy': s, 'trials': len(g), 'completed': len(done),
                'time_mean_s': round(tm, 1), 'time_ci95_s': round(th, 1),
                'time_median_s': round(st.median([t['time_s'] for t in done]), 1) if done else '',
                't99_mean_s': round(ci95([t['t99_s'] for t in done])[0], 1),
                'distance_mean_m': round(dm, 1), 'distance_ci95_m': round(dh, 1),
                'coverage_pct': round(100 * st.mean(t['coverage'] for t in g), 1),
                'wall_f1_pct': round(100 * st.mean(t['wall_f1'] for t in g), 1),
                'free_iou_pct': round(100 * st.mean(t['free_iou'] for t in g), 1),
                'victim_found': f"{sum(t['victim_confirmed'] for t in g)}/{len(g)}",
                'victim_time_mean_s': round(vt, 1) if errs else '',
                'victim_time_ci95_s': round(vh, 1) if errs else '',
                'victim_error_mean_cm': round(100 * st.mean(errs), 1) if errs else '',
                'victim_error_max_cm': round(100 * max(errs), 1) if errs else '',
                'overlap_pct': round(100 * st.mean(ov), 1) if ov else '',
                'loc_failures': (f"{sum(t['loc_max_error_m'] > LOC_FAIL for t in g if t['loc_max_error_m'] is not None)}"
                                 f"/{sum(t['loc_max_error_m'] is not None for t in g)}"),
                'loc_max_error_median_cm': round(100 * st.median(
                    [t['loc_max_error_m'] for t in g if t['loc_max_error_m'] is not None]), 1)
                if any(t['loc_max_error_m'] is not None for t in g) else '',
            })
    return rows


def paired_speedups(trials):
    """Per strategy and n >= 2: time(1 robot, same strategy family) / time(n robots), paired by
    trial index (same start set and victim). 1-robot hungarian doesn't exist: use greedy."""
    out = []
    trials = completed(trials)  # Both members of each speedup pair must have completed.
    one = {(t['strategy'], t['trial']): t['time_s'] for t in trials if t['robots'] == 1}
    for n in sorted({t['robots'] for t in trials if t['robots'] > 1}):
        for s in STRATEGIES:
            ratios = []
            for t in trials:
                if t['robots'] == n and t['strategy'] == s:
                    base = one.get(('greedy' if s == 'hungarian' else s, t['trial']))
                    if base:
                        ratios.append(base / t['time_s'])
            if ratios:
                m, h = ci95(ratios)
                out.append({'robots': n, 'strategy': s, 'pairs': len(ratios),
                            'speedup_mean': round(m, 2), 'speedup_ci95': round(h, 2)})
    return out


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_3)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def completed(trials):
    return [t for t in trials if t['end_reason'] in ('exploration_complete', 'victim_confirmed')]


def fig_time_vs_robots(trials, path):
    trials = completed(trials)
    ns = sorted({t['robots'] for t in trials})
    fig, ax = plt.subplots(figsize=(7.5, 4.6), facecolor=SURFACE)
    style(ax)
    for k, s in enumerate(STRATEGIES):
        xs, ms, hs = [], [], []
        for n in ns:
            v = [t['time_s'] for t in trials if t['robots'] == n and t['strategy'] == s]
            if v:
                m, h = ci95(v)
                xs.append(n + (k - 1.5) * 0.06)
                ms.append(m)
                hs.append(h)
                ax.scatter([n + (k - 1.5) * 0.06] * len(v), v, s=10, color=COLOUR[s], alpha=0.35,
                           linewidths=0, zorder=2)
        if xs:
            ax.errorbar(xs, ms, yerr=hs, color=COLOUR[s], linewidth=2, marker='o', markersize=7,
                        markeredgecolor=SURFACE, markeredgewidth=2, capsize=3, label=LABEL[s], zorder=3)
    ax.set_xticks(ns, [f'{n} robot{"s" if n > 1 else ""}' for n in ns])
    ax.set_ylabel('Exploration time (s, sim time)', color=INK_2, fontsize=9)
    ax.set_ylim(bottom=0)
    ax.set_title('Exploration time vs number of robots', loc='left', fontsize=11, color=INK, pad=10)
    ax.legend(fontsize=8, frameon=False, loc='upper right')
    fig.text(0.01, 0.01, 'Mean ± 95 % CI; faint dots are single trials (same 10 layouts per point). '
             '1-robot hungarian = greedy, not run.', fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    plt.close(fig)


def fig_strategy_comparison(trials, path):
    """Three separate panels (never one dual axis): time, distance, time to victim."""
    ns = sorted({t['robots'] for t in trials})
    done = completed(trials)
    metrics = [('time_s', 'Exploration time (s)'), ('distance_m', 'Distance, all robots (m)'),
               ('victim_time_s', 'Time to confirm victim (s)')]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.4), facecolor=SURFACE)
    w = 0.2
    for ax, (key, title) in zip(axes, metrics):
        style(ax)
        ax.grid(axis='x', visible=False)
        for k, s in enumerate(STRATEGIES):
            for n in ns:
                src = trials if key == 'victim_time_s' else done
                v = [t[key] for t in src if t['robots'] == n and t['strategy'] == s and t[key] is not None]
                if not v:
                    continue
                x = n + (k - 1.5) * w
                m, h = ci95(v)
                ax.bar(x, m, width=w * 0.9, color=COLOUR[s], edgecolor=SURFACE, linewidth=2,
                       label=LABEL[s] if (n == ns[-1] and ax is axes[0]) else None)
                ax.errorbar(x, m, yerr=h, color=INK, linewidth=1, capsize=2)
        ax.set_xticks(ns, [f'{n} robot{"s" if n > 1 else ""}' for n in ns])
        ax.set_title(title, loc='left', fontsize=10, color=INK)
    handles = [plt.Rectangle((0, 0), 1, 1, color=COLOUR[s]) for s in STRATEGIES]
    fig.legend(handles, [LABEL[s] for s in STRATEGIES], loc='upper center', ncol=4, fontsize=8.5,
               frameon=False, bbox_to_anchor=(0.5, 1.0))
    fig.text(0.01, 0.01, 'Mean ± 95 % CI over the same 10 trial layouts. Time to victim: confirmed '
             'trials only (see table for how many).', fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.04, 1, 0.93))
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    plt.close(fig)


def fig_coverage(trials, path):
    ns = sorted({t['robots'] for t in trials})
    fig, axes = plt.subplots(1, len(ns), figsize=(4.3 * len(ns), 4.2), facecolor=SURFACE, sharey=True)
    axes = np.atleast_1d(axes)
    grid_t = np.arange(0, 181, 2.0)
    for ax, n in zip(axes, ns):
        style(ax)
        for s in STRATEGIES:
            curves = []
            for t in trials:
                if t['robots'] == n and t['strategy'] == s and t['curve']:
                    tt, cv = zip(*t['curve'])
                    curves.append(np.interp(grid_t, tt, cv, right=cv[-1]))
            if not curves:
                continue
            c = np.array(curves) * 100
            m = c.mean(axis=0)
            h = stats.t.ppf(0.975, len(c) - 1) * c.std(axis=0, ddof=1) / np.sqrt(len(c)) if len(c) > 1 else 0 * m
            ax.fill_between(grid_t, m - h, m + h, color=COLOUR[s], alpha=0.15, linewidth=0)
            ax.plot(grid_t, m, color=COLOUR[s], linewidth=2, label=LABEL[s])
        ax.set_title(f'{n} robot{"s" if n > 1 else ""}', loc='left', fontsize=10, color=INK)
        ax.set_xlabel('Time since start (s)', color=INK_2, fontsize=9)
        ax.set_xlim(0, grid_t[-1])
        ax.set_ylim(0, 100)
    axes[0].set_ylabel('Floor mapped as free (%)', color=INK_2, fontsize=9)
    axes[-1].legend(fontsize=8, frameon=False, loc='lower right')
    fig.text(0.01, 0.01, 'Mean ± 95 % CI band; runs held at their final value after they finish.',
             fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path, dpi=160, facecolor=SURFACE)
    plt.close(fig)


def write_csv(path, rows):
    if not rows:
        return
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)


def write_md(path, summary, speedups):
    with open(path, 'w') as f:
        f.write('| Robots | Strategy | Trials | Exploration time (s) | Time to 99 % (s) | Distance (m) | '
                'Coverage | Wall F1 | Free IoU | Overlap | Victim found | Time to victim (s) | '
                'Victim error mean / max (cm) | Localization failures | Max loc. error, median (cm) |\n')
        f.write('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n')
        for r in summary:
            inc = '' if r['completed'] == r['trials'] else f" ({r['trials'] - r['completed']} not completed)"
            vt = f"{r['victim_time_mean_s']} ± {r['victim_time_ci95_s']}" if r['victim_time_mean_s'] != '' else '–'
            ve = f"{r['victim_error_mean_cm']} / {r['victim_error_max_cm']}" if r['victim_error_mean_cm'] != '' else '–'
            ov = f"{r['overlap_pct']} %" if r['overlap_pct'] != '' else '–'
            f.write(f"| {r['robots']} | {LABEL[r['strategy']]} | {r['trials']}{inc} | "
                    f"{r['time_mean_s']} ± {r['time_ci95_s']} | {r['t99_mean_s']} | "
                    f"{r['distance_mean_m']} ± {r['distance_ci95_m']} | {r['coverage_pct']} % | "
                    f"{r['wall_f1_pct']} % | {r['free_iou_pct']} % | {ov} | {r['victim_found']} | {vt} | {ve} | "
                    f"{r['loc_failures']} | {r['loc_max_error_median_cm']} |\n")
        if speedups:
            f.write('\n**Paired speed-up over 1 robot** (same trial layout; 1-robot greedy is the '
                    'reference for hungarian):\n\n| Robots | Strategy | Pairs | Speed-up (×) |\n|---|---|---|---|\n')
            for r in speedups:
                f.write(f"| {r['robots']} | {LABEL[r['strategy']]} | {r['pairs']} | "
                        f"{r['speedup_mean']} ± {r['speedup_ci95']} |\n")


def detector_comparison(root, out, aruco):
    """S7b: YOLO (doll) vs ArUco (box) on the same S8 trial layouts (data/s8_yolo vs s8)."""
    yolo = load(root, 's8_yolo')
    if not yolo:
        return
    by_tag = {t['tag']: t for t in aruco}
    pairs = [(by_tag[t['tag']], t) for t in yolo if t['tag'] in by_tag]
    rows = []
    keys = sorted({(a['robots'], a['strategy']) for a, _ in pairs})
    for n, s in keys:
        for vid in ['all', 'V1', 'V2', 'V3']:
            g = [(a, y) for a, y in pairs if a['robots'] == n and a['strategy'] == s
                 and (vid == 'all' or a['victim_id'] == vid)]
            if not g:
                continue
            row = {'robots': n, 'strategy': s, 'victim': vid, 'trials': len(g)}
            for name, idx in (('aruco', 0), ('yolo', 1)):
                found = [p[idx] for p in g if p[idx]['victim_confirmed']]
                tm, th = ci95([t['victim_time_s'] for t in found])
                row[f'{name}_found'] = f'{len(found)}/{len(g)}'
                row[f'{name}_time_s'] = f'{tm:.1f} ± {th:.1f}' if found else '–'
                row[f'{name}_err_cm'] = (f"{100 * st.mean(t['victim_error_m'] for t in found):.1f} / "
                                         f"{100 * max(t['victim_error_m'] for t in found):.1f}") if found else '–'
            both = [(a, y) for a, y in g if a['victim_confirmed'] and y['victim_confirmed']]
            if both:
                d = [a['victim_time_s'] - y['victim_time_s'] for a, y in both]
                m, h = ci95(d)
                row['paired_time_saved_s'] = f'{m:.1f} ± {h:.1f} (n={len(d)})'
            else:
                row['paired_time_saved_s'] = '–'
            yt = [y['time_s'] for _, y in g if y['end_reason'] in ('exploration_complete', 'victim_confirmed')]
            row['yolo_exploration_s'] = f'{st.mean(yt):.1f}' if yt else '–'
            rows.append(row)
    write_csv(os.path.join(out, 'tables', 's7b_yolo_vs_aruco.csv'), rows)
    with open(os.path.join(out, 'tables', 's7b_yolo_vs_aruco.md'), 'w') as f:
        f.write('| Robots | Strategy | Victim | Trials | Found ArUco / YOLO | Time to confirm ArUco (s) | '
                'Time to confirm YOLO (s) | Paired time saved by YOLO (s) | Error mean / max ArUco (cm) | '
                'Error mean / max YOLO (cm) |\n|---|---|---|---|---|---|---|---|---|---|\n')
        for r in rows:
            f.write(f"| {r['robots']} | {LABEL[r['strategy']]} | {r['victim']} | {r['trials']} | "
                    f"{r['aruco_found']} / {r['yolo_found']} | {r['aruco_time_s']} | {r['yolo_time_s']} | "
                    f"{r['paired_time_saved_s']} | {r['aruco_err_cm']} | {r['yolo_err_cm']} |\n")
    # figure: time to confirm, per condition, ArUco vs YOLO (paired layouts)
    fig, ax = plt.subplots(figsize=(8, 4.4), facecolor=SURFACE)
    style(ax)
    ax.grid(axis='x', visible=False)
    colours = {'aruco': '#2a78d6', 'yolo': '#eb6834'}
    labels = []
    for i, (n, s) in enumerate(keys):
        labels.append(f'{n} robot{"s" if n > 1 else ""}\n{s}')
        for k, (name, idx) in enumerate((('aruco', 0), ('yolo', 1))):
            v = [p[idx]['victim_time_s'] for p in pairs
                 if p[0]['robots'] == n and p[0]['strategy'] == s and p[idx]['victim_confirmed']]
            tot = sum(1 for p in pairs if p[0]['robots'] == n and p[0]['strategy'] == s)
            if not v:
                continue
            m, h = ci95(v)
            x = i + (k - 0.5) * 0.36
            ax.bar(x, m, width=0.32, color=colours[name], edgecolor=SURFACE, linewidth=2,
                   label={'aruco': 'ArUco marker (box)', 'yolo': 'YOLOv8n person (doll)'}[name] if i == 0 else None)
            ax.errorbar(x, m, yerr=h, color=INK, linewidth=1, capsize=2)
            ax.text(x, m + h + 1.5, f'{len(v)}/{tot}', ha='center', fontsize=8, color=INK_2)
    ax.set_xticks(range(len(keys)), labels, fontsize=9)
    ax.set_ylabel('Time to confirm the victim (s)', color=INK_2, fontsize=9)
    ax.set_title('Victim detection: YOLO vs ArUco on the same trial layouts', loc='left', fontsize=11,
                 color=INK, pad=10)
    ax.legend(fontsize=8.5, frameon=False, loc='upper right')
    fig.text(0.01, 0.01, 'Mean ± 95 % CI over trials where the victim was confirmed (count above each '
             'bar = confirmed / trials).', fontsize=7.5, color=INK_3)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(os.path.join(out, 'figures', 's8', 'yolo_vs_aruco.png'), dpi=160, facecolor=SURFACE)
    plt.close(fig)


def run(root, out):
    trials = load(root)
    if not trials:
        return 0
    tables, figs = os.path.join(out, 'tables'), os.path.join(out, 'figures', 's8')
    os.makedirs(tables, exist_ok=True)
    os.makedirs(figs, exist_ok=True)
    write_csv(os.path.join(tables, 's8_all_trials.csv'), [{k: v for k, v in t.items() if k != 'curve'} for t in trials])
    summary = summarise(trials)
    speedups = paired_speedups(trials)
    write_csv(os.path.join(tables, 's8_summary.csv'), summary)
    write_csv(os.path.join(tables, 's8_speedups.csv'), speedups)
    write_md(os.path.join(tables, 's8_summary.md'), summary, speedups)
    fig_time_vs_robots(trials, os.path.join(figs, 'time_vs_robots.png'))
    fig_strategy_comparison(trials, os.path.join(figs, 'strategy_comparison.png'))
    fig_coverage(trials, os.path.join(figs, 'coverage_curves.png'))
    detector_comparison(root, out, trials)
    return len(trials)
