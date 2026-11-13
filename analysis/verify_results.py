#!/usr/bin/env python3
"""Check headline results and export paired statistical tests from the raw trial data.

Run from the repository root: python3 analysis/verify_results.py
This reads measurements; it does not run simulation or require the ROS bags.
"""
from pathlib import Path
import statistics

import numpy as np
from scipy import stats

import s8_analysis as analysis


def compare(trials, first, second):
    def group(condition):
        return {t['trial']: t for t in analysis.completed(trials)
                if (t['robots'], t['strategy']) == condition}
    a, b = group(first), group(second)
    pairs = sorted(a.keys() & b.keys())
    differences = np.array([a[k]['time_s'] - b[k]['time_s'] for k in pairs])
    p = stats.wilcoxon(differences, method='exact' if np.all(differences != 0)
                      else 'approx').pvalue
    return {'first': f'{first[0]} {first[1]}', 'second': f'{second[0]} {second[1]}',
            'pairs': len(pairs), 'median_difference_s': round(float(np.median(differences)), 1),
            'second_faster': int((differences > 0).sum()), 'p': float(p)}


def main():
    root = Path(__file__).resolve().parents[1]
    aruco = analysis.load(str(root))
    yolo = analysis.load(str(root), 's8_yolo')
    assert len(aruco) == 110 and len(yolo) == 40
    assert len(analysis.completed(aruco)) == 109
    assert len(analysis.completed(yolo)) == 39
    assert sum(t['victim_confirmed'] for t in aruco) == 100
    assert sum(t['victim_confirmed'] for t in yolo) == 40
    # Submitted Table 1: completed-trial mean exploration time and 95% CI half-width.
    expected = {
        (1, 'independent'): (87.8, 11.4), (1, 'nearest'): (108.7, 29.8),
        (1, 'greedy'): (99.2, 21.6), (2, 'independent'): (115.1, 21.4),
        (2, 'nearest'): (74.2, 13.2), (2, 'greedy'): (64.4, 14.1),
        (2, 'hungarian'): (58.2, 7.7), (3, 'independent'): (132.2, 18.9),
        (3, 'nearest'): (58.0, 5.2), (3, 'greedy'): (47.0, 5.6),
        (3, 'hungarian'): (48.6, 9.9),
    }
    for row in analysis.summarise(aruco):
        condition = row['robots'], row['strategy']
        assert (row['time_mean_s'], row['time_ci95_s']) == expected[condition], condition
    speedups = {(r['robots'], r['strategy']): r for r in analysis.paired_speedups(aruco)}
    for condition, mean, ci in [((2, 'nearest'), 1.61, 0.65), ((3, 'nearest'), 1.96, 0.61)]:
        r = speedups[condition]
        assert (r['pairs'], r['speedup_mean'], r['speedup_ci95']) == (9, mean, ci), r

    comparisons = []
    for n in (2, 3):
        comparisons.extend(('Coordination', (n, 'independent'), (n, s))
                           for s in ('nearest', 'greedy', 'hungarian'))
    for n in (2, 3):
        comparisons.extend(('Rule', (n, a), (n, b)) for a, b in
                           [('nearest', 'greedy'), ('nearest', 'hungarian'), ('greedy', 'hungarian')])
    comparisons.extend([
        ('Team size', (1, 'greedy'), (2, 'greedy')),
        ('Team size', (2, 'greedy'), (3, 'greedy')),
        ('Team size', (1, 'independent'), (2, 'independent')),
        ('Team size', (2, 'independent'), (3, 'independent')),
        ('Single robot', (1, 'independent'), (1, 'greedy')),
    ])
    rows = [dict(family=family, **compare(aruco, a, b)) for family, a, b in comparisons]
    for family in {r['family'] for r in rows}:
        ordered = sorted((r for r in rows if r['family'] == family), key=lambda r: r['p'])
        adjusted = 0.0
        for i, row in enumerate(ordered):
            adjusted = max(adjusted, min(1.0, (len(ordered) - i) * row['p']))
            row['p_holm'] = adjusted
    # Submitted Table 2, in the order constructed above (three-decimal rounding).
    expected_adjusted = [.012] * 6 + [.480, .059, .480, .244, .438, .674,
                                     .027, .018, .008, .193, .557]
    assert [round(r['p_holm'], 3) for r in rows] == expected_adjusted
    analysis.write_csv(str(root / 'results/tables/paired_statistics.csv'), rows)
    errors = [100 * t['victim_error_m'] for t in aruco if t['victim_confirmed']]
    print('Verified trial counts, completions, detections, Table 1 times, nearest speedups,')
    print('and Table 2 Holm-adjusted p-values against the submitted paper.')
    print(f'ArUco confirmed-position error: mean {statistics.mean(errors):.2f} cm; '
          f'maximum {max(errors):.2f} cm (100 confirmed trials).')
    print('Wrote results/tables/paired_statistics.csv')


if __name__ == '__main__':
    main()
