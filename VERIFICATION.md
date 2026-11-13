# Verification against the submitted paper

The submitted paper was inspected locally during release preparation. Paper
files and presentation materials are not distributed in this repository.

## Confirmed from the archived measurements

`python3 result/verify_results.py` checks:

- 110 primary ArUco trials, with 109 completed explorations and 100 victim confirmations.
- 40 complementary YOLO trials, with 39 completed explorations and 40 victim confirmations.
- All 11 Table 1 exploration-time means and 95% confidence-interval half-widths.
- Nearest-strategy paired speedups of 1.61 ± 0.65 (two robots) and 1.96 ± 0.61
  (three robots), using nine completed pairs in each case.
- All 17 Table 2 Holm-adjusted p-values, rounded to three decimal places.

## Analysis correction included in this release

The older `paired_speedups` implementation included a stalled single-robot
nearest trial. This produced 1.67 and 2.00 for the two- and three-robot nearest
conditions. The release filters both members of a pair to completed trials,
matching the paper's stated method and its Table 1 values of 1.61 and 1.96.
Raw trial data are unchanged. Updated CSV and Markdown summaries use this filter.

## Clarifications when interpreting the paper

- The victim-localization paragraph calls 9.8 cm the ArUco **mean** error.
  Across the 100 confirmed primary trials, the data give a mean of approximately
  **4.91 cm** and a **maximum of 9.80 cm**. The 9.8 cm value should be interpreted
  as a maximum, consistent with the abstract and per-condition maximum errors.
- The paper reports median wall F1, whereas the `wall_f1_pct` column in
  `s8_summary.csv` is a mean. These aggregations can differ substantially when
  a trial has a localization failure.
- YOLO confirmation-time improvements use matched trials where both detectors
  confirmed the victim. They are not necessarily the difference between the
  separate condition means in Table 3.
- The statistical implementation uses an exact Wilcoxon calculation when
  differences are nonzero and a normal approximation for comparisons with zero
  differences. This matches the archived analysis, but the paper describes the
  tests collectively as exact. SciPy may warn about the small sample size for
  the approximation; inspect `result/verify_results.py` and the exported test
  table when interpreting those comparisons.

This check does not certify every paper claim or replace an independent rerun.
Physical-robot performance, other arenas, unreliable communication, and damaged
building conditions were not established by this simulation dataset.
