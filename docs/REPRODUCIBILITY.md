# Reproducing and checking results

Release `v1.1.0` provides code and recorded experimental data accompanying
“Coordinated Frontier Exploration and Victim Localization with a Team of Ground
Robots: A Simulation Study” (NCET-2026, Rajib Mandal).

## 1. Check the saved experiments without running ROS

From the repository root:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -c venv_constraints.txt numpy scipy matplotlib pyyaml
.venv/bin/python analysis/verify_results.py
.venv/bin/python analysis/make_result.py
```

Verification checks the campaign sizes, completion counts, detection counts,
Table 1 exploration times and confidence intervals, nearest-strategy paired
speedups, and Table 2 Holm-adjusted p-values. It also writes the full statistical
comparison table. This is a numerical consistency check, not independent
replication of the simulations or a check of every sentence in the paper.

| Submitted-paper result | Data and analysis output |
|---|---|
| Table 1: exploration time, distance, overlap | `data/s8/`, `results/tables/s8_summary.csv` |
| Table 1: paired speedups | `results/tables/s8_speedups.csv` |
| Table 2: paired statistical tests | `analysis/verify_results.py`, `results/tables/paired_statistics.csv` |
| Table 3: ArUco versus YOLO | `data/s8_yolo/`, `results/tables/s7b_yolo_vs_aruco.csv` (rows with `victim=all`) |
| Exploration-time plot | `results/figures/s8/time_vs_robots.png` |
| Robot trajectories | `data/s8/*/trajectories.csv`, `results/figures/s8/trajectories_n3.png` |
| Localization errors | `data/s8/*/loc_error.yaml`, `analysis/loc_error.py`, recorded bags |

Figures are repository versions of the analysis, not exact copies of the paper's
page layout.

## 2. Recompute localization from the ROS bags

Follow the [dataset download instructions](../data/README.md), source ROS 2,
and rerun `analysis/make_result.py` to regenerate the localization time-series
figure. The analysis requires rosbag2 MCAP and zstd support. Without the bags,
the existing localization summaries and figure remain available, but their
underlying TF time-series cannot be recomputed.

## 3. Repeat simulation experiments

Follow the build instructions in [README.md](../README.md) and the detailed
[experiment commands](setup/README.md). Recorded dependency versions
are in `docs/setup/versions.txt`.

Start with one mission before the full campaign:

```bash
ros2 launch mrs_exploration sim_multi_robot.launch.py n:=2 strategy:=greedy
ros2 run mrs_experiments run_campaign.py --dry-run
```

The full campaign runner skips existing summaries. Use a separate checkout and
move its existing `data/` directory aside before collecting a fresh campaign;
do not overwrite the release dataset. The runner terminates matching ROS and
simulation processes between trials, so use a dedicated session.

The recorded setup used Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and an NVIDIA
RTX 4070 Laptop GPU. The archived setup notes estimate roughly 4.5 hours for the
primary campaign; runtime depends on hardware. YOLO has additional Python and
model dependencies described in the README and perception configuration.

Simulation reruns need not reproduce identical timings. Compare repeated-run
statistics and retain deviations, failures, and excluded-run reasons.

## Offline component checks

With ROS 2 sourced, run from the repository root:

```bash
python3 tests/test_map_merge.py
python3 tests/test_coordinator.py
python3 tests/test_aruco_offline.py
```

These existing diagnostic scripts print their numerical observations for review.
The map-merge check reads the source ground-truth map and does not require an
`install/` tree. The synthetic ArUco check includes distances beyond the
configured 2 m acceptance limit, so its overall worst error is not the same
metric as confirmed-victim error in the campaign.

## Migrating from v1.0.0

The recorded measurements now live in `data/`, processing scripts in `analysis/`,
generated tables and figures in `results/`, validation scripts in `tests/`, and
setup instructions in `docs/`. Run all commands from the repository root.

The original dataset download remains valid. Use the extraction command in
[data/README.md](../data/README.md), which translates its original `results/`
archive prefix to `data/`. Do not extract it directly into the new `results/`
folder. Keep an old checkout separate if you need to reproduce the v1.0.0 layout.
