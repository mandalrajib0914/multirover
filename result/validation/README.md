# Validation evidence

Tests behind specific claims in the report. The offline ones need nothing running. Run them
from `~/Desktop/multirover` with ROS sourced:
`source /opt/ros/jazzy/setup.bash && source install/setup.bash`.

| Script | What it checks | Result (2026-09-26) | Used in |
|---|---|---|---|
| `test_map_merge.py` | `simple_map_merge.merge_grids`. (1) An S5 map merged alone reproduces `map_vs_gt.py`'s coverage. (2) A frame rotated 90°. (3) Occupied beats free. | (1) 95.4 % = 95.4 %. (2) 0 misplaced cells. (3) pass. | S6.2 map merge |
| `test_coordinator.py` | Coordinator functions: grid path cost through a door, frontier detection, strategy assignment, timing on a real map. | Path 2.75 m (analytic 2.76 m), no door gives inf. Frontier found at the doorway, none on a complete map. nearest doubles up, greedy and hungarian split. About 6 ms per cycle. | S6.3 coordinator |
| `test_aruco_offline.py` | The real victim texture rendered into a level camera at 0.8–2.5 m and 0–50° off-axis; victim-centre error in the world frame. | ≤ 1.4 cm up to 1.5 m, 3–7 cm at 2.0 m, 12–14 cm at 2.5 m, so `max_range` is 2.0 m. | S7 detector |
| `aruco_gazebo_test.sh x y yaw` + `aruco_gazebo_probe.py` | The real Gazebo camera: one robot placed at a pose, detections and error counted for 8 s (needs the sim; about 1 min). | 1.2 m → 1.8 cm, 1.8 m → 4.3 cm, 2.0 m → 8 cm, 1.5 m at 40° → 4.4 cm; detected in every frame. Poses used: `3.0 3.40 1.5708`, `3.0 2.80 1.5708`, `3.0 2.62 1.5708`, `2.0 3.50 0.85`. | S7 detector |
| `s4_costmap_probe.py` | With `sim_nav` running: spawns a box 0.5 m ahead and prints the map and costmap cells there. | Before the fix, the costmaps showed nothing (obstacle layers got no scan); after it, the box is lethal (100) in both. | S4 obstacle-layer bug |

Also in the source tree:
- `src/mrs_gazebo/scripts/odom_sanity_check.py` (S2 odometry against ground truth)
- `src/mrs_navigation/scripts/nav_check.py` (the S4 Nav2 tests)
- `result/loc_error.py` (S8 localization error from the bags)
