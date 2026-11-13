# Setup and how to reproduce

- `versions.txt`: machine, OS and the exact package versions used for every result.
- `config/`: copies of the configs the results were made with. The live files are under
  `src/`; if they change later, these copies still show what S8 ran with.
  - `floor_plan.yaml`: arena, S2–S7 spawn poses, victim V1, ArUco
  - `trial_sets.yaml`: S8 conditions, start sets, victims
  - `ekf.yaml`, `slam.yaml`: localization and SLAM (map update 1 s from S8 on)
  - `nav2_params.yaml`: Nav2 (Smac 2D, Regulated Pure Pursuit, footprint, costmaps)
  - `coordinator.yaml`: the frontier coordinator (strategy, λ, R, D, hysteresis, investigation)
  - `explore.yaml` + `explore_lite_patches.patch`: the explore_lite baseline and the two local fixes
  - `aruco.yaml`: the victim detector
  - `cyclonedds_local.xml`: DDS over loopback (simulation only)

## Commands (from ~/Desktop/multirover, after `colcon build --symlink-install`)

| Step | Command |
|---|---|
| S2 odometry check | `ros2 launch mrs_gazebo sim_one_robot.launch.py headless:=true rviz:=false`, then `ros2 run mrs_gazebo odom_sanity_check.py` |
| S3 SLAM map | `ros2 launch mrs_bringup sim_slam.launch.py`, `ros2 run mrs_gazebo tour_driver.py`, `ros2 run nav2_map_server map_saver_cli -f <name>`, `ros2 run mrs_gazebo map_vs_gt.py <name>.yaml` |
| S4 navigation tests | `ros2 launch mrs_bringup sim_nav.launch.py headless:=true rviz:=false`, then `ros2 run mrs_navigation nav_check.py --trace out.csv` |
| S5 one robot explores | `ros2 launch mrs_exploration sim_explore.launch.py headless:=true rviz:=false exit_when_done:=true` |
| S6/S7 multi-robot mission | `ros2 launch mrs_exploration sim_multi_robot.launch.py n:=2 strategy:=greedy` (`stop_on_victim:=false` to explore everything) |
| S8 campaign (≈ 4.5 h, resumable) | `ros2 run mrs_experiments run_campaign.py` |
| All tables and figures | `python3 result/make_result.py` (source ROS first to also build the localization figure) |
