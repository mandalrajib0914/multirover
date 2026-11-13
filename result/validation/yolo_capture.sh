#!/bin/bash
# Capture frames of one Fuel person model at 1-4 m for the YOLO test (S7b).
#   bash result/validation/yolo_capture.sh "<Fuel model name>|model://<local>" <out_dir> [yaw]
# Person at (5.7, 0.7) in room B (a clear 4 m line along y = 0.7), facing yaw (default: pi,
# towards -x, where the camera is).
cd ~/Desktop/multirover
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export CYCLONEDDS_URI=file://$HOME/Desktop/multirover/src/mrs_bringup/config/cyclonedds_local.xml
MODEL="$1"; OUT="$2"; YAW="${3:-3.1416}"
W=$(ros2 pkg prefix mrs_gazebo)/share/mrs_gazebo/worlds/floor_novictim.sdf
setsid ros2 launch mrs_gazebo sim_one_robot.launch.py headless:=true rviz:=false world:=$W x:=1.7 y:=0.7 yaw:=0.0 > /tmp/yolo_capture_sim.log 2>&1 &
PG=$!
sleep 15
case "$MODEL" in model://*) URI="$MODEL";; *) URI="https://fuel.gazebosim.org/1.0/OpenRobotics/models/$MODEL";; esac
ros2 run ros_gz_sim create -name person -x 5.7 -y 0.7 -z 0.0 -Y "$YAW" \
  -string "<?xml version=\"1.0\"?><sdf version=\"1.9\"><include><uri>$URI</uri></include></sdf>" > /dev/null 2>&1
sleep 4
timeout 90 python3 result/validation/yolo_capture.py "$OUT" 5.7 0.7 ${DISTS:-1.0 2.0 3.0 4.0}
kill -INT -$PG; sleep 6; kill -9 -$PG 2>/dev/null; true
