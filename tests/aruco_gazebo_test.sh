#!/bin/bash
# usage: cam_test.sh x y yaw  -> single robot at a pose, count detections
S=$(dirname "$0"); cd /home/rajib-mandal/Desktop/multirover
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export CYCLONEDDS_URI=file://$HOME/Desktop/multirover/src/mrs_bringup/config/cyclonedds_local.xml
setsid ros2 launch mrs_gazebo sim_one_robot.launch.py headless:=true rviz:=false x:=$1 y:=$2 yaw:=$3 > $S/cam.log 2>&1 &
sleep 22; timeout 40 python3 $S/aruco_gazebo_probe.py 2>&1 | tail -4
pkill -INT -f "gz sim" ; sleep 5
