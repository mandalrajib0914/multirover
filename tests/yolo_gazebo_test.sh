#!/bin/bash
# YOLO victim detector accuracy on the real Gazebo camera + lidar (S7b).
#   bash tests/yolo_gazebo_test.sh <robot_x> <robot_y> <robot_yaw> [person_yaw]
# One robot (sim_one_robot, un-namespaced, odom TF from gz) at the pose, the doll-sized person
# (model://victim_person) at (5.7, 0.7); yolo_victim_node runs 12 s from the venv in the robot's
# odom frame. Prints its stats and the error against the true position.
cd ~/Desktop/multirover
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export CYCLONEDDS_URI=file://$HOME/Desktop/multirover/src/mrs_bringup/config/cyclonedds_local.xml
RX=$1; RY=$2; RYAW=$3; PYAW=${4:-3.1416}
W=$(ros2 pkg prefix mrs_gazebo)/share/mrs_gazebo/worlds/floor_novictim.sdf
setsid ros2 launch mrs_gazebo sim_one_robot.launch.py headless:=true rviz:=false world:=$W x:=$RX y:=$RY yaw:=$RYAW > /tmp/yolo_test_sim.log 2>&1 &
PG=$!
sleep 15
ros2 run ros_gz_sim create -name person -x 5.7 -y 0.7 -z 0.0 -Y "$PYAW" \
  -string '<?xml version="1.0"?><sdf version="1.9"><include><uri>model://victim_person</uri></include></sdf>' > /dev/null 2>&1
sleep 3
timeout 40 .venv/bin/python - "$RX" "$RY" "$RYAW" <<'EOF' 2>&1 | grep -vE "^\[INFO\].*(watching|stats)"
import math, sys, time, os
import rclpy
rx, ry, ryaw = (float(v) for v in sys.argv[1:4])
rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', "robots:=['']", '-p', 'world_frame:=odom',
                 '-p', 'weights:=.venv/weights/yolov8n.pt', '-p', 'max_range:=6.0', '-p', 'candidate_range:=8.0',
                 '-p', 'confirm_hits:=1', '-p', 'body_offset:=0.0'])
from mrs_perception.yolo_victim_node import YoloVictimNode
n = YoloVictimNode()
t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.05)
# true person position in the robot's odom frame (odom = spawn pose)
dx, dy = 5.7 - rx, 0.7 - ry
c, s = math.cos(-ryaw), math.sin(-ryaw)
tx, ty = c * dx - s * dy, s * dx + c * dy
est = n.estimate() if n.hits else None
err = None if est is None else math.hypot(est[0] - tx, est[1] - ty)
rngs = [round(h[4], 2) for h in n.hits]
print(f"stats {n.stats}, lidar ranges {min(rngs) if rngs else None}-{max(rngs) if rngs else None} m, "
      f"estimate {None if est is None else [round(v, 3) for v in est[:2]]}, truth [{tx:.3f}, {ty:.3f}], "
      f"error {None if err is None else round(err, 3)} m", flush=True)
os._exit(0)
EOF
kill -INT -$PG; sleep 6; kill -9 -$PG 2>/dev/null; true
