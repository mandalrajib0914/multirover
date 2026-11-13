cmake_minimum_required(VERSION 3.8)
project(mrs_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_wheel_odom.py scripts/simple_map_merge.py scripts/robot_scan_filter.py DESTINATION lib/${PROJECT_NAME})

ament_package()
