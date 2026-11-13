cmake_minimum_required(VERSION 3.8)
project(mrs_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config worlds models maps launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/make_victim_person.py scripts/make_floor.py scripts/odom_sanity_check.py scripts/tour_driver.py scripts/map_vs_gt.py DESTINATION lib/${PROJECT_NAME})

ament_package()
