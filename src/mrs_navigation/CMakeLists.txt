cmake_minimum_required(VERSION 3.8)
project(mrs_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/nav_check.py DESTINATION lib/${PROJECT_NAME})

ament_package()
