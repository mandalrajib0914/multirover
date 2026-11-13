"""S4: Gazebo + one robot + the per-robot stack with Nav2 (rsp, EKF, slam_toolbox, Nav2) + RViz.

  ros2 launch mrs_bringup sim_nav.launch.py
  ros2 launch mrs_bringup sim_nav.launch.py headless:=true rviz:=false

Nav2 runs on the live SLAM map. In RViz, use "2D Goal Pose" (it publishes goal_pose, and
bt_navigator turns that into a NavigateToPose goal). Scripted check:
  ros2 run mrs_navigation nav_check.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mrs_bringup')
    gz_pkg = get_package_share_directory('mrs_gazebo')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        # Scoped: an include's launch_arguments (rviz:=false) otherwise leak into this file's
        # own configurations and switch off its RViz too.
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'sim_one_robot.launch.py')),
                launch_arguments={'headless': headless, 'rviz': 'false',
                                  'rsp': 'false', 'odom_tf': 'false'}.items(),
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'one_robot_stack.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'mode': 'sim', 'nav': 'true'}.items(),
        ),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', os.path.join(pkg, 'rviz', 'nav.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
