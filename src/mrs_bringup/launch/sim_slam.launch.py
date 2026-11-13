"""S3: Gazebo + one robot + the per-robot stack (rsp, EKF, slam_toolbox) + RViz.

  ros2 launch mrs_bringup sim_slam.launch.py
  ros2 launch mrs_bringup sim_slam.launch.py headless:=true rviz:=false

Then drive (teleop_twist_keyboard, or `ros2 run mrs_gazebo tour_driver.py`) and save:
  ros2 run nav2_map_server map_saver_cli -f src/mrs_navigation/maps/floor_v1
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
                # The stack runs rsp and the EKF owns odom -> base_footprint.
                launch_arguments={'headless': headless, 'rviz': 'false',
                                  'rsp': 'false', 'odom_tf': 'false'}.items(),
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'one_robot_stack.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'mode': 'sim'}.items(),
        ),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
