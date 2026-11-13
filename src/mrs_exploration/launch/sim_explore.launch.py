"""S5: one command, one robot explores the whole floor with no human input.

  ros2 launch mrs_exploration sim_explore.launch.py
  ros2 launch mrs_exploration sim_explore.launch.py headless:=true rviz:=false exit_when_done:=true

Runs sim_nav.launch.py (Gazebo + rsp + EKF + slam_toolbox + Nav2), explore_lite and the
exploration_monitor. The monitor writes time, coverage vs time, the saved map and its score
to <results_dir>/<timestamp>/. With exit_when_done:=true the whole launch shuts down once the
monitor is done (after the robot has returned to its start), for scripted batch runs (S8).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mrs_exploration')
    bringup = get_package_share_directory('mrs_bringup')

    monitor = Node(
        package='mrs_exploration', executable='exploration_monitor', output='screen',
        condition=IfCondition(LaunchConfiguration('record')),
        parameters=[{'use_sim_time': True,
                     'results_dir': LaunchConfiguration('results_dir'),
                     'timeout': LaunchConfiguration('timeout'),
                     'tag': LaunchConfiguration('tag')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='true', description='Run exploration_monitor'),
        DeclareLaunchArgument('results_dir', default_value='results/s5'),
        DeclareLaunchArgument('timeout', default_value='900.0', description='sim seconds'),
        DeclareLaunchArgument('tag', default_value='', description='suffix for the results folder'),
        DeclareLaunchArgument('exit_when_done', default_value='false'),
        # Scoped: an include's launch_arguments (rviz:=false) otherwise leak into this file's
        # own configurations and switch off its RViz too.
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim_nav.launch.py')),
                launch_arguments={'headless': LaunchConfiguration('headless'), 'rviz': 'false'}.items(),
            ),
        ]),
        Node(package='explore_lite', executable='explore', name='explore_node', output='screen',
             parameters=[os.path.join(pkg, 'config', 'explore.yaml'), {'use_sim_time': True}]),
        monitor,
        RegisterEventHandler(
            OnProcessExit(target_action=monitor, on_exit=[EmitEvent(event=Shutdown(
                reason='exploration finished'))]),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('exit_when_done'), "' == 'true' and '",
                LaunchConfiguration('record'), "' == 'true'"])),
        ),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(pkg, 'rviz', 'explore.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
