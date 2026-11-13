"""One S8 trial: headless sim, N robots, a strategy, a trial plan, a rosbag. Ends by itself.

  ros2 launch mrs_experiments trial.launch.py n:=2 strategy:=greedy \
      plan_file:=results/s8/<tag>/trial_plan.yaml run_dir:=results/s8/<tag> tag:=<tag>

Runs mrs_exploration's sim_multi_robot.launch.py (headless, no RViz, exit_when_done true,
by default full exploration: stop_on_victim false; victim:=aruco box or yolo doll) with the trial plan's start poses and victim, and
records a bag into <run_dir>/bag (zstd): the merged /map, /tf, /tf_static, the victim and
coordinator topics, and per robot scan, map, filtered odometry and explore_lite status.
exploration_monitor writes its results into <run_dir> too. Normally started by
run_campaign.py, one trial at a time.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    n = int(LaunchConfiguration('n').perform(context))
    run_dir = os.path.abspath(LaunchConfiguration('run_dir').perform(context))
    topics = ['/map', '/tf', '/tf_static', '/clock', '/victim/pose', '/victim/confirmed',
              '/victim/candidate', '/coordinator/status', '/coordinator/markers']
    for i in range(n):
        r = f'/robot{i + 1}'
        topics += [f'{r}/scan', f'{r}/map', f'{r}/odometry/filtered', f'{r}/explore/status']
    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', os.path.join(run_dir, 'bag'),
             '--compression-mode', 'file', '--compression-format', 'zstd',
             '--include-unpublished-topics', *topics],
        output='log')
    trial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mrs_exploration'), 'launch', 'sim_multi_robot.launch.py')),
        launch_arguments={
            'n': str(n), 'strategy': LaunchConfiguration('strategy').perform(context),
            'headless': 'true', 'rviz': 'false', 'exit_when_done': 'true',
            'stop_on_victim': LaunchConfiguration('stop_on_victim').perform(context),
            'plan_file': LaunchConfiguration('plan_file').perform(context),
            'victim': LaunchConfiguration('victim').perform(context),
            'run_dir': run_dir, 'tag': LaunchConfiguration('tag').perform(context),
            'timeout': LaunchConfiguration('timeout').perform(context),
        }.items())
    return [bag, trial]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2'),
        DeclareLaunchArgument('strategy', default_value='greedy'),
        DeclareLaunchArgument('plan_file', description='trial plan: spawn_poses + victim'),
        DeclareLaunchArgument('run_dir', description='results folder for this trial'),
        DeclareLaunchArgument('tag', default_value=''),
        DeclareLaunchArgument('timeout', default_value='600.0', description='sim seconds'),
        DeclareLaunchArgument('victim', default_value='aruco', description='aruco | yolo'),
        DeclareLaunchArgument('stop_on_victim', default_value='false',
                              description='S8 explores fully (false); true = end at the victim'),
        OpaqueFunction(function=launch_setup),
    ])
