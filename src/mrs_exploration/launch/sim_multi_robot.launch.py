"""S6: N robots explore the floor together, one command, no human input.

  ros2 launch mrs_exploration sim_multi_robot.launch.py n:=2 strategy:=greedy
  ros2 launch mrs_exploration sim_multi_robot.launch.py n:=3 strategy:=independent \
      headless:=true rviz:=false exit_when_done:=true tag:=x

strategy: nearest | greedy | hungarian  -> coordinator.py on the merged /map
          independent                   -> the baseline: one explore_lite per robot, each on
                                           its own SLAM map, no coordination
Runs sim_multi_nav.launch.py (Gazebo, N namespaced stacks with Nav2 on the merged map,
simple_map_merge), the exploration logic, the ArUco victim detector (victim:=aruco; with
stop_on_victim:=true the coordinator ends the mission once the victim is confirmed; the
independent baseline never stops early) and the exploration_monitor, which records time,
coverage of the merged map vs time and per-robot distance to <results_dir>/<timestamp>/.
With exit_when_done:=true everything shuts down once the monitor is done.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

COLOURS = ['31; 119; 180', '255; 127; 14', '44; 160; 44', '214; 39; 40']


def rviz_config(names):
    """RViz config for N robots: merged map, coordinator markers, per-robot model/scan/plan.
    Every display needs `Enabled: true`: RViz 2 leaves a display without it switched off."""
    d = ['''    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.5
      Plane Cell Count: 20
    - Class: rviz_default_plugins/Map
      Name: Merged map
      Enabled: true
      Topic:
        Value: /map
        Durability Policy: Transient Local
      Alpha: 0.8
    - Class: rviz_default_plugins/MarkerArray
      Name: Coordinator
      Enabled: true
      Topic:
        Value: /coordinator/markers
    - Class: rviz_default_plugins/MarkerArray
      Name: Victim
      Enabled: true
      Topic:
        Value: /victim/markers
        Durability Policy: Transient Local''']
    for i, r in enumerate(names):
        c = COLOURS[i % len(COLOURS)]
        d.append(f'''    - Class: rviz_default_plugins/RobotModel
      Name: {r} model
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /{r}/robot_description
        Durability Policy: Transient Local
    - Class: rviz_default_plugins/LaserScan
      Name: {r} scan
      Enabled: true
      Topic:
        Value: /{r}/scan
        Reliability Policy: Best Effort
      Size (m): 0.03
      Style: Flat Squares
      Color Transformer: FlatColor
      Color: {c}
    - Class: rviz_default_plugins/Path
      Name: {r} plan
      Enabled: true
      Topic:
        Value: /{r}/plan
      Color: {c}
      Line Width: 0.03
      Line Style: Billboards
    - Class: rviz_default_plugins/MarkerArray
      Name: {r} explore_lite frontiers
      Enabled: false
      Topic:
        Value: /{r}/explore/frontiers''')
    text = ('Panels:\n  - Class: rviz_common/Displays\n    Name: Displays\n'
            'Visualization Manager:\n  Class: ""\n  Displays:\n' + '\n'.join(d) + '''
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Scale: 190
      X: 3.0
      Y: 2.5
Window Geometry:
  Height: 900
  Width: 1400
''')
    path = os.path.join(tempfile.mkdtemp(prefix='mrs_rviz_'), 'multi.rviz')
    with open(path, 'w') as f:
        f.write(text)
    return path


def launch_setup(context):
    pkg = get_package_share_directory('mrs_exploration')
    bringup = get_package_share_directory('mrs_bringup')
    n = int(LaunchConfiguration('n').perform(context))
    strategy = LaunchConfiguration('strategy').perform(context)
    record = LaunchConfiguration('record').perform(context) == 'true'
    names = [f'robot{i + 1}' for i in range(n)]
    victim = LaunchConfiguration('victim').perform(context)

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'sim_multi_nav.launch.py')),
        launch_arguments={'n': str(n), 'headless': LaunchConfiguration('headless'),
                          'merge': 'true', 'plan_file': LaunchConfiguration('plan_file'),
                          'victim_model': 'victim_person' if victim == 'yolo' else 'victim'}.items(),
    )]

    if strategy == 'independent':
        # Baseline: explore_lite per robot on its own SLAM map (/robotN/map, relative topic).
        for r in names:
            actions.append(GroupAction([
                PushRosNamespace(r),
                Node(package='explore_lite', executable='explore', name='explore_node',
                     output='screen',
                     parameters=[os.path.join(pkg, 'config', 'explore.yaml'),
                                 {'use_sim_time': True, 'robot_base_frame': f'{r}/base_footprint'}]),
            ]))
        status_topics = [f'/{r}/explore/status' for r in names]
    else:
        actions.append(Node(
            package='mrs_exploration', executable='coordinator', output='screen',
            parameters=[os.path.join(pkg, 'config', 'coordinator.yaml'),
                        {'use_sim_time': True, 'robots': names, 'strategy': strategy,
                         'stop_on_victim': LaunchConfiguration('stop_on_victim').perform(context) == 'true',
                         'debug_dir': LaunchConfiguration('debug_dir').perform(context)}]))
        status_topics = ['/coordinator/status']

    if victim == 'aruco':
        actions.append(Node(
            package='mrs_perception', executable='aruco_victim_node', output='screen',
            parameters=[os.path.join(get_package_share_directory('mrs_perception'), 'config', 'aruco.yaml'),
                        {'use_sim_time': True, 'robots': names}]))
    elif victim == 'yolo':
        # ultralytics/torch live in the project venv, so start the node with its python
        venv = LaunchConfiguration('venv').perform(context)
        actions.append(ExecuteProcess(
            cmd=[os.path.join(venv, 'bin', 'python'), '-m', 'mrs_perception.yolo_victim_node',
                 '--ros-args', '--params-file',
                 os.path.join(get_package_share_directory('mrs_perception'), 'config', 'yolo.yaml'),
                 '-p', f"robots:={names}".replace(' ', ''),
                 '-p', f"weights:={os.path.join(venv, 'weights', 'yolov8n.pt')}"],
            output='screen', name='yolo_victim_node'))

    if record:
        monitor = Node(
            package='mrs_exploration', executable='exploration_monitor', output='screen',
            parameters=[{'use_sim_time': True, 'robots': names, 'map_frame': 'world',
                         'status_topics': status_topics, 'strategy': strategy,
                         'results_dir': LaunchConfiguration('results_dir'),
                         'run_dir': LaunchConfiguration('run_dir'),
                         'plan_file': LaunchConfiguration('plan_file'),
                         'timeout': float(LaunchConfiguration('timeout').perform(context)),
                         'tag': LaunchConfiguration('tag')}])
        actions.append(monitor)
        if LaunchConfiguration('exit_when_done').perform(context) == 'true':
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=monitor,
                on_exit=[EmitEvent(event=Shutdown(reason='exploration finished'))])))

    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', rviz_config(names)],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2', description='number of robots (1-3)'),
        DeclareLaunchArgument('strategy', default_value='greedy',
                              description='nearest | greedy | hungarian | independent'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('record', default_value='true', description='Run exploration_monitor'),
        DeclareLaunchArgument('results_dir', default_value='data/s6'),
        DeclareLaunchArgument('run_dir', default_value='',
                              description='exact results folder (S8 trials); default <results_dir>/<stamp>_<tag>'),
        DeclareLaunchArgument('plan_file', default_value='',
                              description='spawn_poses + victim (default: mrs_gazebo floor_plan.yaml)'),
        DeclareLaunchArgument('timeout', default_value='900.0', description='sim seconds'),
        DeclareLaunchArgument('tag', default_value=''),
        DeclareLaunchArgument('exit_when_done', default_value='false'),
        DeclareLaunchArgument('debug_dir', default_value='', description='coordinator grid dumps'),
        DeclareLaunchArgument('victim', default_value='aruco',
                              description='aruco (box) | yolo (doll-sized person, venv) | none'),
        DeclareLaunchArgument('venv', default_value=os.path.expanduser('~/Desktop/multirover/.venv'),
                              description='python venv with ultralytics (victim:=yolo)'),
        DeclareLaunchArgument('stop_on_victim', default_value='true',
                              description='end the mission once the victim is confirmed (S7)'),
        OpaqueFunction(function=launch_setup),
    ])
