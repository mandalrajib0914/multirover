"""Everything that runs per robot: robot_state_publisher + EKF + slam_toolbox + Nav2 (nav:=true).

  ros2 launch mrs_bringup one_robot_stack.launch.py                      # single robot, sim
  ros2 launch mrs_bringup one_robot_stack.launch.py namespace:=robot1    # S6, one per robot
  ros2 launch mrs_bringup one_robot_stack.launch.py mode:=real use_sim_time:=false
  ros2 launch mrs_bringup one_robot_stack.launch.py nav:=true             # S4, + Nav2

mode:=sim  also runs sim_wheel_odom.py (gz odom -> wheel/odom with covariance and slip).
mode:=real expects the base driver to publish wheel/odom.
All topics are relative; with a namespace the frames get the "<namespace>/" prefix.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def launch_setup(context):
    pkg = get_package_share_directory('mrs_bringup')
    desc_pkg = get_package_share_directory('mrs_description')

    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    mode = LaunchConfiguration('mode').perform(context)
    slam = LaunchConfiguration('slam').perform(context).lower() == 'true'
    nav = LaunchConfiguration('nav').perform(context).lower() == 'true'
    nav_params = LaunchConfiguration('nav_params').perform(context)
    others = [r for r in LaunchConfiguration('other_robots').perform(context).split(',') if r]
    if mode not in ('sim', 'real'):
        raise RuntimeError(f"mode must be 'sim' or 'real', got '{mode}'")
    prefix = f'{ns}/' if ns else ''
    frames = {'map_frame': 'map' if not ns else f'{prefix}map',
              'odom_frame': f'{prefix}odom'}

    actions = [PushRosNamespace(ns)] if ns else []

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': str(use_sim_time).lower(), 'rviz': 'false',
                          'prefix': prefix, 'namespace': ns}.items(),
    ))

    if mode == 'sim':
        actions.append(Node(package='mrs_bringup', executable='sim_wheel_odom.py',
                            output='screen', parameters=[{'use_sim_time': use_sim_time}]))

    actions.append(Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {
            'use_sim_time': use_sim_time,
            'map_frame': frames['map_frame'], 'odom_frame': frames['odom_frame'],
            'world_frame': frames['odom_frame'], 'base_link_frame': f'{prefix}base_footprint'}],
    ))

    if others:
        # SLAM maps without the other robots in them (else they merge as walls, S6)
        actions.append(Node(package='mrs_bringup', executable='robot_scan_filter.py',
                            output='screen', parameters=[{'use_sim_time': use_sim_time,
                                                          'other_robots': others}]))

    if slam:
        slam_node = LifecycleNode(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
            namespace='', output='screen',
            parameters=[os.path.join(pkg, 'config', 'slam.yaml'), {
                'use_sim_time': use_sim_time, 'use_lifecycle_manager': False,
                'map_frame': frames['map_frame'], 'odom_frame': frames['odom_frame'],
                'base_frame': f'{prefix}base_footprint',
                'scan_topic': 'scan_filtered' if others else 'scan'}],
        )
        # Same autostart as slam_toolbox's online_async_launch.py: configure, then activate.
        actions += [
            slam_node,
            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam_node),
                                        transition_id=Transition.TRANSITION_CONFIGURE)),
            RegisterEventHandler(OnStateTransition(
                target_lifecycle_node=slam_node, start_state='configuring', goal_state='inactive',
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE))])),
        ]

    if nav:
        # Our own Nav2 launch: no /tf remap, per-robot frames and namespace (see its docstring).
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('mrs_navigation'), 'launch', 'nav2.launch.py')),
            launch_arguments={'namespace': ns, 'use_sim_time': str(use_sim_time).lower(),
                              'params_file': nav_params,
                              'global_map': LaunchConfiguration('nav_global_map').perform(context)}.items(),
        ))

    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='e.g. robot1 (S6)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('mode', default_value='sim', description='sim | real'),
        DeclareLaunchArgument('slam', default_value='true', description='Run slam_toolbox'),
        DeclareLaunchArgument('nav', default_value='false', description='Run Nav2 (S4)'),
        DeclareLaunchArgument('other_robots', default_value='',
                              description='e.g. robot2,robot3: filtered out of the scan SLAM uses (S6)'),
        DeclareLaunchArgument('nav_global_map', default_value='own',
                              description='own | merged (plan on the merged /map, S6)'),
        DeclareLaunchArgument('nav_params', default_value='',
                              description='Nav2 params file (default: mrs_navigation/config/nav2_params.yaml)'),
        OpaqueFunction(function=launch_setup),
    ])
