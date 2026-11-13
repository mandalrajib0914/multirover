"""S6.1: Gazebo + N namespaced robots, each with its own stack (rsp, EKF, slam_toolbox, Nav2).

  ros2 launch mrs_bringup sim_multi_nav.launch.py n:=2 [headless:=true]

Everything per robot is under /robotN with frames robotN/...; all robots share the global
/tf. The world frame "map" is tied to each robot's SLAM map by a static map -> robotN/map at
that robot's spawn pose (slam_toolbox starts its map at the robot's first pose), so a goal
in "map" can go to any robot and RViz with Fixed Frame "map" shows them all.
merge:=true (default) runs simple_map_merge (/robotN/map -> /map) and has every robot's
Nav2 plan on the merged /map, so a robot can take goals in areas only others have seen.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    pkg = get_package_share_directory('mrs_bringup')
    gz_pkg = get_package_share_directory('mrs_gazebo')
    n = int(LaunchConfiguration('n').perform(context))
    nav = LaunchConfiguration('nav').perform(context)
    merge = LaunchConfiguration('merge').perform(context).lower() == 'true'
    plan_file = LaunchConfiguration('plan_file').perform(context) or os.path.join(
        gz_pkg, 'config', 'floor_plan.yaml')
    with open(plan_file) as f:
        poses = yaml.safe_load(f)['spawn_poses']

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'sim_robots.launch.py')),
        launch_arguments={'n': str(n), 'headless': LaunchConfiguration('headless'),
                          'plan_file': plan_file,
                          'victim_model': LaunchConfiguration('victim_model')}.items(),
    )]
    for i in range(n):
        ns = f'robot{i + 1}'
        x, y, yaw = poses[ns]
        actions += [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'one_robot_stack.launch.py')),
                launch_arguments={'namespace': ns, 'use_sim_time': 'true', 'mode': 'sim',
                                  'nav': nav, 'nav_global_map': 'merged' if merge else 'own',
                                  'other_robots': ','.join(f'robot{j + 1}' for j in range(n) if j != i)
                                  }.items(),
            ),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name=f'map_to_{ns}_map', output='log',
                 arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                            '--frame-id', 'map', '--child-frame-id', f'{ns}/map'],
                 parameters=[{'use_sim_time': True}]),
        ]
    if merge:
        actions.append(Node(
            package='mrs_bringup', executable='simple_map_merge.py', output='screen',
            parameters=[{'use_sim_time': True, 'robots': [f'robot{i + 1}' for i in range(n)]}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2'),
        DeclareLaunchArgument('victim_model', default_value='victim', description='victim | victim_person'),
        DeclareLaunchArgument('plan_file', default_value='',
                              description='spawn_poses + victim (default: mrs_gazebo floor_plan.yaml)'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('nav', default_value='true', description='Run Nav2 per robot'),
        DeclareLaunchArgument('merge', default_value='true',
                              description='simple_map_merge -> /map, and Nav2 plans on it'),
        OpaqueFunction(function=launch_setup),
    ])
