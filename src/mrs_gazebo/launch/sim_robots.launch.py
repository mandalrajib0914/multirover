"""Gazebo Harmonic + N namespaced robots + their ros_gz bridges (S6). Gazebo side only.

  ros2 launch mrs_gazebo sim_robots.launch.py n:=2 [headless:=true]

Per robot robotN (poses from spawn_poses in config/floor_plan.yaml):
  * spawn from /robotN/robot_description, as model robotN. The caller runs the per-robot
    robot_state_publisher (mrs_bringup one_robot_stack.launch.py namespace:=robotN), with
    the xacro processed as prefix:=robotN/ namespace:=robotN, so gz topics are robotN/... and
    frames robotN/...
  * a parameter_bridge in /robotN, config generated from bridge.yaml: gz topic robotN/scan
    <-> ROS /robotN/scan, and so on
  * an image_bridge for robotN/camera/image_raw -> /robotN/camera/image_raw
The world is worlds/floor_novictim.sdf and the victim is spawned at run time, so start
poses and victim both come from plan_file (default config/floor_plan.yaml; S8 trials pass
their own: spawn_poses + victim, same keys).
There is one /clock bridge for all robots. No odom TF is bridged: each robot's EKF owns
robotN/odom -> robotN/base_footprint. For a single un-namespaced robot use
sim_one_robot.launch.py instead.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml


def robot_names(n):
    return [f'robot{i + 1}' for i in range(n)]


def bridge_config(template, ns):
    """bridge.yaml with gz topics moved under ns/ and without the clock (bridged once)."""
    with open(template) as f:
        entries = [e for e in yaml.safe_load(f) if e['ros_topic_name'] != 'clock']
    for e in entries:
        e['gz_topic_name'] = f"{ns}/{e['gz_topic_name']}"
    out = os.path.join(tempfile.mkdtemp(prefix=f'mrs_bridge_{ns}_'), 'bridge.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(entries, f, sort_keys=False)
    return out


def launch_setup(context):
    gz_pkg = get_package_share_directory('mrs_gazebo')
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    n = int(LaunchConfiguration('n').perform(context))
    plan_file = LaunchConfiguration('plan_file').perform(context) or os.path.join(
        gz_pkg, 'config', 'floor_plan.yaml')
    with open(plan_file) as f:
        plan = yaml.safe_load(f)
    poses = plan['spawn_poses']
    victim_model = LaunchConfiguration('victim_model').perform(context)
    names = robot_names(n)
    missing = [r for r in names if r not in poses]
    if missing:
        raise RuntimeError(f'no spawn_poses for {missing} in floor_plan.yaml')

    gz_args = f'-r -v 3 {world}' + (' -s --headless-rendering' if headless else '')
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
        # The victim box (model://victim: ArUco on its +x face), at the plan's victim pose.
        Node(package='ros_gz_sim', executable='create', output='screen', arguments=[
            '-name', 'victim', '-x', str(plan['victim'][0]), '-y', str(plan['victim'][1]),
            # the ArUco box's origin is its centre; victim_person's is on the floor
            '-z', str(plan['aruco']['victim_size'] / 2 if victim_model == 'victim' else 0.0),
            '-Y', str(plan['victim'][2]),
            '-string', '<?xml version="1.0"?><sdf version="1.9"><include>'
                       f'<uri>model://{victim_model}</uri></include></sdf>']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': True}]),
    ]
    template = os.path.join(gz_pkg, 'config', 'bridge.yaml')
    for ns in names:
        x, y, yaw = poses[ns]
        actions += [
            Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', f'/{ns}/robot_description', '-name', ns,
                            '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)]),
            GroupAction([
                PushRosNamespace(ns),
                Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                     parameters=[{'config_file': bridge_config(template, ns),
                                  'use_sim_time': True}]),
            ]),
            # image_bridge names the ROS topic after the gz topic, so it runs un-namespaced.
            Node(package='ros_gz_image', executable='image_bridge', name=f'{ns}_image_bridge',
                 output='screen', arguments=[f'{ns}/camera/image_raw'],
                 parameters=[{'use_sim_time': True}]),
        ]
    return actions


def generate_launch_description():
    gz_pkg = get_package_share_directory('mrs_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('n', default_value='2', description='number of robots (1-3)'),
        DeclareLaunchArgument('world', default_value=os.path.join(gz_pkg, 'worlds', 'floor_novictim.sdf')),
        DeclareLaunchArgument('victim_model', default_value='victim',
                              description='victim (ArUco box) | victim_person (doll, for YOLO)'),
        DeclareLaunchArgument('plan_file', default_value='',
                              description='spawn_poses + victim (default: config/floor_plan.yaml)'),
        DeclareLaunchArgument('headless', default_value='false', description='Run gz server only'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gz_pkg, 'models')),
        OpaqueFunction(function=launch_setup),
    ])
