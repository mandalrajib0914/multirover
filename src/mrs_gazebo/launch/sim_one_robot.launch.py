"""Gazebo Harmonic + one robot + ros_gz bridges (+ RViz).

  ros2 launch mrs_gazebo sim_one_robot.launch.py
  ros2 launch mrs_gazebo sim_one_robot.launch.py headless:=true rviz:=false

odom_tf:=true  bridges the DiffDrive odom -> base_footprint TF (bare S2 sim, no EKF).
odom_tf:=false leaves that TF to the EKF (mrs_bringup, S3+).
rsp:=false     skips robot_state_publisher (the caller runs its own, e.g. one_robot_stack).

The spawn pose defaults to robot1 in config/floor_plan.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context):
    gz_pkg = get_package_share_directory('mrs_gazebo')
    desc_pkg = get_package_share_directory('mrs_description')

    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context)
    odom_tf = LaunchConfiguration('odom_tf').perform(context).lower() == 'true'
    with_rsp = LaunchConfiguration('rsp').perform(context).lower() == 'true'

    with open(os.path.join(gz_pkg, 'config', 'floor_plan.yaml')) as f:
        pose = yaml.safe_load(f)['spawn_poses']['robot1']
    # explicit launch args override the floor plan
    x, y, yaw = (LaunchConfiguration(k).perform(context) or v for k, v in zip(('x', 'y', 'yaw'), pose))

    # -r: start unpaused. -s --headless-rendering: server only, sensors still render (EGL).
    gz_args = f'-r -v 3 {world}' + (' -s --headless-rendering' if headless else '')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_pkg, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': rviz,
            'rviz_config': os.path.join(desc_pkg, 'rviz', 'sim.rviz'),
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'robot1',
                   '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw)],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(gz_pkg, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )
    tf_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='odom_tf_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(gz_pkg, 'config', 'bridge_odom_tf.yaml'),
                     'use_sim_time': True}],
    )

    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge', output='screen',
        arguments=['camera/image_raw'],
        parameters=[{'use_sim_time': True}],
    )

    return ([gazebo] + ([rsp] if with_rsp else []) + [spawn, bridge, image_bridge]
            + ([tf_bridge] if odom_tf else []))


def generate_launch_description():
    gz_pkg = get_package_share_directory('mrs_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(gz_pkg, 'worlds', 'floor.sdf')),
        DeclareLaunchArgument('headless', default_value='false', description='Run gz server only'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('odom_tf', default_value='true',
                              description='Bridge DiffDrive odom TF (false when the EKF runs)'),
        DeclareLaunchArgument('rsp', default_value='true',
                              description='Start robot_state_publisher here'),
        DeclareLaunchArgument('x', default_value='', description='Spawn x (default: floor_plan.yaml robot1)'),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        # Lets the world resolve model://victim
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gz_pkg, 'models')),
        OpaqueFunction(function=launch_setup),
    ])
