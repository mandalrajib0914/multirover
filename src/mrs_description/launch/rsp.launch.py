"""robot_state_publisher for one robot, plus optional joint_state_publisher(_gui) and RViz.

  ros2 launch mrs_description rsp.launch.py gui:=true          # model check (S1)
  ros2 launch mrs_description rsp.launch.py use_sim_time:=true # used by the sim launch (S2)

Joint states come from:
  gui:=true            -> joint_state_publisher_gui (sliders)
  use_sim_time:=false  -> joint_state_publisher (wheels at zero)
  use_sim_time:=true   -> Gazebo (JointStatePublisher system via the bridge)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    pkg = get_package_share_directory('mrs_description')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'
    prefix = LaunchConfiguration('prefix').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'robot.urdf.xacro'),
        mappings={'prefix': prefix, 'namespace': namespace},
    ).toxml()

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        )
    ]

    if gui:
        nodes.append(Node(package='joint_state_publisher_gui',
                          executable='joint_state_publisher_gui',
                          parameters=[{'use_sim_time': use_sim_time}]))
    elif not use_sim_time:
        nodes.append(Node(package='joint_state_publisher',
                          executable='joint_state_publisher',
                          parameters=[{'use_sim_time': use_sim_time}]))

    if rviz:
        nodes.append(Node(package='rviz2', executable='rviz2',
                          arguments=['-d', rviz_config or os.path.join(pkg, 'rviz', 'view_robot.rviz')],
                          parameters=[{'use_sim_time': use_sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Run joint_state_publisher_gui (and RViz)'),
        DeclareLaunchArgument('rviz', default_value=LaunchConfiguration('gui')),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('prefix', default_value='', description='Frame prefix, e.g. robot1/'),
        DeclareLaunchArgument('namespace', default_value='', description='gz topic namespace, e.g. robot1'),
        OpaqueFunction(function=launch_setup),
    ])
