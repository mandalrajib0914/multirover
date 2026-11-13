"""Nav2 for one robot, optionally namespaced (S4 single robot, S6 one per robot).

  ros2 launch mrs_navigation nav2.launch.py                           # un-prefixed robot
  ros2 launch mrs_navigation nav2.launch.py namespace:=robot1         # frames robot1/...

Same nodes, remaps and lifecycle order as nav2_bringup's navigation_launch.py, with three
differences:
  * No /tf -> tf remap. All robots share the global /tf with robotN/ frame prefixes (one TF
    tree: map -> robotN/map -> robotN/odom -> robotN/base_footprint), as the rest of the
    stack does. navigation_launch.py would move Nav2 onto /robotN/tf.
  * The params file is rendered per robot: frame values get the robotN/ prefix (by key name,
    FRAME_KEYS), "<robot_namespace>" becomes "" or "/robotN", and everything is wrapped
    under the namespace so the node-name keys match /robotN/<node>.
  * No route_server or docking_server (unused).
  * global_map:=merged plans on the merged world map instead of the robot's own (S6).
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
import yaml

# Parameters that hold a frame name and get the robotN/ prefix.
FRAME_KEYS = {'global_frame', 'robot_base_frame', 'local_frame', 'base_frame_id',
              'odom_frame_id', 'global_frame_id', 'base_frame', 'fixed_frame'}

LIFECYCLE_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                   'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']


def prefix_frames(node, prefix):
    if isinstance(node, dict):
        return {k: (prefix + v if k in FRAME_KEYS and isinstance(v, str) and not v.startswith(prefix)
                    else prefix_frames(v, prefix)) for k, v in node.items()}
    return node


def render_params(path, ns, merged_map=False):
    """Per-robot copy of a Nav2 params file; returns the path of the rendered file."""
    with open(path) as f:
        text = f.read().replace('<robot_namespace>', f'/{ns}' if ns else '')
    params = yaml.safe_load(text)
    if ns:
        params = prefix_frames(params, f'{ns}/')
    if merged_map:
        # Plan on the merged world map: frame "map", topic /map (simple_map_merge). The robot's
        # own SLAM still localizes it (robotN/map -> robotN/odom); the local costmap stays in
        # robotN/odom. Without this, a goal outside the robot's own SLAM map (e.g. a frontier
        # another robot found) fails at once: "Goal Coordinates ... was outside bounds".
        gc = params['global_costmap']['global_costmap']['ros__parameters']
        gc['global_frame'] = 'map'
        gc['static_layer']['map_topic'] = '/map'
        params['bt_navigator']['ros__parameters']['global_frame'] = 'map'
        params['behavior_server']['ros__parameters']['global_frame'] = 'map'
    if ns:
        params = {ns: params}
    out = os.path.join(tempfile.mkdtemp(prefix='mrs_nav2_'), 'nav2_params.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)
    return out


def launch_setup(context):
    ns = LaunchConfiguration('namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params_file = LaunchConfiguration('params_file').perform(context) or os.path.join(
        get_package_share_directory('mrs_navigation'), 'config', 'nav2_params.yaml')
    log_level = LaunchConfiguration('log_level').perform(context)
    merged = LaunchConfiguration('global_map').perform(context) == 'merged'
    params = render_params(params_file, ns, merged)

    def nav_node(package, executable, name=None, cmd_vel_nav=False):
        return Node(package=package, executable=executable, name=name or executable,
                    output='screen', parameters=[params],
                    arguments=['--ros-args', '--log-level', log_level],
                    remappings=[('cmd_vel', 'cmd_vel_nav')] if cmd_vel_nav else [])

    # cmd_vel chain: controller / behaviors -> cmd_vel_nav -> velocity_smoother
    #   -> cmd_vel_smoothed -> collision_monitor -> cmd_vel
    # Absolute namespace: this file is also included from inside one_robot_stack's own
    # PushRosNamespace(robotN), and a relative push would nest it to /robotN/robotN.
    actions = [PushRosNamespace('/' + ns)] + [
        SetParameter('use_sim_time', use_sim_time),
        nav_node('nav2_controller', 'controller_server', cmd_vel_nav=True),
        nav_node('nav2_smoother', 'smoother_server'),
        nav_node('nav2_planner', 'planner_server'),
        nav_node('nav2_behaviors', 'behavior_server', cmd_vel_nav=True),
        nav_node('nav2_bt_navigator', 'bt_navigator'),
        nav_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav_node('nav2_velocity_smoother', 'velocity_smoother', cmd_vel_nav=True),
        nav_node('nav2_collision_monitor', 'collision_monitor'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             arguments=['--ros-args', '--log-level', log_level],
             parameters=[{'autostart': LaunchConfiguration('autostart').perform(context) == 'true',
                          'node_names': LIFECYCLE_NODES}]),
    ]
    return [SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'), GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='e.g. robot1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='default: mrs_navigation/config/nav2_params.yaml'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('global_map', default_value='own',
                              description="own: plan on this robot's SLAM map; merged: on /map in frame map"),
        OpaqueFunction(function=launch_setup),
    ])
