"""Nav2 launch — local + global planner + costmaps + behavior tree.

Consumes:
  /odometry/filtered     (local EKF)
  /odometry/filtered_map (global EKF)
  /map                   (occupancy from slam_toolbox 2D, or custom from FAST-LIO)
  /plan                  (overridden by dem_global_planner — see autonomy.launch.py)

Publishes:
  /cmd_vel               (consumed by wheels_controller_node)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('deimos_autonomy')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    nav2_bt = PathJoinSubstitution([pkg_share, 'config', 'nav2_bt_navigate.xml'])

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),

        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[nav2_params, {'use_sim_time': use_sim_time}]),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_params, {'use_sim_time': use_sim_time}]),

        Node(package='nav2_behaviors', executable='behavior_server',
             name='recoveries_server', output='screen',
             parameters=[nav2_params, {'use_sim_time': use_sim_time}]),

        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_params, {'use_sim_time': use_sim_time,
                                       'default_nav_to_pose_bt_xml': nav2_bt}]),

        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen',
             parameters=[nav2_params, {'use_sim_time': use_sim_time}]),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'node_names': lifecycle_nodes}]),
    ])
