"""Nano fallback profile: ZED SDK Positional Tracking + slam_toolbox 2D.

Substitutes for FAST-LIO2 on Orin Nano 8GB which lacks the headroom for full
LiDAR-inertial SLAM. Visual-inertial via ZED SDK provides /zed/.../odom; we
project the Ouster cloud to a 2D laser scan for slam_toolbox.

Consumes:
  /ouster/points              -> projected to /scan via pointcloud_to_laserscan
  /zed/zed_node/odom          (ZED SDK Positional Tracking)
  /zed/zed_node/imu/data

Publishes:
  /map      (occupancy grid)
  /scan     (2D laser from Ouster projection)
  /odometry/dlio (alias for ZED odom, EKF expects this topic)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('deimos_autonomy')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Project 3D Ouster cloud to a 2D laser scan slice
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc_to_scan',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'base_link',
                'transform_tolerance': 0.1,
                'min_height': 0.1,
                'max_height': 1.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0087,    # 0.5 deg
                'scan_time': 0.1,
                'range_min': 0.5,
                'range_max': 50.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/ouster/points'),
                ('scan', '/scan'),
            ],
        ),

        # 2D SLAM in async mode (lighter than sync)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_2d.yaml']),
                {'use_sim_time': use_sim_time},
            ],
        ),

        # Bridge ZED odom -> /odometry/dlio so EKF config doesn't change between profiles
        Node(
            package='topic_tools',
            executable='relay',
            name='zed_odom_bridge',
            arguments=['/zed/zed_node/odom', '/odometry/dlio'],
            output='log',
        ),
    ])
