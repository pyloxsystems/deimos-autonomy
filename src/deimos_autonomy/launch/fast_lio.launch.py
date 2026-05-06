# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Pylox Systems (https://pyloxforge.com)
# Licensed under the PolyForm Noncommercial License 1.0.0.
# See LICENSE or https://polyformproject.org/licenses/noncommercial/1.0.0
# Required Notice: Copyright Pylox Systems (https://pyloxforge.com)

"""FAST-LIO2 launch — Ouster OS1 + Ouster IMU.

Consumes:
  /ouster/points (sensor_msgs/PointCloud2) — Ouster 3D point cloud
  /ouster/imu    (sensor_msgs/Imu)         — Ouster built-in 9-DOF IMU

Publishes:
  /odometry/dlio (nav_msgs/Odometry) — fused LiDAR-inertial odometry
  /map           (sensor_msgs/PointCloud2) — accumulated map points
  /path          (nav_msgs/Path)
  TF: odom -> base_link

Tune `config/fast_lio_ouster.yaml` for your specific OS1 beam count (32/64/128).
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

        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'fast_lio_ouster.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                # FAST-LIO defaults to /livox/lidar and /livox/imu
                # Remap to Ouster topics
                ('/livox/lidar', '/ouster/points'),
                ('/livox/imu', '/ouster/imu'),
                # Output remap: FAST-LIO publishes /Odometry by default
                ('/Odometry', '/odometry/dlio'),
            ],
        ),
    ])
