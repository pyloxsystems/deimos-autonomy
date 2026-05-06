# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Copyright (c) 2026 Pylox Systems (https://pyloxforge.com)
# Licensed under the PolyForm Noncommercial License 1.0.0.
# See LICENSE or https://polyformproject.org/licenses/noncommercial/1.0.0
# Required Notice: Copyright Pylox Systems (https://pyloxforge.com)

"""robot_localization dual-EKF + navsat_transform launch.

Local EKF: fuses /odometry/dlio (or /zed/.../odom on Nano profile)
            + /ouster/imu + /zed/.../imu/data
            -> /odometry/filtered (continuous odom frame)

Global EKF: same inputs + GPS via navsat_transform_node
            -> /odometry/filtered_map (map frame, globally anchored)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('deimos_autonomy')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_local = PathJoinSubstitution([pkg_share, 'config', 'ekf_local.yaml'])
    ekf_global = PathJoinSubstitution([pkg_share, 'config', 'ekf_global.yaml'])
    navsat = PathJoinSubstitution([pkg_share, 'config', 'navsat_transform.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('compute_profile', default_value='nx_16gb'),

        # Local EKF — odom frame, continuous, no GPS
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local, {'use_sim_time': use_sim_time}],
            remappings=[('/odometry/filtered', '/odometry/filtered')],
        ),

        # Global EKF — map frame, GPS-corrected
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global, {'use_sim_time': use_sim_time}],
            remappings=[('/odometry/filtered', '/odometry/filtered_map')],
        ),

        # GPS -> odom transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat, {'use_sim_time': use_sim_time}],
            remappings=[
                ('/imu/data', '/zed/zed_node/imu/data'),
                ('/gps/fix', '/fix'),
                ('/odometry/filtered', '/odometry/filtered_map'),
            ],
        ),
    ])
