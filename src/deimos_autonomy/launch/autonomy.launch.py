"""Master autonomy launch file for DEIMOS rover.

Switches between compute profiles via the `compute_profile` launch arg:
  - nx_16gb: FAST-LIO2 + full stack (PRIMARY)
  - nano_8gb: ZED SDK VIO + slam_toolbox 2D (FALLBACK)

Both profiles share the same EKF, Nav2, ArUco action server, and YOLO inference.

Usage:
    ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nx_16gb
    ros2 launch deimos_autonomy autonomy.launch.py compute_profile:=nano_8gb
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('deimos_autonomy')

    compute_profile = LaunchConfiguration('compute_profile')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_args = [
        DeclareLaunchArgument(
            'compute_profile',
            default_value='nano_8gb',
            description='Compute profile: nano_8gb (slam_toolbox 2D + ZED VIO) or nx_16gb (FAST-LIO2). Both supported.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (Gazebo / rosbag replay)'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Auto-start Nav2 lifecycle nodes'
        ),
    ]

    nx_slam = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'fast_lio.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=LaunchConfigurationEquals('compute_profile', 'nx_16gb'),
        ),
    ])

    nano_slam = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'slam_toolbox_2d.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=LaunchConfigurationEquals('compute_profile', 'nano_8gb'),
        ),
    ])

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'ekf.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'compute_profile': compute_profile,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, 'launch', 'nav2.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items(),
    )

    dem_planner = Node(
        package='deimos_autonomy',
        executable='dem_global_planner',
        name='dem_global_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'dem_directory': os.path.expanduser('~/deimos_data/mdrs_dem'),
            'utm_zone': 12,
            'utm_north': True,
        }],
    )

    aruco_action = Node(
        package='deimos_autonomy',
        executable='aruco_action_server',
        name='aruco_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/zed/zed_node/rgb/image_rect_color',
            'approach_distance': 1.0,
            'servo_kp_linear': 0.3,
            'servo_kp_angular': 0.6,
            'cmd_vel_topic': '/cmd_vel',
        }],
    )

    yolo_node = Node(
        package='deimos_autonomy',
        executable='yolo_trt_node',
        name='yolo_trt_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'engine_path_nx': os.path.expanduser('~/deimos_data/yolov8n_urc_fp16.engine'),
            'engine_path_nano': os.path.expanduser('~/deimos_data/yolov8n_urc_int8.engine'),
            'compute_profile': compute_profile,
            'image_topic': '/zed/zed_node/rgb/image_rect_color',
            'detection_topic': '/yolo/detections',
            'confidence_threshold': 0.45,
        }],
    )

    sequencer = Node(
        package='deimos_autonomy',
        executable='goal_sequencer',
        name='goal_sequencer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_file': os.path.expanduser('~/deimos_data/urc_waypoints.yaml'),
        }],
    )

    return LaunchDescription(
        declare_args + [
            nx_slam,
            nano_slam,
            ekf,
            nav2,
            dem_planner,
            aruco_action,
            yolo_node,
            sequencer,
        ]
    )
