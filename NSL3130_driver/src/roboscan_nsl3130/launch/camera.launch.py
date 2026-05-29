#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('roboscan_nsl3130')
    param_path = os.path.join(pkg_share, 'rqt.yaml')
    params = [param_path] if os.path.exists(param_path) else []

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'roboscan_nsl3130rviz.rviz'
    )

    # Standard colcon layout: install/<pkg>/share/<pkg>/ → 4 levels up = workspace root
    # Calibration files live in <workspace>/src/NSL-3130AA-ROS2/calib_output/
    ws_root = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    calib_dir = os.environ.get(
        'NSL_CALIB_DIR',
        os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output'))


    return LaunchDescription([
        # ── General ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch rviz2 with the default config'),
        DeclareLaunchArgument(
            'use_rqt',
            default_value='true',
            description='Launch rqt_reconfigure for parameter tuning'),
        # ── Topic remap targets (azure_kinect-compatible) ─────────────────────
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/rgb/image_raw',
            description='Remap target for roboscanImage (RGB)'),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Remap target for roboscanDistance (depth)'),
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/camera/point_cloud',
            description='Remap target for roboscanPointCloud (XYZI)'),
        DeclareLaunchArgument(
            'point_cloud_rgb_topic',
            default_value='/camera/point_cloud_rgb',
            description='Remap target for roboscanPointCloudRgb (XYZRGB)'),
        # ── Driver node ───────────────────────────────────────────────────────
        Node(
            package='roboscan_nsl3130',
            executable='roboscan_publish_node',
            output='screen',
            parameters=params if params else None,
            additional_env={'NSL_CALIB_DIR': calib_dir},
            remappings=[
                ('roboscanImage',         LaunchConfiguration('rgb_topic')),
                ('roboscanDistance',      LaunchConfiguration('depth_topic')),
                ('roboscanPointCloud',    LaunchConfiguration('point_cloud_topic')),
                ('roboscanPointCloudRgb', LaunchConfiguration('point_cloud_rgb_topic')),
            ]),
        # ── rviz2 ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=conditions.IfCondition(LaunchConfiguration('use_rviz'))),
        # ── rqt parameter reconfigure (delayed to let the node spin first) ──────
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui',
                         '--force-discover', '-s', 'rqt_reconfigure_combo'],
                    output='screen')
            ],
            condition=conditions.IfCondition(LaunchConfiguration('use_rqt'))),
    ])
