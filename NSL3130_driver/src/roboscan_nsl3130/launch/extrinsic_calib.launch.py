#!/usr/bin/env python3
"""
Extrinsic calibration launcher — wraps extrinsic_calib.sh.
Camera serial is auto-detected via detect_camera_id.py if camera_id is not provided.

Usage:
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py camera_id:=N00A5060D
  ros2 launch roboscan_nsl3130 extrinsic_calib.launch.py lidar_topic:=/camera/point_cloud

Requires: camera.launch.py running + intrinsic_calib done first.
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    camera_id   = LaunchConfiguration('camera_id').perform(context)
    image_topic = LaunchConfiguration('image_topic').perform(context)
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)

    pkg_share     = get_package_share_directory('roboscan_nsl3130')
    ws_root       = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    detect_script = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2',
                                 'NSL3130_driver', 'src', 'roboscan_nsl3130',
                                 'scripts', 'detect_camera_id.py')
    sh_path       = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'extrinsic_calib.sh')

    if not camera_id:
        try:
            camera_id = subprocess.check_output(
                ['python3', detect_script], text=True, stderr=subprocess.DEVNULL).strip()
            print(f'[extrinsic_calib] Auto-detected camera_id: {camera_id}')
        except subprocess.CalledProcessError:
            print('[extrinsic_calib] WARNING: Camera not detected.'
                  ' Pass camera_id:=<serial> to override.'
                  ' extrinsic_calib.sh will fall back to intrinsic.yml.')

    return [ExecuteProcess(
        cmd=['bash', sh_path, camera_id, image_topic, lidar_topic],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_id',    default_value='',
            description='Camera serial; auto-detected via detect_camera_id.py if empty'),
        DeclareLaunchArgument('image_topic',  default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('lidar_topic',  default_value='/camera/point_cloud'),
        OpaqueFunction(function=_launch_setup),
    ])
