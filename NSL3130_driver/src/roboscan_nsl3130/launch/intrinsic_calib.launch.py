#!/usr/bin/env python3
"""
Intrinsic calibration launcher — wraps intrinsic_calib.sh.
Camera serial is auto-detected via USB if camera_id is not provided.

Calibration mode (default):
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py board_size:=8x13 square_size:=0.04

Debug / verify mode (requires existing {camera_id}_intrinsic.yml):
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py debug:=true
  ros2 launch roboscan_nsl3130 intrinsic_calib.launch.py debug:=true balance:=0.0
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    board_size  = LaunchConfiguration('board_size').perform(context)
    square_size = LaunchConfiguration('square_size').perform(context)
    image_topic = LaunchConfiguration('image_topic').perform(context)
    camera_id   = LaunchConfiguration('camera_id').perform(context)
    debug       = LaunchConfiguration('debug').perform(context).lower() == 'true'
    balance     = LaunchConfiguration('balance').perform(context)

    pkg_share     = get_package_share_directory('roboscan_nsl3130')
    ws_root       = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    scripts_dir   = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2',
                                 'NSL3130_driver', 'src', 'roboscan_nsl3130', 'scripts')
    detect_script = os.path.join(scripts_dir, 'detect_camera_id.py')
    calib_dir     = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output')

    if not camera_id:
        try:
            camera_id = subprocess.check_output(
                ['python3', detect_script], text=True, stderr=subprocess.DEVNULL).strip()
            print(f'[intrinsic_calib] Auto-detected camera_id: {camera_id}')
        except subprocess.CalledProcessError:
            print('[intrinsic_calib] WARNING: Camera not detected. Pass camera_id:=<serial>.')
            camera_id = 'nsl'

    if debug:
        verify_script = os.path.join(scripts_dir, 'verify_intrinsic.py')
        return [ExecuteProcess(
            cmd=['python3', verify_script,
                 '--camera-id',   camera_id,
                 '--calib-dir',   calib_dir,
                 '--image-topic', image_topic,
                 '--balance',     balance],
            output='screen',
        )]

    sh_path = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'intrinsic_calib.sh')
    return [ExecuteProcess(
        cmd=['bash', sh_path, board_size, square_size, image_topic, camera_id],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('board_size',   default_value='8x13',
            description='Checkerboard inner corners WxH'),
        DeclareLaunchArgument('square_size',  default_value='0.04',
            description='Square side length in metres'),
        DeclareLaunchArgument('image_topic',  default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('camera_id',    default_value='',
            description='Camera serial; auto-detected via USB if empty'),
        DeclareLaunchArgument('debug',        default_value='false',
            description='Show live rectified image using existing intrinsic.yml'),
        DeclareLaunchArgument('balance',      default_value='0.5',
            description='Undistort balance: 0.0=crop black, 1.0=keep all pixels'),
        OpaqueFunction(function=_launch_setup),
    ])
