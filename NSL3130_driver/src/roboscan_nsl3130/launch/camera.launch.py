#!/usr/bin/env python3

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('roboscan_nsl3130')
    param_path = os.path.join(pkg_share, 'rqt.yaml')
    params = [param_path] if os.path.exists(param_path) else []

    rviz_config = os.path.join(pkg_share, 'rviz', 'roboscan_nsl3130rviz.rviz')

    # Standard colcon layout: install/<pkg>/share/<pkg>/ → 4 levels up = workspace root
    ws_root = os.path.normpath(os.path.join(pkg_share, '..', '..', '..', '..'))
    calib_dir = os.environ.get(
        'NSL_CALIB_DIR',
        os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'calib_output'))

    repo_scripts = os.path.join(ws_root, 'src', 'NSL-3130AA-ROS2', 'NSL3130_driver',
                                'src', 'roboscan_nsl3130', 'scripts')
    extrinsic_tf_script = os.path.join(repo_scripts, 'extrinsic_tf_node.py')
    detect_script       = os.path.join(repo_scripts, 'detect_camera_id.py')

    # Sensor-tuning profiles. The driver reads the file in NSL_PARAMS_FILE
    # (see roboscan_publish_node.cpp). Defaults are the repo (zzapzzap) baseline.
    general_params = os.path.join(pkg_share, 'lidar_params.yaml')
    calib_params   = os.path.join(pkg_share, 'lidar_params_calibration.yaml')

    def _resolve_params_file(context):
        """Pick the sensor-params file for the driver.

        Priority:
          calibration:=true  → shared calibration profile
          else, USB serial detected:
              {calib_dir}/{serial}_params.yaml  (created from the general default
              on first run, so each camera keeps & persists its own tuning)
          else (no serial)   → general default (zzapzzap baseline)
        """
        calibration = (LaunchConfiguration('calibration')
                       .perform(context).strip().lower() in ('true', '1', 'yes'))
        if calibration:
            print(f'[camera] sensor params: {calib_params}  [calibration profile]')
            return calib_params

        serial = ''
        try:
            serial = subprocess.check_output(
                ['python3', detect_script], text=True,
                stderr=subprocess.DEVNULL).strip()
        except Exception:
            serial = ''

        if not serial:
            print(f'[camera] sensor params: {general_params}  '
                  f'[general default — no camera id detected]')
            return general_params

        dev_file = os.path.join(calib_dir, f'{serial}_params.yaml')
        if not os.path.exists(dev_file):
            try:
                os.makedirs(calib_dir, exist_ok=True)
                shutil.copy(general_params, dev_file)
                print(f'[camera] seeded per-device params from general default → {dev_file}')
            except OSError as e:
                print(f'[camera] could not seed {dev_file} ({e}); using general default')
                return general_params
        print(f'[camera] sensor params: {dev_file}  [per-device: {serial}]')
        return dev_file

    def _driver_setup(context):
        params_file = _resolve_params_file(context)
        return [Node(
            package='roboscan_nsl3130',
            executable='roboscan_publish_node',
            output='screen',
            parameters=params if params else None,
            additional_env={'NSL_CALIB_DIR': calib_dir,
                            'NSL_PARAMS_FILE': params_file},
            remappings=[
                ('roboscanImage',         LaunchConfiguration('rgb_topic').perform(context)),
                ('roboscanDistance',      LaunchConfiguration('depth_topic').perform(context)),
                ('roboscanPointCloud',    LaunchConfiguration('point_cloud_topic').perform(context)),
                ('roboscanPointCloudRgb', LaunchConfiguration('point_cloud_rgb_topic').perform(context)),
            ])]

    return LaunchDescription([
        # ── General ──────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch rviz2 with the default config'),
        DeclareLaunchArgument(
            'use_rqt', default_value='true',
            description='Launch rqt_reconfigure for parameter tuning'),
        DeclareLaunchArgument(
            'use_extrinsic_tf', default_value='true',
            description='Publish {lidar_frame}→{id}_camera_frame TF from the saved extrinsic'),
        DeclareLaunchArgument(
            'calibration', default_value='false',
            description='true → shared calibration sensor profile (board-tuned); '
                        'false → per-camera profile (calib_output/{id}_params.yaml), '
                        'falling back to the general default'),
        # ── Topic remap targets (azure_kinect-compatible) ─────────────────────
        DeclareLaunchArgument('rgb_topic',           default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('depth_topic',         default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('point_cloud_topic',     default_value='/camera/point_cloud'),
        DeclareLaunchArgument('point_cloud_rgb_topic', default_value='/camera/point_cloud_rgb'),
        # ── Driver node (per-device sensor profile resolved at launch) ─────────
        OpaqueFunction(function=_driver_setup),
        # ── Extrinsic → TF ({lidar_frame} → {id}_camera_frame) ────────────────
        ExecuteProcess(
            cmd=['python3', extrinsic_tf_script,
                 '--calib-dir', calib_dir,
                 '--lidar-topic', LaunchConfiguration('point_cloud_topic')],
            output='screen',
            condition=conditions.IfCondition(LaunchConfiguration('use_extrinsic_tf'))),
        # ── rviz2 ─────────────────────────────────────────────────────────────
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config], output='screen',
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
