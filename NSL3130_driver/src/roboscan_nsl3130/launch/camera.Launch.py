#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()

    param_path = os.path.join(
       get_package_share_directory('roboscan_nsl3130'),
      'rqt.yaml'
    )

    params = [param_path] if os.path.exists(param_path) else []

    roboscan_publish_node = Node(
        package='roboscan_nsl3130',
        executable='roboscan_publish_node',
        output='screen',
        parameters=params if params else None
        )
 
 
#    rviz_node = Node(
#        package='rviz2',
#        executable='rviz2',
#        name='rviz2',
#        arguments=['-d' + os.path.join(get_package_share_directory('roboscan_nsl3130'), 'rviz', 'roboscan_nsl3130rviz.rviz')],output='screen')

    rqt_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_gui', 'rqt_gui'],
        output='screen')


#    launch_description.add_action(rviz_node)
    launch_description.add_action(rqt_node)
    launch_description.add_action(roboscan_publish_node)

    return launch_description
