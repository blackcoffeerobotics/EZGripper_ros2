#! /usr/bin/env python3
"""
Launch the Gripper Action Server node
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    """
    Launch Function
    """

    pkg_dir = get_package_share_directory('ezgripper_control')


    return LaunchDescription([

        Node(
            package='ezgripper_driver',
            executable='ezgripper_action_server.py',
            name='ezgripper_controller',
            parameters=[os.path.join(pkg_dir, 'config', 'gripper_params_single_mount.yaml')],
            output='screen',
            emulate_tty=True,
        )

    ])
