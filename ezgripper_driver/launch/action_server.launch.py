#! /usr/bin/env python3
"""
Launch the Gripper Action Server node
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    ezgripper_module = 'dual_gen2_single_mount'

    # ...............................................................


    pkg_dir = get_package_share_directory('ezgripper_control')


    return LaunchDescription([

        DeclareLaunchArgument('ezgripper_module', \
            default_value=ezgripper_module, \
                description='Required module of ezgripper'),

        Node(
            package='ezgripper_driver',
            executable='ezgripper_action_server.py',
            name='ezgripper_controller',
            parameters=[[os.path.join(pkg_dir, 'config/'), \
                "gripper_params_", LaunchConfiguration("ezgripper_module"), ".yaml"]],
            output='screen',
            emulate_tty=True,
        )

    ])
