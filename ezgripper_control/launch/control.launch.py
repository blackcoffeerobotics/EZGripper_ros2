#! /usr/bin/env python3
"""
Spawn Controller Manager
"""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    ezgripper_module = 'dual_gen2_single_mount'

    # ...............................................................


    return LaunchDescription([

        DeclareLaunchArgument('ezgripper_module', \
            default_value=ezgripper_module, \
                description='Required module of ezgripper'),


        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["ezgripper_controller",
                    "--controller-manager", ["/ezgripper_", \
                        LaunchConfiguration("ezgripper_module"), "/controller_manager"],
                    "--controller-manager-timeout", "10000"],
        ),

    ])
