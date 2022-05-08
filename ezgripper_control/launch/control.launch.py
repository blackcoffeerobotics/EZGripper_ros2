#! /usr/bin/env python3
"""
Spawn Controller Manager
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Function
    """

    return LaunchDescription([

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["ezgripper_controller",
                    "--controller-manager", ["/ezgripper_single_mount", \
                        "/controller_manager"],
                    "--controller-manager-timeout", "10000"],
        ),

    ])
