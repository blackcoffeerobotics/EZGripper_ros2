#! /usr/bin/env python3
"""
Spawn Robot in RViz
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node



def generate_launch_description():
    """
    Launch Function
    """
    # .................. Configurable Arguments .....................

    gui = True
    # ...............................................................


    pkg_dir = get_package_share_directory('ezgripper_description')


    return LaunchDescription([

        # Launch Arguments
        DeclareLaunchArgument('gui', \
            default_value=str(gui), \
                description='Flag to enable joint_state_publisher_gui'),

        # Nodes
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui')),
            remappings={'/joint_states': '/ezgripper_single_mount/joint_states'}.items()
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui')),
            remappings={'/joint_states': '/ezgripper_single_mount/joint_states'}.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command( \
                ['xacro ', os.path.join(pkg_dir, 'urdf', 'ezgripper_single_mount_standalone.urdf.xacro')])}],
            remappings={'/joint_states': '/ezgripper_single_mount/joint_states'}.items()
        )

    ])
