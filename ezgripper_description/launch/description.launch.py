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

    ezgripper_module = 'dual_gen1'

    # ...............................................................


    pkg_dir = get_package_share_directory('ezgripper_description')


    return LaunchDescription([

        # Launch Arguments
        DeclareLaunchArgument('gui', \
            default_value=str(gui), \
                description='Flag to enable joint_state_publisher_gui'),


        DeclareLaunchArgument('ezgripper_module', \
            default_value=ezgripper_module, \
                description='Required module of ezgripper'),

        # Nodes
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('gui'))
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration('gui'))
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command( \
                ['xacro ', os.path.join(pkg_dir, 'urdf/'), \
                    LaunchConfiguration("ezgripper_module"), "/", \
                      "ezgripper_", LaunchConfiguration("ezgripper_module"), \
                          '_standalone.urdf.xacro'
                ])}]
        )

    ])
