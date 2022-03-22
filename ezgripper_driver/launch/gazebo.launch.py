#! /usr/bin/env python3
"""
Spawn Robot in Gazebo
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    use_sim_time = True

    ezgripper_module = 'dual_gen2'

    # ...............................................................


    pkg_dir = get_package_share_directory('ezgripper_driver')


    return LaunchDescription([

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument('ezgripper_module', \
            default_value=ezgripper_module, \
                description='Required module of ezgripper'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', \
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command( \
                ['xacro ', os.path.join(pkg_dir, 'urdf/'), \
                    LaunchConfiguration("ezgripper_module"), "/", \
                      "ezgripper_", LaunchConfiguration("ezgripper_module"), \
                          '_standalone.urdf.xacro'
                ])}]
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["ezgripper_controller",
                    "--controller-manager", "/controller_manager",
                    "--controller-manager-timeout", "10000"],
        ),

        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', LaunchConfiguration("ezgripper_module"),
                       '-x', '0',
                       '-y', '0',
                       '-z', '0'
                      ],
            output='screen'
        ),

    ])
