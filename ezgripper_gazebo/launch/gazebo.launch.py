#! /usr/bin/env python3
"""
Spawn Robot in Gazebo
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Function
    """

    # .................. Configurable Arguments .....................

    gui = False
    use_sim_time = True

    world_name = 'mars.world'
    # ...............................................................

    pkg_dir = get_package_share_directory('ezgripper_gazebo')

    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_dir, 'models')


    return LaunchDescription([

        DeclareLaunchArgument('gui', \
            default_value=str(gui), \
                description='Flag to enable joint_state_publisher_gui'),

        DeclareLaunchArgument("use_sim_time", \
            default_value=str(use_sim_time), \
                description="Use simulation/Gazebo clock"),

        DeclareLaunchArgument("world_name", \
            default_value=world_name, \
                description="Choice of Gazebo World"),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', \
                [os.path.join(pkg_dir, 'worlds/'), \
                    LaunchConfiguration("world_name")],
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'ezgripper_single_mount',
                       '-x', '0',
                       '-y', '0',
                       '-z', '0'
                      ],
            output='screen'
        ),

        IncludeLaunchDescription( \
            PythonLaunchDescriptionSource( \
                os.path.join(get_package_share_directory("ezgripper_description"), \
                    'launch', 'description.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                }.items(),
        ),

        IncludeLaunchDescription( \
            PythonLaunchDescriptionSource( \
                os.path.join(get_package_share_directory("ezgripper_control"), \
                    'launch', 'control.launch.py')),
            launch_arguments={
                }.items(),
        ),

    ])
