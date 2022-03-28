#! /usr/bin/env python3
"""
Spawn Robot in RViz
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    """
    Launch Function
    """
    # .................. Configurable Arguments .....................

    gui = True
    rviz_config = 'urdf.rviz'

    ezgripper_module = 'dual_gen1'

    # ...............................................................


    pkg_dir = get_package_share_directory('ezgripper_driver')

    return LaunchDescription([

        # Launch Arguments
        DeclareLaunchArgument('gui', \
            default_value=str(gui), \
                description='Flag to enable joint_state_publisher_gui'),

        DeclareLaunchArgument("rviz_config", \
            default_value=rviz_config, \
                description="RViz configuration file"),

        DeclareLaunchArgument('ezgripper_module', \
            default_value=ezgripper_module, \
                description='Required module of ezgripper'),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', [os.path.join(pkg_dir, 'rviz/'), \
                LaunchConfiguration("rviz_config")]],
        ),

        IncludeLaunchDescription( \
            PythonLaunchDescriptionSource( \
                os.path.join(get_package_share_directory("ezgripper_description"), \
                    'launch', 'description.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'ezgripper_module': LaunchConfiguration('ezgripper_module'),
                }.items(),
        )


    ])
