#! /usr/bin/env python3
"""
Control Gazebo simulation through RViz2 using MoveIt2!
"""

import os
import xacro
import yaml
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


PKG_DIR = get_package_share_directory("ezgripper_dual_gen2_triple_mount_moveit_config")


def load_params(file_path, is_yaml = True):
    """
    Load parameters from a specified file
    """
    absolute_file_path = os.path.join(PKG_DIR, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            if is_yaml:
                return yaml.safe_load(file)

            return file.read()
    except EnvironmentError:
        return None


rviz_config = os.path.join(PKG_DIR, 'launch', 'moveit.rviz')

robot_description_config = xacro.process_file( \
        os.path.join(get_package_share_directory("ezgripper_description"), \
            "urdf", "dual_gen2_triple_mount", \
                "ezgripper_dual_gen2_triple_mount_standalone.urdf.xacro")
    )

robot_description = {"robot_description": robot_description_config.toxml()}

robot_description_semantic = { \
    "robot_description_semantic": load_params( \
        "config/ezgripper_dual_gen2_triple_mount.srdf", is_yaml= False)}

kinematics_yaml = load_params("config/kinematics.yaml")

ompl_planning_pipeline_config = {
    "planning_pipelines": ["ompl"],
    "default_planning_pipeline": "ompl",
    "ompl": {"planning_plugin": "ompl_interface/OMPLPlanner"}
}

ompl_planning_yaml = load_params("config/ompl_planning.yaml")
ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

simple_moveit_controllers_yaml = load_params("config/simple_moveit_controllers.yaml")
trajectory_execution_yaml = load_params("config/trajectory_execution.yaml")
planning_scene_yaml = load_params("config/planning_scene.yaml")
joint_limits_yaml = load_params("config/joint_limits.yaml")


def generate_launch_description():
    """
    Launch Function
    """

    return LaunchDescription(
        [

            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),

            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_pipeline_config,
                    trajectory_execution_yaml,
                    simple_moveit_controllers_yaml,
                    planning_scene_yaml,
                    joint_limits_yaml
                ]
            ),

            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    ompl_planning_pipeline_config,
                    kinematics_yaml
                ],
            ),

			IncludeLaunchDescription( \
				PythonLaunchDescriptionSource( \
					os.path.join(get_package_share_directory("ezgripper_driver"), \
						'launch', 'action_server.launch.py')),
                launch_arguments={
                    'ezgripper_module': 'dual_gen2_triple_mount',
                    }.items(),
			),

    ])
