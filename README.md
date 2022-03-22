# EZGripper

A ROS package that serves as a driver to the [EZGripper module](https://sakerobotics.com/) designed by SAKE Robotics. If you are not using ROS, use https://github.com/SAKErobotics/SAKErobotics

## Tutorial

### Hardware
---

* Install the python EZGripper library. For python2 use [this link](https://github.com/SAKErobotics/libezgripper/tree/master) and for python3 use [the ubuntu 20.04 branch](https://github.com/SAKErobotics/libezgripper/tree/ubuntu-20.04).

* Install `gazebo_ros2_control` to enable Gazebo to mimic the EZGripper joints:

	  git clone https://github.com/leander-dsouza/gazebo_ros2_control -b foxy

* Install all the remaining dependencies using `rosdep`, at the root of your ROS workspace:

	  rosdep install --from-paths src --ignore-src -r -y

* Clone the ROS Driver at you `src` folder:

   	  git clone --branch=foxy-devel https://github.com/SAKErobotics/EZGripper.git

* Build your workspace and source it:

	  colcon build --symlink-install --packages-select ezgripper_driver gazebo_ros2_control ezgripper_dual_gen1_moveit_config  ezgripper_dual_gen2_moveit_config ezgripper_quad_moveit_config --allow-overriding ezgripper_driver gazebo_ros2_control ezgripper_dual_gen1_moveit_config  ezgripper_dual_gen2_moveit_config ezgripper_quad_moveit_config && source install/setup.bash


### Software
---

* Set the bash variable according to your gripper module - (`dual_gen1`, `dual_gen2`, `quad`):

	  export ezgripper_module=<your_gripper_module>

* Launch the gripper module in RViz :

	  ros2 launch ezgripper_driver display.launch.py ezgripper_module:=${ezgripper_module}

* Similarly to launch in Gazebo:

	  ros2 launch ezgripper_driver gazebo.launch.py ezgripper_module:=${ezgripper_module}

* To actuate the gripper into its respective open/close configurations in Gazebo:

	  # Open Gripper
	  ros2 run ezgripper_driver open_gripper ezgripper_module:=${ezgripper_module}

	  # Close Gripper
	  ros2 run ezgripper_driver close_gripper ezgripper_module:=${ezgripper_module}

* Result of actuation:

	![ezgripper_gif](https://user-images.githubusercontent.com/45683974/156144685-b91e2684-b484-4067-aee8-527d2e223650.gif)

### MoveIt!
---

* To launch the ezgripper in Gazebo and RViz for control:

	  ros2 launch ezgripper_${ezgripper_module}_moveit_config demo_gazebo.launch.py

* To control the ezgripper hardware through MoveIt!:

	  ros2 launch ezgripper_${ezgripper_module}_moveit_config ezgripper_${ezgripper_module}_moveit_planning_execution.launch.py

## URDF Models
---

Access the URDF [models](https://github.com/SAKErobotics/EZGripper/tree/master/ezgripper_driver/urdf) for additional information.


## TroubleShooting
---

### Serial connection issues:

* The following message indicates you have a new version of serial library that causes issues.

	  Error message: 'Serial' object has no attribute 'setParity'  ---

  Do the following command to load an older serial library.

	  sudo apt-get install python3-serial==2.0.0

* This indicates the user does not have privileges to use the `/dev/ttyUSBx`:

	  Error message: permission denied (get accurate error message).

	The solution is to add the `<user>` to the `dialout` group.  After executing the following command, reboot.

	  sudo adduser <user> dialout
	  reboot
