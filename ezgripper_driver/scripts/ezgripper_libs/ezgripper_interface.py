#!/usr/bin/python3
"""
EZGripper Interface Module
"""

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, SAKE Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
##

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from std_srvs.srv import Empty

# http://docs.ros.org/indigo/api/control_msgs/html/msg/GripperCommand.html
# float64 position  # if 0, torque mode, if >0 to 100 correlates to 0-100% rotation range
# float64 max_effort  # if 0, torque released,  if >0 to 100 increasing torque
#
# http://docs.ros.org/indigo/api/control_msgs/html/action/GripperCommand.html
# GripperCommand command
# ---
# float64 position  # The current gripper gap size (% rotation of EZGripper fingers) (NOT in meters)
# float64 effort    # The current effort exerted (% available, NOT in Newtons)
# bool stalled      # True iff the gripper is exerting max effort and not moving
# bool reached_goal # True iff the gripper position has reached the commanded setpoint
#



OPEN_SINGLE_MOUNT_POS = -1.5707
CLOSE_SINGLE_MOUNT_POS = 0.27

MIN_SIMULATED_EFFORT = 0.0
MAX_SIMULATED_EFFORT = 1.0


def remap(input_val, in_min, in_max, out_min, out_max):
    """
    Remap Function
    """
    return (input_val - in_min) * (out_max - out_min) / \
            (in_max - in_min) + out_min


class EZGripper():
    """
    EZGripper Class
    """

    def __init__(self, node, action_name):

        self.node = node
        self.action_name = action_name

        self._grip_max = 100.0 # maximum open position for grippers - correlates to .17 meters
        self._grip_value = self._grip_max
        self._grip_min = 0.01 # if 0.0, torque mode, not position mode
        self._grip_step = self._grip_max /15 # gripper step Cross Up and Cross Down

        self._connect_to_gripper_action()
        self._connect_to_calibrate_srv()


    def _connect_to_gripper_action(self):

        self.node.get_logger().info("Waiting for action server {}...".format(self.action_name))
        self._action_client = ActionClient(self.node, GripperCommand, self.action_name)
        self._action_client.wait_for_server(60)
        self.node.get_logger().info("Connected to action server")

    def _connect_to_calibrate_srv(self):

        service_name = self.action_name + '/calibrate'
        self.cli = self.node.create_client(Empty, service_name)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        self.node.get_logger().info("Connected to service " + service_name)

    def calibrate(self):
        """
        Calibration Function
        """
        self.node.get_logger().info("ezgripper_interface: calibrate")
        try:
            self.cli.call_async(Empty.Request())
        except Exception as exc:
            self.node.get_logger().info("Service did not process request: " + str(exc))
        else:
            self._grip_value = self._grip_max
        self.node.get_logger().info("ezgripper_interface: calibrate done")

    def feedback_callback(self, feedback_msg):
        """
        Callback for Feedback
        """
        feedback = feedback_msg.feedback
        self.node.get_logger().info('Received feedback: {}'.format(feedback))

    def open_step(self):
        """
        Step Opening the gripper
        """

        self._grip_value = self._grip_value + self._grip_step
        if self._grip_value > self._grip_max:
            self._grip_value = self._grip_max

        self.node.get_logger().info( \
            "ezgripper_interface: goto position {:.3f}".format(self._grip_value))

        goal = GripperCommand.Goal()
        goal.command.position = \
            remap(self._grip_value, 100.0, 0.0, OPEN_SINGLE_MOUNT_POS, CLOSE_SINGLE_MOUNT_POS)
        goal.command.max_effort = 0.5
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: goto position done")


    def close_step(self):
        """
        Step Closing the gripper
        """

        self._grip_value = self._grip_value - self._grip_step
        if self._grip_value < self._grip_min:
            self._grip_value = self._grip_min

        self.node.get_logger().info( \
            "ezgripper_interface: goto position {:.3f}".format(self._grip_value))

        goal = GripperCommand.Goal()
        goal.command.position = \
            remap(self._grip_value, 100.0, 0.0, OPEN_SINGLE_MOUNT_POS, CLOSE_SINGLE_MOUNT_POS)
        goal.command.max_effort = 0.5
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: goto position done")

    def close(self, max_effort):
        """
        Close the gripper
        """

        self.node.get_logger().info( \
            "ezgripper_interface: close, effort {:.3f}".format(max_effort))

        goal = GripperCommand.Goal()
        goal.command.position = \
            remap(0.0, 100.0, 0.0, OPEN_SINGLE_MOUNT_POS, CLOSE_SINGLE_MOUNT_POS)
        goal.command.max_effort = max_effort
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: close done")
        self._grip_value = self._grip_min

    def hard_close(self):
        """
        Hard Closing the gripper
        """
        self.node.get_logger().info("ezgripper_interface: hard close")

        goal = GripperCommand.Goal()
        goal.command.position = \
            remap(0.0, 100.0, 0.0, OPEN_SINGLE_MOUNT_POS, CLOSE_SINGLE_MOUNT_POS)
        goal.command.max_effort = 1.0
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: hard close done")
        self._grip_value = self._grip_min

    def soft_close(self):
        """
        Soft Closing the gripper
        """
        self.node.get_logger().info("ezgripper_interface: soft close")

        goal = GripperCommand.Goal()
        goal.command.position = \
            remap(0.0, 100.0, 0.0, OPEN_SINGLE_MOUNT_POS, CLOSE_SINGLE_MOUNT_POS)
        goal.command.max_effort = 0.2
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: soft close done")
        self._grip_value = self._grip_min

    def open(self):
        """
        Opening the gripper
        """
        self.node.get_logger().info("ezgripper_interface: open")

        goal = GripperCommand.Goal()
        goal.command.position = \
            remap(100.0, 100.0, 0.0, OPEN_SINGLE_MOUNT_POS, CLOSE_SINGLE_MOUNT_POS)
        goal.command.max_effort = 1.0
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: open done")
        self._grip_value = self._grip_max

    def goto_position(self, grip_position, grip_effort):
        """
        Go to specific position
        """
        self.node.get_logger().info( \
            "ezgripper_interface: goto position {:.3f}".format(grip_position))

        goal = GripperCommand.Goal()
        goal.command.position = \
            remap(grip_position, 100.0, 0.0, OPEN_SINGLE_MOUNT_POS, CLOSE_SINGLE_MOUNT_POS)
        goal.command.max_effort = grip_effort / 100.0
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: goto position done")
        self._grip_value = grip_position

    def release(self):
        """
        Release the gripper
        """
        self.node.get_logger().info("ezgripper_interface: release")
        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        goal.command.max_effort = 0.0 # releases all torque on motor
        self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self.node.get_logger().info("ezgripper_interface: release done")
        self._grip_value = self._grip_min



def main(args=None):
    """
    Main Function
    """

    rclpy.init(args=args)
    node = Node('ezgripper_interface_node')

    ez_obj = EZGripper(node, '/ezgripper_single_mount/ezgripper_controller/gripper_cmd')

    ez_obj.open()
    ez_obj.calibrate()
    ez_obj.open()
    ez_obj.hard_close()
    ez_obj.open()
    ez_obj.soft_close()
    ez_obj.open()

    node.get_logger().info('Exiting')

    rclpy.shutdown()


if __name__ == "__main__":
    main()
