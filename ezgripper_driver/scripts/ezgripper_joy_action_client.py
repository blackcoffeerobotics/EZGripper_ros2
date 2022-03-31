#!/usr/bin/python3
"""
EZGripper Joy Action Client Module
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
from rclpy.qos import QoSProfile, \
    QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ezgripper_libs.ezgripper_interface import EZGripper


qos_unlatched = QoSProfile( \
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, \
    depth=1, \
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE, \
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
)


class EZGripperJoy():
    """
    EZGripper Joy Action Client
    """

    def __init__(self, node, module_types, gripper_names):

        self.node = node

        self.ezgripper_left = EZGripper(node, module_types[0], gripper_names[0])
        if len(gripper_names) > 1:
            self.ezgripper_right = EZGripper(node, module_types[1], gripper_names[1])
        else:
            self.ezgripper_right = None
        self.last_command_end_time = self.get_time()

    def get_time(self):
        """
        Get current time in seconds
        """
        time_msg = self.node.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)

    def now_from_start(self, start_time):
        """
        start_time is in seconds
        Get time difference from start time
        """
        return self.get_time() - start_time

    def joy_callback(self, joy):
        """
        Joystick Callback
        """

        if not joy.buttons:
            return # Don't break on an empty list

        if joy.buttons[5] == 1 and self.ezgripper_right is not None: # RB
            gripper = self.ezgripper_right
        else:
            gripper = self.ezgripper_left

        if self.now_from_start(self.last_command_end_time) > 0.2:
            # This check should flush all messages accumulated during command execution
            # and avoid executing it again.

            if joy.buttons[0] == 1: # A - hard close
                gripper.hard_close()
                self.last_command_end_time = self.get_time()

            if joy.buttons[3] == 1: # Y - soft close
                gripper.soft_close()
                self.last_command_end_time = self.get_time()

            if joy.buttons[1] == 1: # B - open
                gripper.open()
                self.last_command_end_time = self.get_time()

            if joy.buttons[2] == 1: # X - release
                gripper.release()
                self.last_command_end_time = self.get_time()

            if joy.buttons[6] == 1: # BACK - Calibrate
                gripper.calibrate()
                self.last_command_end_time = self.get_time()

            if joy.buttons[13] == 1: # xpad driver mapping
            #if joy.axes[7] == 1.0: # xboxdrv mapping
                gripper.open_step()
                self.last_command_end_time = self.get_time()

            if joy.buttons[14] == 1:
            #if joy.axes[7] == -1.0:
                gripper.close_step()
                self.last_command_end_time = self.get_time()

def main(args=None):
    """
    Main Function
    """
    rclpy.init(args=args)
    node = Node('ezgripper_joy_action_client_node')

    node.declare_parameter("no_of_grippers")

    no_of_grippers = node.get_parameter("no_of_grippers").value

    gripper_names = []
    module_types = []

    for i in range(1, int(no_of_grippers) + 1):

        node.declare_parameter('gripper_{}.action_name'.format(i))
        node.declare_parameter('gripper_{}.module_type'.format(i))
        node.declare_parameter('gripper_{}.robot_ns'.format(i))

        action_name = node.get_parameter( \
            'gripper_{}.action_name'.format(i)).value
        module_type = node.get_parameter( \
            'gripper_{}.module_type'.format(i)).value
        robot_ns = node.get_parameter( \
            'gripper_{}.robot_ns'.format(i)).value

        module_types.append(module_type)
        gripper_names.append(robot_ns + '/ezgripper_controller/'+ action_name)


    ezgripper_joy = EZGripperJoy(node, module_types, gripper_names)

    node.create_subscription(Joy, '/joy', ezgripper_joy.joy_callback, qos_unlatched)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
