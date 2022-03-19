#!/usr/bin/python3
"""
EZGripper Action Server Module
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
#

from functools import partial
from math import fabs
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_srvs.srv import Empty
from control_msgs.action import GripperCommand
from libezgripper import create_connection, Gripper

class GripperAction(Node):
    """
    GripperCommand Action Server to send Feedback to MoveIt! and
        to recieve goals from MoveIt!
    """

    _feedback = GripperCommand.Feedback()
    _result = GripperCommand.Result()

    def __init__(self):
        super().__init__('ezgripper_action_server_node')

        self._timeout = 3.0
        self._positional_buffer = 0.05

        self.declare_parameter('port')
        self.declare_parameter('baudrate')
        self.declare_parameter('no_of_grippers')

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.no_of_grippers = self.get_parameter('no_of_grippers').value

        self.grippers = {}
        # connection = \
        #     create_connection(dev_name=self.port, baudrate=self.baudrate)

        for i in range(1, int(self.no_of_grippers) + 1):

            self.declare_parameter('gripper_%s.action_name' % str(i))
            self.declare_parameter('gripper_%s.servo_ids' % str(i))
            self.declare_parameter('gripper_%s.module_type' % str(i))

            action_name = self.get_parameter('gripper_%s.action_name' % str(i)).value
            servo_ids = self.get_parameter('gripper_%s.servo_ids' % str(i)).value
            module_type = self.get_parameter('gripper_%s.module_type' % str(i)).value

            # self.grippers[action_name] = Gripper(connection, action_name, servo_ids)
            self.create_service(Empty, '~/'+ action_name + '/calibrate', \
                partial(self.calibrate_srv, action_name))

            ActionServer( \
                self, \
                GripperCommand, \
                '~/' + action_name, \
                goal_callback = self._goal_callback,
                cancel_callback = self._cancel_callback,
                execute_callback = partial(self._execute_callback, action_name, module_type)
            )

        self.get_logger().info('Gripper server ready')

    def calibrate_srv(self, action_name, request, response):
        """
        Calibration Service
        """
        self.get_logger().info("Calibrate service: request received")
        self.grippers[action_name].calibrate()
        self.grippers[action_name].open()
        self.get_logger().info("Calibrate service: request completed")
        return response

    def get_time(self):
        """
        Get current time in seconds
        """
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)

    def now_from_start(self, start):
        """
        Get time difference from start time
        """
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9) - start

    def _goal_callback(self, goal_request):
        """
        Goal callback for action server
        """
        self.get_logger().debug('Gripper received goal request')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """
        Cancel callback for action server
        """
        self.get_logger().debug('Gripper received cancel request')
        return CancelResponse.ACCEPT

    def _command_gripper(self, action_name, module_type, position, effort):
        """
        Actuate gripper to position and effort
        """
        # Debug string
        self.get_logger().info("Execute goal: position=%.1f, max_effort=%.1f"%
                      (position, effort))

        # Actuate Gripper
        if effort == 0.0:
            self.get_logger().info("Release torque: start")
            self.grippers[action_name].release()
            self.get_logger().info("Release torque: done")
        else:
            self.get_logger().info("Go to position: start")
            self.grippers[action_name].goto_position(position, effort, \
                use_percentages = False, gripper_module = module_type)
            self.get_logger().info("Go to position: done")

    def _check_state(self, action_name, module_type, position):
        """
        Check if gripper has reached desired position
        """
        return fabs(self.grippers[action_name].get_position( \
            use_percentages = False, \
                gripper_module = module_type) - position) < self._positional_buffer

    def _publish_feedback_and_update_result(self, action_name, module_type, position, effort):
        """
        Publish Gripper Feedback and Update Result
        """
        self._feedback.position = self.grippers[action_name].get_position( \
            use_percentages = False, gripper_module = module_type)
        self._feedback.effort = effort
        self._feedback.reached_goal = self._check_state(action_name, module_type, position)
        self._result = self._feedback
        return self._feedback

    def _execute_callback(self, action_name, module_type, goal_handle):
        """
        Execute callback for action server
        """

        self.get_logger().debug('Gripper executing goal...')

        position = goal_handle.request.command.position
        effort = goal_handle.request.command.max_effort

        rate = self.create_rate(100)
        start_time = self.get_time()

        # Iterate until goal is reached or timeout
        while rclpy.ok() \
            and self.now_from_start(start_time) < self._timeout:

            # Publish Feedback and Update Result
            goal_handle.publish_feedback( \
                self._publish_feedback_and_update_result( \
                    action_name, module_type, position, effort))

            # Command gripper
            self._command_gripper(action_name, module_type, position, effort)

            # Check if goal is reached
            if self._check_state(action_name, module_type, position):
                goal_handle.succeed()
                self.get_logger().info("Gripper has reached desired position")
                return

            rate.sleep()

        self.get_logger().info("Gripper has grasped an object")


def main(args=None):
    """
    Main Function
    """
    rclpy.init(args=args)

    gripper_action = GripperAction()
    rclpy.spin(gripper_action)

    gripper_action.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
