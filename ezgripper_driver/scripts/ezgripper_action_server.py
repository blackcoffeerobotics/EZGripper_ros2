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

import time
from functools import partial
from math import fabs
import rclpy
from rclpy.qos import QoSProfile, \
    QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from std_srvs.srv import Empty
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from libezgripper import create_connection, Gripper


qos_unlatched = QoSProfile( \
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, \
    depth=1, \
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE, \
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
)

class GripperAction(Node):
    """
    GripperCommand Action Server
    """

    def __init__(self):
        super().__init__('ezgripper_controller')

        self.update_rate = 50
        self.time_period = 1./self.update_rate

        self._timeout = 3.0
        self._positional_buffer = 0.1
        self.all_servos = []

        self.declare_parameter('port')
        self.declare_parameter('baudrate')
        self.declare_parameter('no_of_grippers')

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.no_of_grippers = self.get_parameter('no_of_grippers').value

        self._feedback = {}
        self._result = {}
        self.grippers = {}
        self.joint_state_pub = {}

        connection = create_connection(dev_name=self.port, baudrate=self.baudrate)

        for i in range(1, int(self.no_of_grippers) + 1):

            self.declare_parameter('gripper_{}.action_name'.format(i))
            self.declare_parameter('gripper_{}.servo_ids'.format(i))
            self.declare_parameter('gripper_{}.module_type'.format(i))
            self.declare_parameter('gripper_{}.robot_ns'.format(i))

            action_name = self.get_parameter('gripper_{}.action_name'.format(i)).value
            servo_ids = self.get_parameter('gripper_{}.servo_ids'.format(i)).value
            module_type = self.get_parameter('gripper_{}.module_type'.format(i)).value
            robot_ns = self.get_parameter('gripper_{}.robot_ns'.format(i)).value

            self._feedback[action_name] = GripperCommand.Feedback()
            self._result[action_name] = GripperCommand.Result()

            self.grippers[action_name] = Gripper(connection, action_name, servo_ids)
            self.all_servos += self.grippers[action_name].servos

            self.grippers[action_name].calibrate()
            self.grippers[action_name].open()

            self.joint_state_pub[action_name] = \
                self.create_publisher(JointState, '/{}/joint_states'.format(robot_ns), \
                    qos_unlatched)

            self.create_service(Empty, \
                robot_ns + '/ezgripper_controller/'+ action_name + '/calibrate', \
                    partial(self.calibrate_srv, action_name))

            ActionServer( \
                self, \
                GripperCommand, \
                robot_ns + '/ezgripper_controller/'+ action_name, \
                goal_callback = self._goal_callback,
                cancel_callback = self._cancel_callback,
                execute_callback = partial(self._execute_callback, action_name, module_type)
            )

        # Publishers
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", qos_unlatched)

        # Timers
        self.create_timer(self.time_period, self.joint_state_update)
        self.create_timer(1.0, self.diagnostics_and_servo_update)

        self.get_logger().info('Gripper server ready')


    def joint_state_update(self):
        """
        Publish joint state
        """

        for i in range(1, int(self.no_of_grippers) + 1):

            action_name = self.get_parameter('gripper_{}.action_name'.format(i)).value
            module_type = self.get_parameter('gripper_{}.module_type'.format(i)).value

            current_gripper_position = self.grippers[action_name].get_position( \
                use_percentages = False, gripper_module=module_type)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [current_gripper_position]

            if module_type == 'dual_gen1' or module_type == 'quad':
                finger_joint = 'left_ezgripper_knuckle_1'

            elif module_type == 'dual_gen2' or module_type == 'dual_gen2_single_mount':
                finger_joint = 'left_ezgripper_knuckle_palm_L1_1'

            elif module_type == 'dual_gen2_triple_mount':
                finger_joint = 'left1_ezgripper_knuckle_palm_L1_1'

            msg.name = [finger_joint]

            self.joint_state_pub[action_name].publish(msg)


    def diagnostics_and_servo_update(self):
        """
        Send Diagnostic Data and monitor servos
        """

        msg = DiagnosticArray()
        msg.status = []
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(1, int(self.no_of_grippers) + 1):
            action_name = self.get_parameter('gripper_{}.action_name'.format(i)).value

            gripper = self.grippers[action_name]

            temp_msg = KeyValue()
            temp_msg.key = 'Temperature'

            volt_msg = KeyValue()
            volt_msg.key = 'Voltage'

            for servo in gripper.servos:
                status = DiagnosticStatus()
                status.name = "Gripper - {}, Servo - {}".format(gripper.name, servo.servo_id)
                status.hardware_id = '{}'.format(servo.servo_id)
                temperature = servo.read_temperature()

                temp_msg.value = str(temperature)
                volt_msg.value = str(servo.read_voltage())

                status.values.append(temp_msg)
                status.values.append(volt_msg)

                if temperature >= 70:
                    status.level = DiagnosticStatus.ERROR
                    status.message = 'OVERHEATING'
                elif temperature >= 65:
                    status.level = DiagnosticStatus.WARN
                    status.message = 'HOT'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = 'OK'

                msg.status.append(status)

        self.diagnostics_pub.publish(msg)

        for servo in self.all_servos:
            try:
                servo.check_overload_and_recover()
            except Exception as error:
                self.get_logger().info('Exception while checking overload')
                servo.flushAll()


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

    def now_from_start(self, start_time):
        """
        start_time is in seconds
        Get time difference from start time
        """
        return self.get_time() - start_time

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

    def _publish_feedback(self, action_name, module_type, position, effort):
        """
        Publish Gripper Feedback
        """
        self._feedback[action_name].position = self.grippers[action_name].get_position( \
            use_percentages = False, gripper_module = module_type)
        self._feedback[action_name].effort = effort
        self._feedback[action_name].reached_goal = \
            self._check_state(action_name, module_type, position)
        return self._feedback[action_name]

    def _execute_callback(self, action_name, module_type, goal_handle):
        """
        Execute callback for action server
        """

        self.get_logger().debug('Gripper executing goal...')

        position = goal_handle.request.command.position
        effort = goal_handle.request.command.max_effort

        start_time = self.get_time()

        # Iterate until goal is reached or timeout
        while self.now_from_start(start_time) < self._timeout:

            # Publish Feedback
            goal_handle.publish_feedback( \
                self._publish_feedback( \
                    action_name, module_type, position, effort))

            # Command gripper
            self._command_gripper(action_name, module_type, position, effort)

            # Check if goal is reached
            if self._feedback[action_name].reached_goal:
                goal_handle.succeed()
                break

            time.sleep(0.01)

        if not self._feedback[action_name].reached_goal:
            self.get_logger().info("Gripper has grasped an object")
            goal_handle.succeed()

        else:
            self.get_logger().info("Gripper has reached desired position")

        self._result[action_name].reached_goal = self._feedback[action_name].reached_goal
        self._result[action_name].position = self._feedback[action_name].position
        self._result[action_name].effort = self._feedback[action_name].effort

        return self._result[action_name]



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
