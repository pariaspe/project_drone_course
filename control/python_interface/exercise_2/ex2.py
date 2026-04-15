#!/usr/bin/env python3

# Copyright 2026
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Drone Controller Class.

This module implements an interface for controlling a drone using the
Aerostack2 framework. It handles arming, offboard mode, control mode
setting, and speed commands.
"""

__authors__ = 'Pedro Arias Pérez'

import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.modules.motion_reference_handler_module import MotionReferenceHandlerModule
from geometry_msgs.msg import TwistStamped, Point


class DummyDrone(DroneInterfaceBase):
    def __init__(self, drone_name: str, verbose=False, use_sim_time=True):
        super().__init__(drone_name, verbose, use_sim_time)

        self.motion_ref_handler = MotionReferenceHandlerModule(drone=self)
        self.send_vel = self.motion_ref_handler.speed.send_speed_command_with_yaw_speed

        # Class variables
        self.path: list[Point] = [Point(x=4.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=5.0, z=2.1),
                                  Point(x=4.0, y=5.0, z=2.1)]
        self.path_idx: int = 0

    def do_mission(self):
        """ Run the mission """

        while True:
            # Desired position reference
            position_ref = self.path[self.path_idx]
            position_ref = (position_ref.x, position_ref.y, position_ref.z)

            # Read current drone state
            current_x = self.position[0]
            current_y = self.position[1]
            current_z = self.position[2]

            position_error_x = position_ref[0] - current_x
            position_error_y = position_ref[1] - current_y
            position_error_z = position_ref[2] - current_z
            position_error = (position_error_x**2 + position_error_y **
                              2 + position_error_z**2) ** 0.5

            # TODO(Exercise 2):
            # Calculate and send velocity commands using self.send_vel method

            velocity_command_x = 0.0
            velocity_command_y = 0.0
            velocity_command_z = 0.0

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'earth'
            twist_msg.twist.linear.x = velocity_command_x
            twist_msg.twist.linear.y = velocity_command_y
            twist_msg.twist.linear.z = velocity_command_z
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            self.send_vel(twist_msg)

            if position_error < 0.2:
                self.get_logger().info(
                    'Drone has reached the target position', throttle_duration_sec=1.0
                )
                self.path_idx = (self.path_idx + 1) % len(self.path)


if __name__ == '__main__':
    rclpy.init()

    uav = DummyDrone("drone0", verbose=False, use_sim_time=True)

    uav.do_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
