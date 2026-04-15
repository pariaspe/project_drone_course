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
setting, and position commands.
"""

__authors__ = 'Pedro Arias Pérez'

import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.modules.motion_reference_handler_module import MotionReferenceHandlerModule
from geometry_msgs.msg import PoseStamped, Point


class DummyDrone(DroneInterfaceBase):
    def __init__(self, drone_name: str, verbose=False, use_sim_time=True):
        super().__init__(drone_name, verbose, use_sim_time)

        self.motion_ref_handler = MotionReferenceHandlerModule(drone=self)
        self.send_pose = self.motion_ref_handler.position.send_position_command_with_yaw_angle

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

            # Generate motion reference command
            position_msg = PoseStamped()
            position_msg.header.stamp = self.get_clock().now().to_msg()
            position_msg.header.frame_id = 'earth'
            position_msg.pose.position.x = position_ref.x
            position_msg.pose.position.y = position_ref.y
            position_msg.pose.position.z = position_ref.z
            position_msg.pose.orientation.w = 1.0  # Neutral orientation
            position_msg.pose.orientation.x = 0.0
            position_msg.pose.orientation.y = 0.0
            position_msg.pose.orientation.z = 0.0

            # Read current drone state
            current_x = self.position[0]
            current_y = self.position[1]
            current_z = self.position[2]

            position_error_x = position_ref.x - current_x
            position_error_y = position_ref.y - current_y
            position_error_z = position_ref.z - current_z
            position_error = (position_error_x**2 + position_error_y **
                              2 + position_error_z**2) ** 0.5

            # Publish motion reference command
            # TODO(Exercise 1): Send the motion reference command using self.send_pose method

            if position_error < 0.2:
                self.get_logger().info(
                    'Drone has reached the target position', throttle_duration_sec=1.0
                )
                # TODO(Exercise 1): Update self.path_idx to move to the next position in PATH list
                self.path_idx = 1


if __name__ == '__main__':
    rclpy.init()

    uav = DummyDrone("drone0", verbose=False, use_sim_time=True)

    uav.do_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
