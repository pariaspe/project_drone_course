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

This module implements a interface for controlling a drone using the
Aerostack2 framework. It handles arming, offboard mode, control mode
setting, and position commands.
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from geometry_msgs.msg import Point


class DummyDrone(DroneInterface):
    def __init__(self, drone_name: str, verbose=False, use_sim_time=True):
        super().__init__(drone_name, verbose, use_sim_time)

        # Class variables
        self.path: list[Point] = [Point(x=4.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=5.0, z=2.1),
                                  Point(x=4.0, y=5.0, z=2.1)]
        self.path_idx: int = 0
        self.sleep_time: float = 1.0

    def do_mission(self):
        """ Run the mission """
        ##### ARM OFFBOARD #####
        print("Offboard")
        self.offboard()
        sleep(self.sleep_time)
        print("Arm")
        self.arm()
        sleep(self.sleep_time)

        ##### TAKE OFF #####
        print("Take Off")
        self.takeoff(1.5, speed=1.0)
        print("Take Off done")
        sleep(self.sleep_time)

        # TODO (Exercise 4.1):
        # Modify mission behaviors to complete the race
        # A. Using go_to() method
        # B. Using go_to_path_facing() method
        # C. Using go_to_with_yaw() method
        # D. Using follow_path() method

        ##### GO TO #####
        print(f"Go to with path facing Point(x=4.0, y=0.0, z=1.5)")
        self.go_to.go_to(x=4.0, y=0.0, z=1.5, speed=1.0)
        print(f"Go to with path facing Point(x=0.0, y=0.0, z=1.5)")
        self.go_to.go_to(x=0.0, y=0.0, z=1.5, speed=1.0)
        print("Go to done")
        sleep(self.sleep_time)

        # TODO (Exercise 4.2):
        # Non-blocking behaviors:
        # A. Using go_to_with_yaw() method with wait=False
        # B. Modify its behavior goal (yaw only) while executing

        ##### LAND #####
        print("Landing")
        self.land(speed=0.5)
        print("Land done")

        self.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav = DummyDrone("drone0", verbose=False, use_sim_time=True)

    uav.do_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
