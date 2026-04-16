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

import os
from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterfaceBase, DroneInterface
from as2_python_api.modules.module_base import ModuleBase
from geometry_msgs.msg import Point


class GateRaceModule(ModuleBase):
    """Gate Race Module
    """
    __alias__ = "gate_race"
    __deps__ = ['takeoff', 'go_to', 'follow_path', 'land']

    def __init__(self, drone: 'DroneInterfaceBase') -> None:
        super().__init__(drone, self.__alias__)
        self.namespace = drone.drone_id

        self.takeoff_behavior = getattr(drone, 'takeoff')
        self.takeoff = getattr(self.takeoff_behavior, '__call__')
        self.go_to_behavior = getattr(drone, 'go_to')
        self.go_to = getattr(self.go_to_behavior, '__call__')
        self.follow_path_behavior = getattr(drone, 'follow_path')
        self.follow_path = getattr(self.follow_path_behavior, '__call__')
        self.land_behavior = getattr(drone, 'land')
        self.land = getattr(self.land_behavior, '__call__')

        self.__feedback = {"gate_number": None}
        self.__result = None

    @property
    def feedback(self):
        """Behavior feedback

        :return: rclpy.Feedback
        """
        return self.__feedback

    def __call__(self, gate_info_path: str) -> bool:
        return self.__gate_race(gate_info_path)

    def __gate_race(self, gate_info_path: str) -> bool:
        if not os.path.exists(gate_info_path):  # check if file exists
            print(f"Gate info file not found: {gate_info_path}")
            return False

        # TODO (Exercise 5.1):
        # Do behavior composition to complete the race
        # A. Parse yaml to get gate poses
        # B. Call behaviors in sequence
        # C. Publish feedback with current gate number while executing
        # D. Set result to True if race completed successfully, False otherwise

        self.__feedback["gate_number"] = 0
        print(f"Feedback: {self.feedback}")
        sleep(3)  # wait for feedback to be read

        return True

    def destroy(self) -> None:
        """Destroy module, clean exit."""


class DummyDrone(DroneInterface):
    def __init__(self, drone_name: str, verbose=False, use_sim_time=True):
        super().__init__(drone_name, verbose, use_sim_time)

        self.gate_race = GateRaceModule(drone=self)

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

        print(f"Current working directory: {os.getcwd()}")
        self.gate_race("config/gates_config.yaml")

        self.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav = DummyDrone("drone0", verbose=False, use_sim_time=True)

    uav.do_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
