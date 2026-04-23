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

import numpy as np
from math import cos, pi
from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterfaceBase
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix
from rclpy.qos import qos_profile_sensor_data


class DummyDrone(DroneInterfaceBase):
    def __init__(self, drone_name: str, verbose=False, use_sim_time=False):
        super().__init__(drone_name, verbose, use_sim_time)

        self.imu_sub = self.create_subscription(
            Imu, 'sensor_measurements/imu',
            self.__imu_callback, qos_profile_sensor_data)

        self.navsatfix_sub = self.create_subscription(
            NavSatFix, 'sensor_measurements/gps',
            self.__navsatfix_callback, qos_profile_sensor_data)

        self.pose_with_cov_sub = self.create_publisher(
            PoseWithCovarianceStamped, 'self_localization/pose_with_covariance', 10)

        # Class variables
        self.path: list[Point] = [Point(x=4.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=5.0, z=2.1),
                                  Point(x=4.0, y=5.0, z=2.1)]
        self.path_idx: int = 0
        self.sleep_time: float = 1.0

        # IMU noise parameters
        self.accel_noise_variance = 0.005  # m^2/s^4 (tune this based on expected IMU noise)
        self.gyro_noise_variance = 0.03  # rad^2/s^4 (tune this based on expected gyro noise)

        # GPS origin
        self.gps_origin_lat = 40.4405
        self.gps_origin_lon = -3.68982
        self.gps_origin_alt = 100.0

    def __imu_callback(self, msg: Imu):
        """ IMU callback function """
        # Get the current IMU data
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not hasattr(self, "_last_imu_time"):
            self._last_imu_time = t
            self._pos = [0.0, 0.0, 0.0]
            self._vel = [0.0, 0.0, 0.0]
            self._orientation = orientation
            self._pos_var = [0.0, 0.0, 0.0]
            self._ori_var = [0.0, 0.0, 0.0]
            return

        dt = t - self._last_imu_time
        if dt <= 0.0 or dt > 0.5:
            self._last_imu_time = t
            return

        self._last_imu_time = t

        # Accumulate covariance over time (random walk model)
        dt2 = dt * dt
        dt4 = dt2 * dt2
        for i in range(3):
            self._pos_var[i] += self.accel_noise_variance * dt4 / 4.0
            self._ori_var[i] += self.gyro_noise_variance * dt2

        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # Normalize quaternion
        norm = (qx**2 + qy**2 + qz**2 + qw**2) ** 0.5
        if norm < 1e-6:
            return
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        ax_b = linear_acceleration.x
        ay_b = linear_acceleration.y
        az_b = linear_acceleration.z

        def rotate_body_to_world(vx, vy, vz):
            tx = 2.0 * (qy * vz - qz * vy)
            ty = 2.0 * (qz * vx - qx * vz)
            tz = 2.0 * (qx * vy - qy * vx)
            vpx = vx + qw * tx + (qy * tz - qz * ty)
            vpy = vy + qw * ty + (qz * tx - qx * tz)
            vpz = vz + qw * tz + (qx * ty - qy * tx)
            return vpx, vpy, vpz

        ax_w, ay_w, az_w = rotate_body_to_world(ax_b, ay_b, az_b)
        az_w -= 9.81

        # TODO (Exercise 4.1): Prediction step using IMU data.
        # This may involve integrating the IMU measurements over time
        # to update the drone's position and orientation.
        # Use Verlet-like or Euler integration.

        self._pos = None
        self._vel = None
        self._orientation = orientation
        self._angular_velocity = angular_velocity

    def __navsatfix_callback(self, msg: NavSatFix):
        """ NavSatFix callback function """
        # Get the current GPS data
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude

        # TODO (Exercise 4.2): Measurement update step using the GPS data.
        # This may involve converting the GPS coordinates to a local frame
        # and updating the drone's position estimate.
        # Use flat Earth approximation: https://es.mathworks.com/help/aeroblks/llatoflatearth.html
        R = 6378137.0  # Earth radius in meters

        x = longitude
        y = latitude
        self._gps_measurement = None
        self._gps_measurement_var = 0.000001

    def kalman_like_fusion(self, gps_pos, gps_pos_var):
        """ Kalman-like fusion of IMU and GPS data """
        if not hasattr(self, "_pos"):
            self._pos = [gps_pos[0], gps_pos[1], gps_pos[2]]
            self._vel = [0.0, 0.0, 0.0]
            self._pos_var = [gps_pos_var, gps_pos_var, gps_pos_var]
            self._ori_var = [0.0, 0.0, 0.0]
            return

        # TODO (Exercise 4.3): Correction step.
        # Kalman-like fusion of the GPS measurement with the current state estimate.
        pass

    def publish_estimation(self):
        """ Publish the current state estimation """
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "earth"
        if not hasattr(self, "_pos") or self._pos is None:
            self.pose_with_cov_sub.publish(pose)
            return

        if hasattr(self, "_gps_measurement"):
            self.kalman_like_fusion(self._gps_measurement, self._gps_measurement_var)

        pose.pose.pose.position.x = self._pos[0]
        pose.pose.pose.position.y = self._pos[1]
        pose.pose.pose.position.z = self._pos[2]
        pose.pose.pose.orientation = self._orientation
        cov = [0.0] * 36
        cov[0] = self._pos_var[0]  # x
        cov[7] = self._pos_var[1]  # y
        cov[14] = self._pos_var[2]  # z
        cov[21] = self._ori_var[0]  # roll
        cov[28] = self._ori_var[1]  # pitch
        cov[35] = self._ori_var[2]  # yaw
        pose.pose.covariance = cov
        self.pose_with_cov_sub.publish(pose)

    def do_mission(self):
        """ Run the mission """

        while True:
            sleep(0.05)
            self.publish_estimation()


if __name__ == '__main__':
    rclpy.init()

    uav = DummyDrone("drone0", verbose=False, use_sim_time=True)

    uav.do_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
