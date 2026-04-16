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

import time
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_python_api.modules.motion_reference_handler_module import MotionReferenceHandlerModule
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from trajectory_generator import TrajectoryGenerator
from types import SimpleNamespace
import numpy as np


class DummyDrone(DroneInterfaceBase):
    def __init__(self, drone_name: str, verbose=False, use_sim_time=True):
        super().__init__(drone_name, verbose, use_sim_time)

        self.motion_ref_handler = MotionReferenceHandlerModule(drone=self)
        self.send_pose = self.motion_ref_handler.position.send_position_command_with_yaw_angle
        self.send_vel = self.motion_ref_handler.speed.send_speed_command_with_yaw_speed

        # Class variables
        self.path: list[Point] = [Point(x=4.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=0.0, z=2.1),
                                  Point(x=11.0, y=5.0, z=2.1),
                                  Point(x=4.0, y=5.0, z=2.1)]
        self.path_idx: int = 0

        self.pub_traj_markers = self.create_publisher(
            MarkerArray, 'trajectory_markers', 10)

    def generate_trajectory_example(self):
        """
        Example to illustrate how to compute a trajectory
        """

        # TODO (Exercise 3.1):
        # Modify trajectory waypoints to complete the race
        traj_gen = TrajectoryGenerator()
        traj_gen.add_waypoint([0, 0, 0], 0)  # Add waypoint [0,0,0] at time t=0
        traj_gen.add_waypoint([0, 0, 1.5], 1)  # Add waypoint [0,0,0] at time t=1
        traj_gen.add_waypoint([0, 2, 2], 3)  # Add waypoint [0,0,0] at time t=3
        traj_gen.add_waypoint([2, 2, 1.5], 4)  # Add waypoint [0,0,0] at time t=4
        traj_gen.add_waypoint([2, 0, 1], 5)  # Add waypoint [0,0,0] at time t=5
        traj_gen.add_waypoint([0, 0, 1], 8)  # Add waypoint [0,0,0] at time t=8

        # Constraints
        traj_gen.add_waypoint_constraint(derivative_order=1, wpt_index=0, value=[
                                         0.0, 0.0, 0.0], side="right")   # v(t0)
        traj_gen.add_waypoint_constraint(derivative_order=1, wpt_index=len(
            traj_gen.times) - 1, value=[0, 0, 0], side="left")   # v(tf)
        traj_gen.add_waypoint_constraint(derivative_order=2, wpt_index=len(
            traj_gen.times) - 1, value=[0, 0, 0], side="left")   # a(tf)

        # Compute trajectory
        # opts = SimpleNamespace(trajectory_type="piecewise_polynomial", order=3)
        opts = SimpleNamespace(trajectory_type="minimum_snap", order=7,
                               continuity_order=3, reg=1e-9, normalized=True)
        traj_gen.generate_trajectory(opts)
        self.get_logger().info("Trajectory successfully generated.")

        # times, positions, velocities = self.generate_trajectory_example()
        self.draw_trajectory_markers(traj_gen)

        return traj_gen

    def do_mission(self):
        """ Run the mission """

        traj = self.generate_trajectory_example()
        t0 = time.time()
        while time.time() - t0 < 20.0:
            # TODO (Exercise 3.2):
            # Evaluate the trajectory to obtain references to follow a smoothly
            # A. Using position commands (self.send_pose)
            # B. Using velocity commands (self.send_vel)
            pass

    def draw_trajectory_markers(
        self,
        trajectory: TrajectoryGenerator,
        deltat: float = 0.1,
        vel_scale: float = 0.3,
        shaft_diam: float = 0.03,
        head_diam: float = 0.06,
        head_len: float = 0.08,
        max_arrow_len: float | None = None,
    ):
        """
        Visualize the spline trajectory in RViz using a MarkerArray:
        - LINE_STRIP for the full path,
        - SPHERE_LIST for sampled points,
        - ARROW markers for velocity vectors (direction = velocity, length ∝ |v|).
        The arrow length is |v| * vel_scale (optionally clamped by max_arrow_len).
        """
        deltat = 0.05
        t0 = float(trajectory.times[0])
        tf = float(trajectory.times[-1])
        n_steps = int(np.floor((tf - t0) / deltat)) + 1
        ts = t0 + np.arange(n_steps, dtype=float) * deltat
        if ts[-1] < tf - 1e-9:
            ts = np.append(ts, tf)

        pts, vels = [], []
        for t in ts:
            pos = trajectory.evaluate_trajectory(t, derivative_order=0)
            vel = trajectory.evaluate_trajectory(t, derivative_order=1)
            pts.append(Point(x=pos[0], y=pos[1], z=pos[2]))
            vels.append(tuple(vel))

        # 1) Trajectory path (line)
        line_marker = Marker()
        line_marker.header.frame_id = "earth"
        # line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.header.stamp.sec = 0
        line_marker.header.stamp.nanosec = 0
        line_marker.ns = "trajectory_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.03  # line thickness
        line_marker.color = ColorRGBA(r=0.0, g=0.6, b=1.0, a=1.0)
        line_marker.lifetime = Duration(sec=0)
        line_marker.points = pts

        # 2) Sampled points (spheres)
        dots_marker = Marker()
        dots_marker.header.frame_id = "earth"
        # dots_marker.header.stamp = self.get_clock().now().to_msg()
        dots_marker.header.stamp.sec = 0
        dots_marker.header.stamp.nanosec = 0
        dots_marker.ns = "trajectory_points"
        dots_marker.id = 1
        dots_marker.type = Marker.SPHERE_LIST
        dots_marker.action = Marker.ADD
        dots_marker.pose.orientation.w = 1.0
        dots_marker.scale.x = 0.06
        dots_marker.scale.y = 0.06
        dots_marker.scale.z = 0.06
        dots_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        dots_marker.lifetime = Duration(sec=0)
        dots_marker.points = pts

        # 3) Velocity arrows
        # Use ARROW with two points per marker: start (p) and end (p + dir * length).
        # When using 'points', the arrow length is the distance between the two points.
        arrow_markers = []
        for i, (p, v) in enumerate(zip(pts, vels)):
            vx, vy, vz = v
            speed = float(np.sqrt(vx * vx + vy * vy + vz * vz))

            # Skip zero-speed (or near-zero) to avoid degenerate arrows
            if speed < 1e-6:
                continue

            # Compute arrow length from speed and optional clamp
            length = speed * float(vel_scale)
            if max_arrow_len is not None:
                length = min(length, float(max_arrow_len))

            # Direction = normalized velocity
            inv = 1.0 / speed
            dx, dy, dz = vx * inv * length, vy * inv * length, vz * inv * length

            end = Point(x=p.x + dx, y=p.y + dy, z=p.z + dz)

            arrow = Marker()
            arrow.header.frame_id = "earth"
            # arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.header.stamp.sec = 0
            arrow.header.stamp.nanosec = 0
            arrow.ns = "velocity_arrows"
            arrow.id = 100 + i  # avoid collision with other IDs
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0  # ignored when using 'points' for ARROW

            # Arrow defined by two points: start and end
            arrow.points = [p, end]

            # Geometry: when using points, scale.x = shaft diameter, scale.y = head diameter, scale.z = head length
            arrow.scale.x = float(shaft_diam)
            arrow.scale.y = float(head_diam)
            arrow.scale.z = float(head_len)

            # Color (magenta); adjust alpha if you want translucency
            arrow.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)

            arrow.lifetime = Duration(sec=0)
            arrow_markers.append(arrow)

        # Publish everything together
        marray = MarkerArray()
        marray.markers.append(line_marker)
        marray.markers.append(dots_marker)
        marray.markers.extend(arrow_markers)
        self.pub_traj_markers.publish(marray)


if __name__ == '__main__':
    rclpy.init()

    uav = DummyDrone("drone0", verbose=False, use_sim_time=True)

    uav.do_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
