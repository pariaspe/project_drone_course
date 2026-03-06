#!/usr/bin/env python3

# Copyright 2026 Universidad Politécnica de Madrid
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
"""Drone Controller Node.

This module implements a ROS2 node for controlling a drone using the
Aerostack2 framework. It handles arming, offboard mode, control mode
setting, and position commands.
"""

from as2_msgs.msg import ControlMode
from as2_msgs.srv import SetControlMode
from drone_course_msgs.msg import Point
from drone_course_msgs.srv import RequestPath
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class DroneController(Node):
    """Drone Controller class for managing drone operations.

    This class handles subscribing to drone state information,
    publishing motion commands, and calling services to control the
    drone's operational state.
    """

    def __init__(self):
        """Initialize the DroneController node.

        Sets up subscribers, publishers, service clients, and the main
        control timer.
        """
        super().__init__('drone_controller')
        # QoS profile for reliable topics
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        # QoS profile for best effort topics
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        
        # Callback group for handling synchronous service calls from timer
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        # TODO(Exercise 1): Subcribe to localization pose
        # Subscribe to topic: /drone0/self_localization/pose
        # Message type: geometry_msgs/msg/PoseStamped (PoseStamped)
        # Callback: self.state_pose_callback
        # QoS: best effort
        self.pose_sub = None

        # Publishers
        # TODO(Exercise 1): Publish motion reference command
        # Publish to topic: /drone0/motion_reference/pose
        # Message type: geometry_msgs/msg/PoseStamped (PoseStamped)
        # QoS: reliable
        self.pose_pub = None

        # Class variables
        self.state_pose: PoseStamped = PoseStamped()
        self.control_mode_set: bool = False
        self.path: list[Point] = [Point(x=4.0, y=0.0, z=2.1), 
                                  Point(x=11.0, y=0.0, z=2.1), 
                                  Point(x=11.0, y=5.0, z=2.1), 
                                  Point(x=4.0, y=5.0, z=2.1)]
        self.path_idx: int = 0

        # Timer at 10Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.run, callback_group=self.callback_group)

        self.get_logger().info('Drone Controller Node initialized')

    def state_pose_callback(self, msg):
        """Callback for state pose messages.

        :param msg: The received pose message
        :type msg: PoseStamped
        """
        self.state_pose = msg

    def run(self):
        """Main control loop."""

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
        current_x = self.state_pose.pose.position.x
        current_y = self.state_pose.pose.position.y
        current_z = self.state_pose.pose.position.z

        position_error_x = position_ref.x - current_x
        position_error_y = position_ref.y - current_y
        position_error_z = position_ref.z - current_z
        position_error = (position_error_x**2 + position_error_y**2 + position_error_z**2) ** 0.5

        # Publish motion reference command
        # TODO(Exercise 1): Publish the generated motion reference command to /drone0/motion_reference/pose

        if position_error < 0.2:
            self.get_logger().info(
                'Drone has reached the target position', throttle_duration_sec=1.0
            )
            # TODO(Exercise 1): Update self.path_idx to move to the next position in the PATH list
            self.path_idx = 1


def main(args=None):
    """Main entry point for the drone controller node.

    :param args: Command line arguments
    :type args: list
    """
    rclpy.init(args=args)
    drone_controller = DroneController()

    # Use MultiThreadedExecutor to allow synchronous service calls from timer
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    executor.add_node(drone_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
