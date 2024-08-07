#!/usr/bin/env python3

# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Andrea Capodacqua

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPosePublisher(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # Goal Position
        msg.pose.position.x = 1.5
        msg.pose.position.y = 1.5
        msg.pose.position.z = 0.0
        # Goal orientation (Quaternion)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)
        self.get_logger().info('Publishing goal pose: [x: %.2f, y: %.2f, z: %.2f]' %
                               (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    publisher = GoalPosePublisher()

    rclpy.spin(publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
