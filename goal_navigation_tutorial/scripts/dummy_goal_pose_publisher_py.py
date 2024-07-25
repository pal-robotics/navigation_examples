#!/usr/bin/env python3

# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

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
