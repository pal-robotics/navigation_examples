#!/usr/bin/env python3

# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class NavigateToPoseActionClient(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_client')
        self._action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        # Goal position
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.position.z = 0.0
        # Goal orientation
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        # Behavior tree full path (if empty using "navigate_to_pose_w_replanning_and_recovery.xml")
        goal_msg.behavior_tree = ''

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        future.result()
        self.get_logger().info('Result received')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: distance remaining: {feedback.distance_remaining}')


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
