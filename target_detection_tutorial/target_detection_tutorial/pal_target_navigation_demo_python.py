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
# Author: Martina Annicelli

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from pal_nav2_msgs.action import NavigateToTarget
from nav2_msgs.srv import SetInitialPose
import time


class TargetNavigationDemo(Node):

    def __init__(self):
        super().__init__('pal_target_navigation_demo_1')

        self.ac_target_nav = ActionClient(self, NavigateToTarget, 'navigate_to_target')
        self.ac_set_initial_pose = self.create_client(SetInitialPose, 'set_initial_pose')

        self.get_logger().info("Waiting for action server to start.")

        if not self.ac_set_initial_pose.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service SetInitialPose not available after waiting")
            rclpy.shutdown()
            return

        self.get_logger().info("Service SetInitialPose started, sending goal.")

        request = SetInitialPose.Request()
        initial_pose = PoseWithCovarianceStamped()

        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = 0.462
        initial_pose.pose.pose.position.y = 1.05
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.7920874589023841
        initial_pose.pose.pose.orientation.w = 0.6104076158187772

        request.pose = initial_pose

        future = self.ac_set_initial_pose.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Initial pose set")
        else:
            self.get_logger().error("Failed to set initial pose")
            rclpy.shutdown()
            return

        time.sleep(5)

        if not self.ac_target_nav.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server NavigateToTarget not available after waiting")
            rclpy.shutdown()
            return

        self.get_logger().info("Action server started, sending goal.")

        goal = NavigateToTarget.Goal()
        goal.target_id = 0
        goal.behavior_tree = "navigate_to_target"
        goal.detector = "aruco_target_detector"

        self._send_goal_future = self.ac_target_nav.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().error('Goal failed')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    pal_target_navigation_demo = TargetNavigationDemo()

    rclpy.spin(pal_target_navigation_demo)


if __name__ == '__main__':
    main()
