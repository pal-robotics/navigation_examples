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
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from pal_nav2_msgs.action import NavigateToTarget


class TargetNavigationClient(Node):
    def __init__(self):
        super().__init__("pal_target_navigation_demo_1")

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.timer = self.create_timer(1.0, self.publish_initial_pose)
        self._action_client = ActionClient(self, NavigateToTarget, "navigate_to_target")
        self._shutdown_called = False

    def send_goal(self, goal):
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def shutdown(self):
        if not self._shutdown_called and rclpy.ok():
            self._shutdown_called = True
            rclpy.shutdown()

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == NavigateToTarget.Result.NONE:
            self.get_logger().info("SUCCESS")
            rclpy.shutdown()
        else:
            self.get_logger().info("FAILURE")
            rclpy.shutdown()

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = 0.03476977348327637
        initial_pose.pose.pose.position.y = 2.022066116333008
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = 0.7920874589023841
        initial_pose.pose.pose.orientation.w = 0.6104076158187772
        self.publisher_.publish(initial_pose)


def main(args=None):
    rclpy.init(args=args)
    target_navigation_client = TargetNavigationClient()

    # Wait for action server to be available
    target_navigation_client.get_logger().info("Waiting for action server to start.")
    if not target_navigation_client._action_client.wait_for_server(timeout_sec=10):
        target_navigation_client.get_logger().error(
            "Action server NavigateToTarget not available after waiting"
        )
        target_navigation_client.shutdown()
        return

    time.sleep(5)
    target_navigation_client.get_logger().info("Action server started, sending goal.")

    goal = NavigateToTarget.Goal()
    goal.target_id = 0
    goal.behavior_tree = "navigate_to_target"
    goal.detector = "aruco_target_detector"

    target_navigation_client.send_goal(goal)

    try:
        # Spin to process callbacks
        rclpy.spin(target_navigation_client)
    except KeyboardInterrupt:
        pass
    finally:
        target_navigation_client.shutdown()
        return 0


if __name__ == "__main__":

    main()
