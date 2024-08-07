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
from pal_nav2_msgs.action import NavigateThroughWaypoints
from pal_nav2_msgs.msg import Waypoint, Action
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import time


class NavigateClient(Node):
    def __init__(self):
        super().__init__('pal_wp_navigation_demo_1')
        self._action_client = ActionClient(self, NavigateThroughWaypoints,
                                           'navigate_through_waypoints')
        self._shutdown_called = False

    def send_goal(self, goal):
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def shutdown(self):
        if not self._shutdown_called and rclpy.ok():
            self._shutdown_called = True
            rclpy.shutdown()

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == NavigateThroughWaypoints.Result.NONE:
            self.get_logger().info('SUCCESS')
            rclpy.shutdown()
        else:
            self.get_logger().info('FAILURE')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    navigate_client = NavigateClient()

    # Wait for action server to be available
    navigate_client.get_logger().info('Waiting for action server to start.')
    if not navigate_client._action_client.wait_for_server(timeout_sec=10):
        navigate_client.get_logger().error
        ('Action server NavigateThroughWaypoints not available after waiting')
        navigate_client.shutdown()
        return

    time.sleep(5)
    navigate_client.get_logger().info('Action server started, sending goal.')

    goal = NavigateThroughWaypoints.Goal()
    goal.number_of_loops = 0  # You can repeat the waypoints sequence a number of times
    goal.behavior_tree = 'follow_waypoints_w_process'

    wp_1 = Waypoint()
    wp_2 = Waypoint()
    wp_3 = Waypoint()

    # POSES
    wp_1.pose = PoseStamped()
    wp_1.pose.header.frame_id = 'map'
    wp_1.pose.pose.position.x = 1.15
    wp_1.pose.pose.position.y = -2.03
    wp_1.pose.pose.orientation.z = -0.582
    wp_1.pose.pose.orientation.w = 0.813

    wp_2.pose = PoseStamped()
    wp_2.pose.header.frame_id = 'map'
    wp_2.pose.pose.position.x = 1.39
    wp_2.pose.pose.position.y = -3.96
    wp_2.pose.pose.orientation.z = -0.7006
    wp_2.pose.pose.orientation.w = 0.7135

    wp_3.pose = PoseStamped()
    wp_3.pose.header.frame_id = 'map'
    wp_3.pose.pose.position.x = 3.7
    wp_3.pose.pose.position.y = -6.39
    wp_3.pose.pose.orientation.z = 0.7174
    wp_3.pose.pose.orientation.w = 0.6966

    # ACTIONS
    wait_action_1 = Action()
    wait_action_2 = Action()
    spin_action_1 = Action()
    spin_action_2 = Action()

    wait_action_1.name = 'wait_at_waypoint'
    wait_action_2.name = 'wait_at_waypoint'
    spin_action_1.name = 'spin_at_waypoint'
    spin_action_2.name = 'spin_at_waypoint'

    enabled = Parameter()
    enabled.name = 'enabled'
    enabled.value = ParameterValue()
    enabled.value.type = ParameterType.PARAMETER_BOOL
    enabled.value.bool_value = True

    waypoint_pause_duration = Parameter()
    waypoint_pause_duration.name = 'waypoint_pause_duration'
    waypoint_pause_duration.value = ParameterValue()
    waypoint_pause_duration.value.type = ParameterType.PARAMETER_INTEGER
    waypoint_pause_duration.value.integer_value = 2000

    waypoint_rotation_angle = Parameter()
    waypoint_rotation_angle.name = 'waypoint_rotation_angle'
    waypoint_rotation_angle.value = ParameterValue()
    waypoint_rotation_angle.value.type = ParameterType.PARAMETER_DOUBLE
    waypoint_rotation_angle.value.double_value = 6.28

    wait_action_1.parameters.append(waypoint_pause_duration)
    wait_action_1.parameters.append(enabled)

    waypoint_pause_duration.value.integer_value = 5000
    wait_action_2.parameters.append(waypoint_pause_duration)
    wait_action_2.parameters.append(enabled)

    spin_action_1.parameters.append(waypoint_rotation_angle)
    spin_action_1.parameters.append(enabled)

    waypoint_rotation_angle.value.double_value = 3.14
    spin_action_2.parameters.append(waypoint_rotation_angle)
    spin_action_2.parameters.append(enabled)

    wp_1.actions.append(wait_action_1)
    wp_2.actions.append(spin_action_1)
    wp_3.actions.append(wait_action_2)
    wp_3.actions.append(spin_action_2)

    # WAYPOINTS GOALS
    goal.waypoints.append(wp_1)
    goal.waypoints.append(wp_2)
    goal.waypoints.append(wp_3)

    navigate_client.send_goal(goal)

    try:
        # Spin to process callbacks
        rclpy.spin(navigate_client)
    except KeyboardInterrupt:
        pass
    finally:
        navigate_client.shutdown()
        return 0


if __name__ == '__main__':

    main()
