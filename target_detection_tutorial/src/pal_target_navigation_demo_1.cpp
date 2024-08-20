// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Martina Annicelli

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pal_nav2_msgs/action/navigate_to_target.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pal_target_navigation_demo_1");

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub =
    node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10);

  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
    std::chrono::seconds(1), [&pub]() -> void {
      geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;

      initial_pose.header.frame_id = "map";
      initial_pose.pose.pose.position.x = 0.03476977348327637;
      initial_pose.pose.pose.position.y = 2.022066116333008;
      initial_pose.pose.pose.position.z = 0.0;
      initial_pose.pose.pose.orientation.x = 0.0;
      initial_pose.pose.pose.orientation.y = 0.0;
      initial_pose.pose.pose.orientation.z = 0.7920874589023841;
      initial_pose.pose.pose.orientation.w = 0.6104076158187772;

      pub->publish(initial_pose);
    });

  //  creation of the action clients
  auto ac_target_nav =
    rclcpp_action::create_client<pal_nav2_msgs::action::NavigateToTarget>(
    node,
    "navigate_to_target");

  RCLCPP_INFO(node->get_logger(), "Waiting for action server to start.");

  if (!ac_target_nav->wait_for_action_server()) {
    RCLCPP_ERROR(
      node->get_logger(), "Action server NavigateToTarget not available after waiting");
    rclcpp::shutdown();
  }
  rclcpp::sleep_for(std::chrono::seconds(5));
  RCLCPP_INFO(node->get_logger(), "Action server started, sending goal.");

  pal_nav2_msgs::action::NavigateToTarget::Goal goal;
  goal.target_id = 0;
  goal.behavior_tree = "navigate_to_target";
  goal.detector = "aruco_target_detector";

  auto future = ac_target_nav->async_send_goal(goal);

  // EXIT

  auto result = rclcpp::spin_until_future_complete(node, future);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to navigate to target");
    return false;
  }

  timer->cancel();
  rclcpp::shutdown();
  return true;
}
