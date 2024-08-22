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
#include "nav2_msgs/srv/set_initial_pose.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("pal_target_navigation_demo_1");

  rclcpp::sleep_for(std::chrono::seconds(1));


  //  creation of the action clients
  auto ac_target_nav =
    rclcpp_action::create_client<pal_nav2_msgs::action::NavigateToTarget>(
    node,
    "navigate_to_target");

  auto ac_set_initial_pose =
    node->create_client<nav2_msgs::srv::SetInitialPose>("set_initial_pose");

  RCLCPP_INFO(node->get_logger(), "Waiting for action server to start.");

  if (!ac_set_initial_pose->wait_for_service()) {
    RCLCPP_ERROR(
      node->get_logger(), "Service SetInitialPose not available after waiting");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(node->get_logger(), "Service SetInitialPose started, sending goal.");

  auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  nav2_msgs::srv::SetInitialPose::Response::SharedPtr srv_response;

  initial_pose.header.frame_id = "map";
  initial_pose.pose.pose.position.x = 0.462;
  initial_pose.pose.pose.position.y = 1.05;
  initial_pose.pose.pose.position.z = 0.0;
  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = 0.7920874589023841;
  initial_pose.pose.pose.orientation.w = 0.6104076158187772;

  request->pose = initial_pose;

  srv_response.reset();

  auto response_callback =
    [node, &srv_response](rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedFuture future_1) {
      try {
        auto response = future_1.get();
        if (response) {
          srv_response = response;
        } else {
          RCLCPP_ERROR(node->get_logger(), "Received empty response");
          srv_response = nullptr;
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          node->get_logger(), "Exception while waiting for response: %s",
          e.what());
        srv_response = nullptr;
      }
    };

  auto status = ac_set_initial_pose->async_send_request(request, response_callback);
  auto result_1 = rclcpp::spin_until_future_complete(node, status);

  if (result_1 == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Initial pose set");
    rclcpp::sleep_for(std::chrono::seconds(5));
    if (!ac_target_nav->wait_for_action_server()) {
      RCLCPP_ERROR(
        node->get_logger(), "Action server NavigateToTarget not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(node->get_logger(), "Action server started, sending goal.");

    pal_nav2_msgs::action::NavigateToTarget::Goal goal;
    goal.target_id = 0;
    goal.behavior_tree = "navigate_to_target";
    goal.detector = "aruco_target_detector";

    auto future_2 = ac_target_nav->async_send_goal(goal);

    // EXIT

    auto result_2 = rclcpp::spin_until_future_complete(node, future_2);
    if (result_2 != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to navigate to target");
      rclcpp::shutdown();
    }
  }
  rclcpp::shutdown();
  return 0;
}
