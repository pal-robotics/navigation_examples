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


#include <memory>
#include <chrono>

#include "pal_nav2_msgs/action/navigate_through_waypoints.hpp"
#include "pal_nav2_msgs/msg/action.hpp"
#include "pal_nav2_msgs/msg/waypoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("pal_wp_navigation_demo_1");

  //  creation of the action clients
  auto ac_wp_nav =
    rclcpp_action::create_client<pal_nav2_msgs::action::NavigateThroughWaypoints>(
    node,
    "navigate_through_waypoints");

  RCLCPP_INFO(node->get_logger(), "Waiting for action server to start.");

  if (!ac_wp_nav->wait_for_action_server()) {
    RCLCPP_ERROR(
      node->get_logger(), "Action server NavigateThroughWaypoints not available after waiting");
    rclcpp::shutdown();
  }
  rclcpp::sleep_for(std::chrono::seconds(5));
  RCLCPP_INFO(node->get_logger(), "Action server started, sending goal.");

  pal_nav2_msgs::action::NavigateThroughWaypoints::Goal goal;
  goal.number_of_loops = 0;  // You can repeat the waypoints sequence a number of times
  goal.behavior_tree = "follow_waypoints_w_process";
  pal_nav2_msgs::msg::Waypoint wp_1, wp_2, wp_3;

  //  POSES

  wp_1.pose.header.frame_id = "map";
  wp_1.pose.pose.position.x = 1.15;
  wp_1.pose.pose.position.y = -2.03;
  wp_1.pose.pose.orientation.z = -0.582;
  wp_1.pose.pose.orientation.w = 0.813;

  wp_2.pose.header.frame_id = "map";
  wp_2.pose.pose.position.x = 1.39;
  wp_2.pose.pose.position.y = -3.96;
  wp_2.pose.pose.orientation.z = -0.7006;
  wp_2.pose.pose.orientation.w = 0.7135;

  wp_3.pose.header.frame_id = "map";
  wp_3.pose.pose.position.x = 3.7;
  wp_3.pose.pose.position.y = -6.39;
  wp_3.pose.pose.orientation.z = 0.7174;
  wp_3.pose.pose.orientation.w = 0.6966;

  //  ACTIONS

  pal_nav2_msgs::msg::Action wait_action_1, wait_action_2;
  pal_nav2_msgs::msg::Action spin_action_1, spin_action_2;

  wait_action_1.name = "wait_at_waypoint";
  wait_action_2.name = "wait_at_waypoint";
  spin_action_1.name = "spin_at_waypoint";
  spin_action_2.name = "spin_at_waypoint";

  rcl_interfaces::msg::Parameter enabled;
  enabled.name = "enabled";
  enabled.value = rcl_interfaces::msg::ParameterValue();
  enabled.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  enabled.value.bool_value = true;

  rcl_interfaces::msg::Parameter waypoint_pause_duration;
  waypoint_pause_duration.name = "waypoint_pause_duration";
  waypoint_pause_duration.value = rcl_interfaces::msg::ParameterValue();
  waypoint_pause_duration.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  waypoint_pause_duration.value.integer_value = 2000;

  rcl_interfaces::msg::Parameter waypoint_rotation_angle;
  waypoint_rotation_angle.name = "waypoint_rotation_angle";
  waypoint_rotation_angle.value = rcl_interfaces::msg::ParameterValue();
  waypoint_rotation_angle.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  waypoint_rotation_angle.value.double_value = 6.28;

  wait_action_1.parameters.push_back(waypoint_pause_duration);
  wait_action_1.parameters.push_back(enabled);

  waypoint_pause_duration.value.integer_value = 5000;
  wait_action_2.parameters.push_back(waypoint_pause_duration);
  wait_action_2.parameters.push_back(enabled);

  spin_action_1.parameters.push_back(waypoint_rotation_angle);
  spin_action_1.parameters.push_back(enabled);

  waypoint_rotation_angle.value.double_value = 3.14;
  spin_action_2.parameters.push_back(waypoint_rotation_angle);
  spin_action_2.parameters.push_back(enabled);

  wp_1.actions.push_back(wait_action_1);
  wp_2.actions.push_back(spin_action_1);
  wp_3.actions.push_back(wait_action_2);
  wp_3.actions.push_back(spin_action_2);

  // WAYPOINTS GOALS

  goal.waypoints.push_back(wp_1);
  goal.waypoints.push_back(wp_2);
  goal.waypoints.push_back(wp_3);

  auto future = ac_wp_nav->async_send_goal(goal);

  // EXIT

  auto result = rclcpp::spin_until_future_complete(node, future);
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to set parameters at waypoint");
    return false;
  }

  // rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return true;
}
