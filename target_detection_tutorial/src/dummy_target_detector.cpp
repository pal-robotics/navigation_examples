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
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pal_nav2_core/target_detector.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace target_detection_tutorial
{

class DummyTargetDetector : public pal_nav2_core::TargetDetector
{
public:
  DummyTargetDetector() = default;
  ~DummyTargetDetector() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> ) override
  {
    parent_ = parent;
    auto node = parent_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node in simple target detector plugin!"};
    }
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();
  }

  void cleanup() override
  {
    RCLCPP_INFO(logger_, "Cleaning up %s target detector", plugin_name_.c_str());
  }

  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s target detector", plugin_name_.c_str());
  }

  void deactivate() override
  {
    RCLCPP_INFO(logger_, "DummyTargetDetector: Deactivating target detector");
  }

  bool detectTarget(
    int id, geometry_msgs::msg::TransformStamped & transform,
    double & accuracy) override
  {
    if (id != 0) {
      RCLCPP_WARN(logger_, "%s: Target not detected", plugin_name_.c_str());
      return false;
    }
    accuracy = 0.8;
    transform.header.frame_id = "base_footprint";
    transform.child_frame_id = "target";
    transform.header.stamp = clock_->now();
    transform.transform.translation.x = 2.0;

    return true;
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("dummy_target_detector")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
};
}  // namespace target_detection_tutorial
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  target_detection_tutorial::DummyTargetDetector,
  pal_nav2_core::TargetDetector)
