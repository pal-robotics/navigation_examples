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

#include "costmap_filters_tutorial/dummy_map_mask.hpp"
#include "nav2_util/node_utils.hpp"


namespace costmap_filters_tutorial
{
DummyMapMask::DummyMapMask()
: mask_topic_("/dummy_mask") {}

void DummyMapMask::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to Lock Parent Node");
  }
  node_ = parent;

  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".mask_topic",
    rclcpp::ParameterValue(std::string()));

  node->get_parameter(
    plugin_name + ".mask_topic",
    mask_topic_);

  costmap_filter_info_publisher_ = node->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
    mask_topic_ + "_info",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Constant values for dummy mask topic
  costmap_filter_info_msg_.filter_mask_topic = mask_topic_;
  costmap_filter_info_msg_.type = 0;
  costmap_filter_info_msg_.base = 0.0;
  costmap_filter_info_msg_.multiplier = 1.0;
  mask_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    costmap_filter_info_msg_.filter_mask_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void DummyMapMask::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning DummyMapMask plugin");
  costmap_filter_info_publisher_.reset();
  mask_publisher_.reset();
}

void DummyMapMask::activate()
{
  RCLCPP_INFO(logger_, "Activating DummyMapMask plugin");
}

void DummyMapMask::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating DummyMapMask plugin");
}

bool DummyMapMask::generateMask()
{
  auto locked_node = node_.lock();
  if (!locked_node) {
    throw std::runtime_error("Unable to Lock Parent Node");
  }
  costmap_filter_info_msg_.header.frame_id = header_.frame_id;
  costmap_filter_info_msg_.header.stamp = locked_node->now();
  costmap_filter_info_publisher_->publish(costmap_filter_info_msg_);

  auto occupancy_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  occupancy_grid->header = header_;
  occupancy_grid->info = info_;

  occupancy_grid->data = std::vector<int8_t>(info_.width * info_.height, 0);
  for (size_t i = 0; i < occupancy_grid->data.size(); i++) {
    if (i % 2 == 0) {
      occupancy_grid->data.at(i) = 100;
    }
  }

  mask_publisher_->publish(*occupancy_grid);
  return true;
}
}  // namespace costmap_filters_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  costmap_filters_tutorial::DummyMapMask,
  pal_map_masks::MapMask)
