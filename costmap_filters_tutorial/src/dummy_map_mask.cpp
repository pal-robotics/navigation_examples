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
// Author: Antonio Brandi

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "pal_map_masks/core/map_mask.hpp"
#include "nav2_util/node_utils.hpp"


namespace costmap_filters_tutorial
{
class DummyMapMask : public pal_map_masks::MapMask
{
public:
  /**
   * @brief DummyMapMask plugin of type MapMask
  */
  DummyMapMask(): mask_topic_("/dummy_mask") {}

  /**
   * @brief DummyMapMask destructor
  */
  ~DummyMapMask() = default;

  /**
   * @brief Overridden method to configure DummyMapMask plugin.
   * @param parent pointer to user's node
   * @param plugin_name name of the plugin assigned at runtime
  */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override
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

  /**
   * @brief Overridden method to cleanup DummyMapMask plugin.
   */
  void cleanup() override
  {
  RCLCPP_INFO(logger_, "Cleaning DummyMapMask plugin");
  costmap_filter_info_publisher_.reset();
  mask_publisher_.reset();
}

  /**
   * @brief Overridden method to activate DummyMapMask plugin.
   */
  void activate() override
  {
  RCLCPP_INFO(logger_, "Activating DummyMapMask plugin");
}

  /**
   * @brief Overridden method to deactivate DummyMapMask plugin.
   */
  void deactivate() override
  {
  RCLCPP_INFO(logger_, "Deactivating DummyMapMask plugin");
}

  /**
   * @brief Overridden method to implement logic for the Mask generation of DummyMapMask plugin.
   */
  bool generateMask() override
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

private:
  // Interfaces
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_ {rclcpp::get_logger("DummyMapMask")};

  // ROS2 service interfaces
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr costmap_filter_info_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_publisher_;

  // Info msg
  nav2_msgs::msg::CostmapFilterInfo costmap_filter_info_msg_;

  // Parameters
  std::string mask_topic_;
  std::string plugin_name_;
};
}  // namespace costmap_filters_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  costmap_filters_tutorial::DummyMapMask,
  pal_map_masks::MapMask)
