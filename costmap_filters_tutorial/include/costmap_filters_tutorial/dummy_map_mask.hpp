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

#ifndef COSTMAP_FILTERS_TUTORIAL__DUMMY_MAP_MASK_HPP_
#define COSTMAP_FILTERS_TUTORIAL__DUMMY_MAP_MASK_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "pal_map_masks/core/map_mask.hpp"


namespace costmap_filters_tutorial
{
class DummyMapMask : public pal_map_masks::MapMask
{
public:
  /**
   * @brief DummyMapMask plugin of type MapMask
  */
  DummyMapMask();

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
    const std::string & plugin_name) override;

  /**
   * @brief Overridden method to cleanup DummyMapMask plugin.
   */
  void cleanup() override;
  /**
   * @brief Overridden method to activate DummyMapMask plugin.
   */
  void activate() override;

  /**
   * @brief Overridden method to deactivate DummyMapMask plugin.
   */
  void deactivate() override;

  /**
   * @brief Overridden method to implement logic for the Mask generation of DummyMapMask plugin.
   */
  bool generateMask() override;

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

#endif  // COSTMAP_FILTERS_TUTORIAL__DUMMY_MAP_MASK_HPP_
