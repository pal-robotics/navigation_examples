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

#ifndef COSTMAP_FILTERS_TUTORIAL__DUMMY_COSTMAP_FILTER_HPP_
#define COSTMAP_FILTERS_TUTORIAL__DUMMY_COSTMAP_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace costmap_filters_tutorial
{
/**
 * @class DummyCostmapFilter
 * @brief Reads in a speed restriction mask and enables a robot to
 * dynamically adjust speed based on pose in map to slow in dangerous
 * areas. Done via absolute speed setting or percentage of maximum speed
 */
class DummyCostmapFilter : public nav2_costmap_2d::CostmapFilter
{
public:
  /**
   * @brief A constructor
   */
  DummyCostmapFilter();

  /**
   * @brief Initialize the filter and subscribe to the info topic
   */
  void initializeFilter(
    const std::string & filter_info_topic);

  /**
   * @brief Process the keepout layer at the current pose / bounds / grid
   */
  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose);

  /**
   * @brief Reset the costmap filter / topic / info
   */
  void resetFilter();

  /**
   * @brief If this filter is active
   */
  bool isActive();

private:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  /**
   * @brief Changes binary state of filter. Sends a message with new state.
   * @param state New binary state
   */
  void changeState(const bool state);

  // Working with filter info and mask
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr filter_mask_;

  std::string global_frame_;  // Frame of currnet layer (master_grid)

  double base_, multiplier_;
  // Filter values higher than this threshold,
  // will set binary state to non-default
  double flip_threshold_;
};

}  // namespace costmap_filters_tutorial

#endif  // COSTMAP_FILTERS_TUTORIAL__DUMMY_COSTMAP_FILTER_HPP_