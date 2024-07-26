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

#include <memory>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "costmap_filters_tutorial/dummy_costmap_filter.hpp"

#include "nav2_util/occ_grid_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace costmap_filters_tutorial
{
/**
 * @class DummyCostmapFilter
 * @brief Reads a map mask and prints a log message
 * when the robot is in a cell with a value higher than a threshold
 */
class DummyCostmapFilter : public nav2_costmap_2d::CostmapFilter
{
public:
  /**
   * @brief DummyCostmapFilter plugin of type CostmapFilter
   */
  DummyCostmapFilter()
  : filter_info_sub_(nullptr), mask_sub_(nullptr),
    filter_mask_(nullptr), global_frame_("")
  {
  }

  /**
   * @brief Initialize the filter and subscribe to the info topic
   */
  void initializeFilter(
    const std::string & filter_info_topic)
  {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    // Declare parameters specific to DummyCostmapFilter only
    declareParameter("mask_threshold", rclcpp::ParameterValue(50.0));
    node->get_parameter(name_ + "." + "mask_threshold", mask_threshold);

    filter_info_topic_ = filter_info_topic;
    // Setting new costmap filter info subscriber
    RCLCPP_INFO(
      logger_,
      "DummyCostmapFilter: Subscribing to \"%s\" topic for filter info...",
      filter_info_topic_.c_str());
    filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
      filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&DummyCostmapFilter::filterInfoCallback, this, std::placeholders::_1));

    // Get global frame required for binary state publisher
    global_frame_ = layered_costmap_->getGlobalFrameID();
  }

  /**
   * @brief Process the given map mask at the current pose
   */
  void process(
    nav2_costmap_2d::Costmap2D &,
    int, int, int, int,
    const geometry_msgs::msg::Pose2D & pose)
  {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!filter_mask_) {
      // Show warning message every 2 seconds to not litter an output
      RCLCPP_WARN_THROTTLE(
        logger_, *(clock_), 2000,
        "DummyCostmapFilter: Filter mask was not received");
      return;
    }

    geometry_msgs::msg::Pose2D mask_pose; // robot coordinates in mask frame

    // Transforming robot pose from current layer frame to mask frame
    if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
      return;
    }

    // Converting mask_pose robot position to filter_mask_ indexes (mask_robot_i, mask_robot_j)
    unsigned int mask_robot_i, mask_robot_j;
    if (!worldToMask(filter_mask_, mask_pose.x, mask_pose.y, mask_robot_i, mask_robot_j)) {
      // Robot went out of mask range. Set "false" state by-default
      RCLCPP_WARN(
        logger_,
        "DummyCostmapFilter: Robot is outside of filter mask. Resetting binary state to default.");
      return;
    }

    // Getting filter_mask data from cell where the robot placed
    int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
    if (mask_data == nav2_util::OCC_GRID_UNKNOWN) {
      // Corresponding filter mask cell is unknown.
      // Warn and do nothing.
      RCLCPP_WARN_THROTTLE(
        logger_, *(clock_), 2000,
        "DummyCostmapFilter: Filter mask [%i, %i] data is unknown. Do nothing.",
        mask_robot_i, mask_robot_j);
      return;
    }
    // Check mask threshold
    if (base_ + mask_data * multiplier_ > mask_threshold) {
      RCLCPP_INFO(logger_, "Robot is Inside the mask");
    } else {
      RCLCPP_INFO(logger_, "Robot is Outside the mask");
    }
  }

  /**
   * @brief Reset the costmap filter / topic / info
   */
  void resetFilter()
  {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    filter_info_sub_.reset();
    mask_sub_.reset();
  }

  /**
   * @brief If this filter is active
   */
  bool isActive()
  {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (filter_mask_) {
      return true;
    }
    return false;
  }

private:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
  {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    rclcpp_lifecycle::LifecycleNode::SharedPtr node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    if (!mask_sub_) {
      RCLCPP_INFO(
        logger_,
        "DummyCostmapFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
    } else {
      RCLCPP_WARN(
        logger_,
        "DummyCostmapFilter: New costmap filter info from %s topic. Updating old filter info.",
        filter_info_topic_.c_str());
      // Resetting previous subscriber each time when new costmap filter information arrives
      mask_sub_.reset();
    }

    // Set base_ and multiplier_
    base_ = msg->base;
    multiplier_ = msg->multiplier;
    // Set topic name to receive filter mask from
    mask_topic_ = msg->filter_mask_topic;

    // Setting new filter mask subscriber
    RCLCPP_INFO(
      logger_,
      "DummyCostmapFilter: Subscribing to \"%s\" topic for filter mask...",
      mask_topic_.c_str());
    mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&DummyCostmapFilter::maskCallback, this, std::placeholders::_1));
  }

  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

    if (!filter_mask_) {
      RCLCPP_INFO(
        logger_,
        "DummyCostmapFilter: Received filter mask from %s topic.", mask_topic_.c_str());
    } else {
      RCLCPP_WARN(
        logger_,
        "DummyCostmapFilter: New filter mask arrived from %s topic. Updating old filter mask.",
        mask_topic_.c_str());
      filter_mask_.reset();
    }

    filter_mask_ = msg;
  }

  // Working with filter info and mask
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr filter_mask_;

  std::string global_frame_;  // Frame of currnet layer (master_grid)

  double base_, multiplier_;
  double mask_threshold;
};

}  // namespace costmap_filters_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap_filters_tutorial::DummyCostmapFilter, nav2_costmap_2d::Layer)
