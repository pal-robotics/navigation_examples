// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.
//
// Author: Andrea Capodacqua

#ifndef GOAL_NAVIGATION_TUTORIAL__PLUGINS__ACTION__LOG_PRINTER_NODE_HPP_
#define GOAL_NAVIGATION_TUTORIAL__PLUGINS__ACTION__LOG_PRINTER_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace goal_navigation_tutorial
{
/**
 * @brief A BT node that logs a message to the console
 * with a specific log level that could be INFO, WARN, ERROR.
 */
class LogPrinter : public BT::AsyncActionNode
{
public:
/**
 * @brief A constructor for goal_navigation_tutorial::LogPrinter
 * 
 * @param xml_tag_name Name for the XML tag for this node
 * @param conf  BT node configuration
 */
    LogPrinter(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);
    /**
    * @brief Creates list of BT ports
    * @return BT::PortsList Containing basic ports along with node-specific ports
    */

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<std::string>("log_text", "Text to be logged"),
            BT::InputPort<std::string>("log_level", "Log level (INFO, WARN, ERROR)"),
        };
    }
private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */

    BT::NodeStatus tick() override;

    std::string log_text_;
    std::string log_level_;
    rclcpp::Node::SharedPtr node_;
};
}  // namespace goal_navigation_tutorial

#endif  // GOAL_NAVIGATION_TUTORIAL__PLUGINS__ACTION__LOG_PRINTER_NODE_HPP_
