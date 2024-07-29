// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.
//
// Author: Andrea Capodacqua

#include <memory>
#include <string>

#include "goal_navigation_tutorial/plugins/action/log_printer_node.hpp"

namespace goal_navigation_tutorial
{
LogPrinter::LogPrinter(
    const std::string & name,
    const BT::NodeConfiguration & conf)
: BT::AsyncActionNode(name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("log_text", log_text_);
    getInput("log_level", log_level_);

    if (log_text_.empty()) {
        RCLCPP_WARN(
        node_->get_logger(),
        "LogPrinter: log_text is empty");
    }
    if (log_level_.empty()) {
        RCLCPP_WARN(
        node_->get_logger(),
        "LogPrinter: log_level is empty, setting to INFO");
        log_level_ = "INFO";
    }
}

BT::NodeStatus LogPrinter::tick()
{
    if (log_level_ == "INFO") {
        RCLCPP_INFO(
        node_->get_logger(),
        "%s", log_text_.c_str());
        return BT::NodeStatus::SUCCESS;
    } else if (log_level_ == "WARN") {
        RCLCPP_WARN(
        node_->get_logger(),
        "%s", log_text_.c_str());
        return BT::NodeStatus::SUCCESS;
    } else if (log_level_ == "ERROR") {
        RCLCPP_ERROR(
        node_->get_logger(),
        "%s", log_text_.c_str());
        return BT::NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(
        node_->get_logger(),
        "Invalid log level: %s", log_level_.c_str());
        return BT::NodeStatus::FAILURE;
    }
}

}  // namespace goal_navigation_tutorial

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<goal_navigation_tutorial::LogPrinter>("LogPrinter");
}
