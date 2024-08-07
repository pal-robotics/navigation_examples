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
// Author: Andrea Capodacqua

#include <memory>
#include <string>

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
      const BT::NodeConfiguration & conf)
   : BT::AsyncActionNode(xml_tag_name, conf)
   {
      node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
      getInput("log_text", log_text_);
      getInput("log_level", log_level_);

      if (log_text_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "LogPrinter: log_text is empty");
      }
      if (log_level_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "LogPrinter: log_level is empty, setting to INFO");
            log_level_ = "INFO";
      }
   }

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
   BT::NodeStatus tick() override
   {
      if (log_level_ == "INFO") {
            RCLCPP_INFO(node_->get_logger(), "%s", log_text_.c_str());
            return BT::NodeStatus::SUCCESS;
      } else if (log_level_ == "WARN") {
            RCLCPP_WARN(node_->get_logger(), "%s", log_text_.c_str());
            return BT::NodeStatus::SUCCESS;
      } else if (log_level_ == "ERROR") {
            RCLCPP_ERROR(node_->get_logger(), "%s", log_text_.c_str());
            return BT::NodeStatus::SUCCESS;
      } else {
            RCLCPP_ERROR(node_->get_logger(), "Invalid log level: %s", log_level_.c_str());
            return BT::NodeStatus::FAILURE;
      }
   }

   std::string log_text_;
   std::string log_level_;
   rclcpp::Node::SharedPtr node_;
};

}  // namespace goal_navigation_tutorial

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
   factory.registerNodeType<goal_navigation_tutorial::LogPrinter>("LogPrinter");
}