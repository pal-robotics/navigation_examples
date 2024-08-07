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
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class GoalPosePublisher : public rclcpp::Node
{
public:
  GoalPosePublisher()
  : Node("goal_pose_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {this->timer_callback();}
    );
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    // Goal position
    msg.pose.position.x = 1.5;
    msg.pose.position.y = 1.5;
    msg.pose.position.z = 0.0;
    // Goal orientation (Quaternion)
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    publisher_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(), "Publishing goal pose: [x: %.2f, y: %.2f, z: %.2f]",
      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    timer_->cancel();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GoalPosePublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
