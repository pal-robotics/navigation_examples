// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Unauthorized copying of this file, via any medium is strictly prohibited,
// unless it was supplied under the terms of a license agreement or
// nondisclosure agreement with PAL Robotics SL. In this case it may not be
// copied or disclosed except in accordance with the terms of that agreement.

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
