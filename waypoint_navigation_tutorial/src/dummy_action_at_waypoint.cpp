#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_util/node_utils.hpp"

namespace waypoint_navigation_tutorial
{

class DummyActionAtWaypoint : public nav2_core::WaypointTaskExecutor
{
private:
  bool is_enabled_;

  rclcpp::Logger logger_{ rclcpp::get_logger("dummy_action_at_waypoint") };
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

public:
  DummyActionAtWaypoint()
  {
    is_enabled_ = true;
  }

  ~DummyActionAtWaypoint() = default;

  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, const std::string& plugin_name) override
  {
    parent_ = parent;
    auto node = parent_.lock();
    if (!node)
    {
      throw std::runtime_error{ "Failed to lock node in dummy action at waypoint plugin!" };
    }
    logger_ = node->get_logger();

    nav2_util::declare_parameter_if_not_declared(node, plugin_name + ".enabled", rclcpp::ParameterValue(is_enabled_));
    node->get_parameter(plugin_name + ".enabled", is_enabled_);
  }

  bool processAtWaypoint(const geometry_msgs::msg::PoseStamped& /*curr_pose*/, const int& curr_waypoint_index) override
  {
    if (!is_enabled_)
    {
      RCLCPP_INFO(logger_, "DummyActionAtWaypoint is disabled");
      return true;
    }

    auto node = parent_.lock();
    if (!node)
    {
      throw std::runtime_error{ "Failed to lock node in dummy action at waypoint plugin!" };
    }

    // Here you can do your custom action

    RCLCPP_INFO(logger_, "Arrived at %i'th waypoint", curr_waypoint_index);
    return true;
  }
};
}  // namespace waypoint_navigation_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(waypoint_navigation_tutorial::DummyActionAtWaypoint, nav2_core::WaypointTaskExecutor)
