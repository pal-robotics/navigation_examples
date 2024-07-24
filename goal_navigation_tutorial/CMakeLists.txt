cmake_minimum_required(VERSION 3.5)
project(goal_navigation_tutorial)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)

add_executable(dummy_goal_pose_publisher src/dummy_goal_pose_publisher.cpp)
ament_target_dependencies(dummy_goal_pose_publisher rclcpp geometry_msgs)

add_executable(dummy_navigate_to_pose_client src/dummy_navigate_to_pose_client.cpp)
ament_target_dependencies(dummy_navigate_to_pose_client
  rclcpp
  nav2_msgs
  geometry_msgs
  rclcpp_action
  rclcpp_components
)

install(TARGETS dummy_goal_pose_publisher dummy_navigate_to_pose_client
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/dummy_goal_pose_publisher_py.py
  scripts/dummy_navigate_to_pose_client_py.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
