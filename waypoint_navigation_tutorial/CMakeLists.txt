cmake_minimum_required(VERSION 3.8)
project(waypoint_navigation_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)


set(dependencies
rclcpp
rclcpp_lifecycle
geometry_msgs
nav2_core
nav2_util
pluginlib
)

add_library(dummy_action_at_waypoint SHARED src/dummy_action_at_waypoint.cpp)
ament_target_dependencies(dummy_action_at_waypoint ${dependencies})

install(TARGETS dummy_action_at_waypoint
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_export_libraries(dummy_action_at_waypoint)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(waypoint_navigation_tutorial plugins.xml)

ament_package()
