cmake_minimum_required(VERSION 3.8)
project(target_detection_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pal_nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  pal_nav2_core
  pluginlib
)

add_library(dummy_target_detector SHARED src/dummy_target_detector.cpp)
ament_target_dependencies(dummy_target_detector ${dependencies})

install(TARGETS dummy_target_detector
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

pluginlib_export_plugin_description_file(pal_nav2_core plugins.xml)

ament_package()


