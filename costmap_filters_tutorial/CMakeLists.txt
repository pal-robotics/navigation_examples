cmake_minimum_required(VERSION 3.8)
project(costmap_filters_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pal_map_masks REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav_msgs
  nav2_msgs
  pal_map_masks
  nav2_util
)

add_library(dummy_map_mask SHARED src/dummy_map_mask.cpp)
ament_target_dependencies(dummy_map_mask SYSTEM ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS dummy_map_mask
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(dummy_map_mask)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(pal_map_masks plugins.xml)
ament_package()

