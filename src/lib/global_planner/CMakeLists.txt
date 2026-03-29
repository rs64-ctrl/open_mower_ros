cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(mbf_costmap_core REQUIRED)
find_package(mbf_utility REQUIRED)

# global_planner library (plugin)
add_library(global_planner_lib SHARED
  src/quadratic_calculator.cpp
  src/dijkstra.cpp
  src/astar.cpp
  src/grid_path.cpp
  src/gradient_path.cpp
  src/orientation_filter.cpp
  src/planner_core.cpp
)
target_include_directories(global_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(global_planner_lib
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  rcl_interfaces
  mbf_costmap_core
  mbf_utility
)

# Standalone planner node
add_executable(planner
  src/plan_node.cpp
)
target_include_directories(planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(planner global_planner_lib)
ament_target_dependencies(planner
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Export plugin description for MBF
pluginlib_export_plugin_description_file(mbf_costmap_core bgp_plugin.xml)

# Install
install(TARGETS global_planner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES bgp_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(global_planner_lib)
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  rcl_interfaces
)

ament_package()
