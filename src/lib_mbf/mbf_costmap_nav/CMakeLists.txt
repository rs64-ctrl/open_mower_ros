cmake_minimum_required(VERSION 3.5)
project(mbf_costmap_nav)

find_package(ament_cmake_ros REQUIRED)
set(dependencies
  geometry_msgs
  mbf_abstract_core
  mbf_abstract_nav
  mbf_costmap_core
  mbf_msgs
  mbf_utility
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  tf2
  tf2_ros
  lifecycle_msgs
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

add_library(mbf_costmap_server
  src/mbf_costmap_nav/costmap_navigation_server.cpp
  src/mbf_costmap_nav/costmap_planner_execution.cpp
  src/mbf_costmap_nav/costmap_controller_execution.cpp
  src/mbf_costmap_nav/costmap_recovery_execution.cpp
  src/mbf_costmap_nav/costmap_wrapper.cpp
  src/mbf_costmap_nav/footprint_helper.cpp
)
target_compile_features(mbf_costmap_server PUBLIC c_std_99 cxx_std_17)
target_include_directories(mbf_costmap_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mbf_costmap_server ${dependencies})

add_executable(mbf_costmap_nav_node
  src/move_base_server_node.cpp
)
target_compile_features(mbf_costmap_nav_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(mbf_costmap_nav_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mbf_costmap_nav_node mbf_costmap_server)
ament_target_dependencies(mbf_costmap_nav_node ${dependencies})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  mbf_costmap_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mbf_costmap_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
