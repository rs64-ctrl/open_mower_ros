cmake_minimum_required(VERSION 3.8)
project(ftc_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(mbf_abstract_core REQUIRED)
find_package(mbf_costmap_core REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PID.msg"
  "srv/PlannerGetProgress.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# ftc_local_planner library
add_library(ftc_local_planner_lib SHARED
  src/ftc_planner.cpp
  src/oscillation_detector.cpp
)
target_include_directories(ftc_local_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(ftc_local_planner_lib
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  mbf_abstract_core
  mbf_costmap_core
  mbf_utility
  mbf_msgs
  rcl_interfaces
  builtin_interfaces
)
# Link generated message/service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ftc_local_planner_lib ${cpp_typesupport_target})

# backward_forward_recovery library
add_library(backward_forward_recovery SHARED
  src/backward_forward_recovery.cpp
)
target_include_directories(backward_forward_recovery PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(backward_forward_recovery
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
  mbf_abstract_core
  mbf_costmap_core
  mbf_utility
)

# Export plugins
# NOTE: The first argument is the package that defines the base class.
# Export plugin descriptions for MBF
pluginlib_export_plugin_description_file(mbf_costmap_core mbf_plugin.xml)
pluginlib_export_plugin_description_file(mbf_costmap_core recovery_plugin.xml)

# Install
install(TARGETS ftc_local_planner_lib backward_forward_recovery
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES mbf_plugin.xml recovery_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ftc_local_planner_lib backward_forward_recovery)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  mbf_abstract_core
  mbf_msgs
)

ament_package()
