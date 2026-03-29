cmake_minimum_required(VERSION 3.5)
project(mbf_costmap_core)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mbf_abstract_core REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  mbf_abstract_core
  mbf_utility
  nav2_costmap_2d
  rclcpp
  std_msgs
  tf2_ros
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
