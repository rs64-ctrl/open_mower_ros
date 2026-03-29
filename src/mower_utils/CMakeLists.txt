cmake_minimum_required(VERSION 3.8)
project(mower_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(open_mower REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mower_map REQUIRED)
find_package(slic3r_coverage_planner REQUIRED)

# planner_test executable
add_executable(planner_test
  src/planner_test.cpp
)
ament_target_dependencies(planner_test
  rclcpp
  open_mower
  geometry_msgs
  nav_msgs
  mower_map
  slic3r_coverage_planner
)

# xbot_pose_converter executable
add_executable(xbot_pose_converter
  src/xbot_pose_converter.cpp
)
ament_target_dependencies(xbot_pose_converter
  rclcpp
  geometry_msgs
  xbot_msgs
)

install(TARGETS
  planner_test
  xbot_pose_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
