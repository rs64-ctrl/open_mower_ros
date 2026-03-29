cmake_minimum_required(VERSION 3.8)
project(mower_logic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ftc_local_planner REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mower_map REQUIRED)
find_package(mower_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(slic3r_coverage_planner REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot_positioning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CRYPTOPP REQUIRED libcrypto++)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CheckPoint.msg"
)

# mower_logic executable
add_executable(mower_logic_node
  src/mower_logic/mower_logic.cpp
  src/mower_logic/behaviors/DockingBehavior.cpp
  src/mower_logic/behaviors/UndockingBehavior.cpp
  src/mower_logic/behaviors/IdleBehavior.cpp
  src/mower_logic/behaviors/MowingBehavior.cpp
  src/mower_logic/behaviors/PerimeterDocking.cpp
  src/mower_logic/behaviors/AreaRecordingBehavior.cpp
)

target_include_directories(mower_logic_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CRYPTOPP_INCLUDE_DIRS}
)

ament_target_dependencies(mower_logic_node
  rclcpp
  rclcpp_action
  ftc_local_planner
  mbf_msgs
  mower_map
  mower_msgs
  nav_msgs
  slic3r_coverage_planner
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  sensor_msgs
  xbot_msgs
  xbot_positioning
  geometry_msgs
  std_msgs
  visualization_msgs
)

target_link_libraries(mower_logic_node ${CRYPTOPP_LIBRARIES})

# Link generated messages to the mower_logic target
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mower_logic_node "${cpp_typesupport_target}")

# monitoring executable
add_executable(monitoring src/monitoring/monitoring.cpp)

target_include_directories(monitoring PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(monitoring
  rclcpp
  mower_msgs
  xbot_msgs
  xbot_positioning
  geometry_msgs
  std_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  mower_logic_node
  monitoring
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
