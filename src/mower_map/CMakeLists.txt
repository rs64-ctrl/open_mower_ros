cmake_minimum_required(VERSION 3.8)
project(mower_map)

## Compile as C++20 for designated initializers and nlohmann/json ordered_json support
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot_rpc REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapArea.msg"
  "srv/AddMowingAreaSrv.srv"
  "srv/GetDockingPointSrv.srv"
  "srv/GetMowingAreaSrv.srv"
  "srv/SetDockingPointSrv.srv"
  "srv/ClearNavPointSrv.srv"
  "srv/SetNavPointSrv.srv"
  "srv/ClearMapSrv.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

###########
## Build ##
###########

add_executable(mower_map_service src/mower_map_service.cpp)

ament_target_dependencies(mower_map_service
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  tf2
  tf2_geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  xbot_msgs
  xbot_rpc
)

target_link_libraries(mower_map_service
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)

# Link the generated message/service typesupport to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mower_map_service "${cpp_typesupport_target}")

#############
## Install ##
#############

install(TARGETS
  mower_map_service
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
