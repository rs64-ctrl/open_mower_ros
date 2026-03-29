cmake_minimum_required(VERSION 3.16)
project(mower_comms_v1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# ROS2 Dependencies
# ==============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mower_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xesc_msgs REQUIRED)
find_package(xesc_driver REQUIRED)
find_package(serial REQUIRED)

find_package(Boost REQUIRED)

# ==============================================================================
# Executable Target
# ==============================================================================

add_executable(mower_comms_v1
    src/mower_comms.cpp
)

target_include_directories(mower_comms_v1 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ==============================================================================
# ROS2 Dependencies (via ament)
# ==============================================================================

ament_target_dependencies(mower_comms_v1
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    mower_msgs
    xbot_msgs
    xesc_msgs
    xesc_driver
    serial
)

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS mower_comms_v1
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Ament Package Export
# ==============================================================================

ament_package()
