cmake_minimum_required(VERSION 3.8)
project(mower_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mower_msgs REQUIRED)
find_package(mower_map REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot_positioning REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# xbot_framework for service interface codegen
set(XBOT_FRAMEWORK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../lib/xbot_framework")
add_subdirectory(
    "${XBOT_FRAMEWORK_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/_xbot_framework"
    EXCLUDE_FROM_ALL
)
include("${XBOT_FRAMEWORK_DIR}/codegen/cmake/AddService.cmake")

add_executable(mower_simulation
        src/mower_simulation.cpp
        src/SimRobot.cpp
        src/SimRobot.h
        src/services/imu_service/imu_service.cpp
        src/services/power_service/power_service.cpp
        src/services/emergency_service/emergency_service.cpp
        src/services/diff_drive_service/diff_drive_service.cpp
        src/services/mower_service/mower_service.cpp
        src/services/gps_service/gps_service.cpp
)

target_include_directories(mower_simulation PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(mower_simulation
        rclcpp
        tf2
        tf2_geometry_msgs
        tf2_ros
        mower_msgs
        mower_map
        xbot_msgs
        xbot_positioning
        nav_msgs
        geometry_msgs
        sensor_msgs
)

target_link_libraries(mower_simulation spdlog::spdlog)

target_add_service(mower_simulation ImuService ${CMAKE_CURRENT_SOURCE_DIR}/../../services/imu_service.json)
target_add_service(mower_simulation PowerService ${CMAKE_CURRENT_SOURCE_DIR}/../../services/power_service.json)
target_add_service(mower_simulation EmergencyService ${CMAKE_CURRENT_SOURCE_DIR}/../../services/emergency_service.json)
target_add_service(mower_simulation DiffDriveService ${CMAKE_CURRENT_SOURCE_DIR}/../../services/diff_drive_service.json)
target_add_service(mower_simulation MowerService ${CMAKE_CURRENT_SOURCE_DIR}/../../services/mower_service.json)
target_add_service(mower_simulation GpsService ${CMAKE_CURRENT_SOURCE_DIR}/../../services/gps_service.json)

install(TARGETS mower_simulation
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
