cmake_minimum_required(VERSION 3.8)
include(ExternalProject)
project(xbot_driver_gps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)


set(EP_NEMATODE "NemaTode")
ExternalProject_Add(
        ${EP_NEMATODE}

        PREFIX ${EP_NEMATODE}
        GIT_REPOSITORY https://github.com/ClemensElflein/NemaTode
        GIT_TAG b9ea99be965167a67c6e238336de15304cdb8002
        GIT_SHALLOW OFF

        BUILD_ALWAYS OFF

        INSTALL_COMMAND cmake -E echo "Skipping install step."

        CMAKE_ARGS -DCMAKE_POLICY_DEFAULT_CMP0048=NEW
        CMAKE_CACHE_ARGS
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release -j
)


find_package(
        Boost
)

###########
## Build ##
###########

ExternalProject_Get_property(${EP_NEMATODE} SOURCE_DIR)
message("Source dir of ${EP_NEMATODE} = ${SOURCE_DIR}")
include_directories(${SOURCE_DIR}/include)

add_executable(driver_gps_node
        src/driver_gps_node.cpp
        src/gps_log.h
        src/devices/gps_device.cpp
        src/devices/gps_device.h
        src/devices/serial_gps_device.cpp
        src/devices/serial_gps_device.h
        src/devices/tcp_gps_device.cpp
        src/devices/tcp_gps_device.h
        src/interfaces/gps_interface.cpp
        src/interfaces/gps_interface.h
        src/interfaces/nmea_gps_interface.cpp
        src/interfaces/nmea_gps_interface.h
        src/interfaces/ublox_gps_interface.cpp
        src/interfaces/ublox_gps_interface.h
        src/interfaces/ubx_datatypes.h
)

ament_target_dependencies(driver_gps_node
        rclcpp
        geometry_msgs
        xbot_msgs
        serial
        robot_localization
        tf2_geometry_msgs
        tf2
        sensor_msgs
        std_msgs
        nmea_msgs
        rtcm_msgs
)

add_dependencies(driver_gps_node ${EP_NEMATODE})
ExternalProject_Get_property(${EP_NEMATODE} BINARY_DIR)
target_link_libraries(driver_gps_node ${GeographicLib_LIBRARIES} ${BINARY_DIR}/libNemaTode.a)
target_include_directories(driver_gps_node PRIVATE ${GeographicLib_INCLUDE_DIRS})

install(TARGETS
        driver_gps_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
