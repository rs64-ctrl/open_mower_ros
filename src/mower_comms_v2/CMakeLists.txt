cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mower_comms_v2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================================================================
# Options
# ==============================================================================

set(XBOT_FRAMEWORK_DIR "" CACHE PATH
    "Path to xbot_framework root. Auto-detected if not set.")

set(SERVICES_DIR "" CACHE PATH
    "Path to services JSON definitions. Auto-detected if not set.")

# ==============================================================================
# ROS2 Dependencies
# ==============================================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(mower_msgs REQUIRED)
find_package(xesc_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)

# ==============================================================================
# System Dependencies
# ==============================================================================

find_package(Boost)

# GeographicLib - for GPS coordinate transformations
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

# ==============================================================================
# xbot_framework Setup
# ==============================================================================

# Auto-detect xbot_framework location
if(NOT XBOT_FRAMEWORK_DIR)
    set(_xbot_candidates
        "${CMAKE_CURRENT_SOURCE_DIR}/../lib/xbot_framework"
        "${CMAKE_CURRENT_SOURCE_DIR}/../xbot_framework"
    )
    foreach(_path IN LISTS _xbot_candidates)
        if(EXISTS "${_path}/codegen/cmake/AddServiceInterface.cmake")
            set(XBOT_FRAMEWORK_DIR "${_path}")
            message(STATUS "Auto-detected xbot_framework: ${XBOT_FRAMEWORK_DIR}")
            break()
        endif()
    endforeach()
endif()

# Validate xbot_framework path
if(NOT EXISTS "${XBOT_FRAMEWORK_DIR}/codegen/cmake/AddServiceInterface.cmake")
    message(FATAL_ERROR
        "xbot_framework not found. Set -DXBOT_FRAMEWORK_DIR=/path/to/xbot_framework "
        "or place it at ../lib/xbot_framework relative to this package.")
endif()

# Add xbot_framework as subdirectory (it's not an ament package)
add_subdirectory(
    "${XBOT_FRAMEWORK_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/_xbot_framework"
    EXCLUDE_FROM_ALL
)

# Include the codegen macros
include("${XBOT_FRAMEWORK_DIR}/codegen/cmake/AddServiceInterface.cmake")

# ==============================================================================
# Services Directory
# ==============================================================================

# Auto-detect services directory
if(NOT SERVICES_DIR)
    set(_services_candidates
        "${CMAKE_CURRENT_SOURCE_DIR}/../../services"
        "${CMAKE_CURRENT_SOURCE_DIR}/../services"
    )
    foreach(_path IN LISTS _services_candidates)
        if(EXISTS "${_path}/imu_service.json")
            get_filename_component(SERVICES_DIR "${_path}" ABSOLUTE)
            message(STATUS "Auto-detected services directory: ${SERVICES_DIR}")
            break()
        endif()
    endforeach()
endif()

if(NOT EXISTS "${SERVICES_DIR}/imu_service.json")
    message(FATAL_ERROR
        "Services directory not found. Set -DSERVICES_DIR=/path/to/services "
        "or ensure services/ exists at project root.")
endif()

# ==============================================================================
# Source Files
# ==============================================================================

set(SOURCES
    src/mower_comms.cpp
    src/EmergencyServiceInterface.cpp
    src/EmergencyServiceInterface.h
    src/DiffDriveServiceInterface.cpp
    src/DiffDriveServiceInterface.h
    src/MowerServiceInterface.cpp
    src/MowerServiceInterface.h
    src/ImuServiceInterface.cpp
    src/ImuServiceInterface.h
    src/PowerServiceInterface.cpp
    src/PowerServiceInterface.h
    src/GpsServiceInterface.cpp
    src/GpsServiceInterface.h
)

# ==============================================================================
# Executable Target
# ==============================================================================

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# ==============================================================================
# ROS2 Dependencies (via ament)
# ==============================================================================

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    robot_localization
    xbot_msgs
    mower_msgs
    xesc_msgs
    nmea_msgs
    rtcm_msgs
)

# ==============================================================================
# System Libraries
# ==============================================================================

target_link_libraries(${PROJECT_NAME}
    ${GeographicLib_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${GeographicLib_INCLUDE_DIRS}
)

# ==============================================================================
# Service Interface Code Generation
# ==============================================================================

target_add_service_interface(${PROJECT_NAME}
    ImuServiceInterface        "${SERVICES_DIR}/imu_service.json")
target_add_service_interface(${PROJECT_NAME}
    EmergencyServiceInterface  "${SERVICES_DIR}/emergency_service.json")
target_add_service_interface(${PROJECT_NAME}
    DiffDriveServiceInterface  "${SERVICES_DIR}/diff_drive_service.json")
target_add_service_interface(${PROJECT_NAME}
    MowerServiceInterface      "${SERVICES_DIR}/mower_service.json")
target_add_service_interface(${PROJECT_NAME}
    PowerServiceInterface      "${SERVICES_DIR}/power_service.json")
target_add_service_interface(${PROJECT_NAME}
    GpsServiceInterface        "${SERVICES_DIR}/gps_service.json")

# ==============================================================================
# Install
# ==============================================================================

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Ament Package Export
# ==============================================================================

ament_package()
