cmake_minimum_required(VERSION 3.11)
project(xbot_monitoring)

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(xbot_rpc REQUIRED)

find_package(Boost REQUIRED)

## System dependencies are found with CMake's conventions
find_package(PahoMqttCpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

###########
## Build ##
###########

add_executable(xbot_monitoring
        src/xbot_monitoring.cpp)
ament_target_dependencies(xbot_monitoring
        rclcpp
        xbot_msgs
        geometry_msgs
        std_msgs
        xbot_rpc
)
target_link_libraries(xbot_monitoring
        PahoMqttCpp::paho-mqttpp3
        nlohmann_json::nlohmann_json
)

add_executable(xbot_sensor_example
        src/xbot_sensor_example.cpp)
ament_target_dependencies(xbot_sensor_example
        rclcpp
        xbot_msgs
)

add_executable(heatmap_generator
        src/heatmap_generator.cpp)
ament_target_dependencies(heatmap_generator
        rclcpp
        xbot_msgs
        sensor_msgs
        grid_map_core
        grid_map_ros
        grid_map_msgs
)

install(TARGETS
        xbot_monitoring
        xbot_sensor_example
        heatmap_generator
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
