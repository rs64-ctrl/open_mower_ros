cmake_minimum_required(VERSION 3.14)
project(xbot_remote)

## Compile as C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost)
find_package(websocketpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

###########
## Build ##
###########

add_executable(xbot_remote
        src/xbot_remote.cpp)

ament_target_dependencies(xbot_remote
  rclcpp
  xbot_msgs
  geometry_msgs
  std_msgs
)

target_link_libraries(xbot_remote
  nlohmann_json::nlohmann_json
  websocketpp::websocketpp
)

install(TARGETS xbot_remote
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
