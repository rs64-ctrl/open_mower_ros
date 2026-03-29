cmake_minimum_required(VERSION 3.8)
project(xesc_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xesc_msgs REQUIRED)
find_package(vesc_driver REQUIRED)
find_package(xesc_2040_driver REQUIRED)
find_package(xesc_yfr4_driver REQUIRED)
find_package(xesc_interface REQUIRED)

###########
## Build ##
###########

# driver library
add_library(xesc_driver
  src/xesc_driver.cpp
)

target_include_directories(xesc_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(xesc_driver
  rclcpp
  std_msgs
  xesc_msgs
  xesc_interface
  vesc_driver
  xesc_2040_driver
  xesc_yfr4_driver
)

# node executable
add_executable(xesc_driver_node src/xesc_driver_node.cpp)

target_include_directories(xesc_driver_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(xesc_driver_node xesc_driver)

ament_target_dependencies(xesc_driver_node
  rclcpp
  std_msgs
  xesc_msgs
  xesc_interface
)

#############
## Install ##
#############

install(TARGETS xesc_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS xesc_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs xesc_msgs xesc_interface vesc_driver xesc_2040_driver xesc_yfr4_driver)

ament_package()
