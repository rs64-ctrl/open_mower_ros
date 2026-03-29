cmake_minimum_required(VERSION 3.8)
project(xesc_2040_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xesc_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(xesc_interface REQUIRED)

# driver library
add_library(xesc_2040_driver
  src/xesc_2040_interface.cpp
  src/xesc_2040_driver.cpp
)

target_include_directories(xesc_2040_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(xesc_2040_driver
  rclcpp
  std_msgs
  xesc_msgs
  serial
  xesc_interface
)

# Install
install(TARGETS xesc_2040_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs xesc_msgs serial xesc_interface)

ament_package()
