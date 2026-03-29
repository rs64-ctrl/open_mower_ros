cmake_minimum_required(VERSION 3.8)

project(xesc_interface)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(xesc_msgs REQUIRED)

# Header-only library - install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(xesc_msgs)

ament_package()
