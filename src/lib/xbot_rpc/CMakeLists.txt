cmake_minimum_required(VERSION 3.8)
project(xbot_rpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RpcRequest.msg"
  "msg/RpcResponse.msg"
  "msg/RpcError.msg"
  "srv/RegisterMethodsSrv.srv"
  DEPENDENCIES std_msgs
)

add_library(xbot_rpc_lib
  src/provider.cpp
)

target_include_directories(xbot_rpc_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(xbot_rpc_lib PUBLIC cxx_std_17)

ament_target_dependencies(xbot_rpc_lib PUBLIC
  rclcpp
  std_msgs
)

target_link_libraries(xbot_rpc_lib PUBLIC nlohmann_json::nlohmann_json)

# Link generated message/service headers to the library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(xbot_rpc_lib PUBLIC ${cpp_typesupport_target})

install(TARGETS xbot_rpc_lib
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
ament_export_dependencies(rclcpp std_msgs rosidl_default_runtime)

ament_package()
