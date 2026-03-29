cmake_minimum_required(VERSION 3.10)
project(xbot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message deps
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(XBOT_MSG_FILES
  "msg/AbsolutePose.msg"
  "msg/WheelTick.msg"
  "msg/SensorInfo.msg"
  "msg/SensorDataString.msg"
  "msg/SensorDataDouble.msg"
  "msg/RobotState.msg"
  "msg/MapSize.msg"
  "msg/ActionInfo.msg"
  "msg/MapOverlay.msg"
  "msg/MapOverlayPolygon.msg"
)

set(XBOT_SRV_FILES
  "srv/RegisterActionsSrv.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${XBOT_MSG_FILES}
  ${XBOT_SRV_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
