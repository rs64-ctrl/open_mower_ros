cmake_minimum_required(VERSION 3.10)
project(mower_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interface dependencies used in .msg/.srv
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(MOWER_MSG_FILES
  msg/Status.msg
  msg/ImuRaw.msg
  msg/ESCStatus.msg
  msg/HighLevelStatus.msg
  msg/Perimeter.msg
  msg/Emergency.msg
  msg/Power.msg
  msg/DockingSensor.msg
)

set(MOWER_SRV_FILES
  srv/EmergencyStopSrv.srv
  srv/GPSControlSrv.srv
  srv/MowerControlSrv.srv
  srv/HighLevelControlSrv.srv
  srv/PerimeterControlSrv.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MOWER_MSG_FILES}
  ${MOWER_SRV_FILES}
  DEPENDENCIES
    std_msgs
    builtin_interfaces
)

find_package(rosidl_default_runtime REQUIRED)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
