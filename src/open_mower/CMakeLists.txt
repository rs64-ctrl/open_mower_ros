cmake_minimum_required(VERSION 3.8)
project(open_mower)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install parameter/config files
install(DIRECTORY
  params/
  DESTINATION share/${PROJECT_NAME}/params
)

# Install config directory
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz files
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
