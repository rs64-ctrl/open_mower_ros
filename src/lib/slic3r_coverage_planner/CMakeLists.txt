cmake_minimum_required(VERSION 3.8)
include(ExternalProject)
include(ProcessorCount)
project(slic3r_coverage_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Path.msg"
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs builtin_interfaces
)

# External project for Slic3r
ProcessorCount(NPROC)
set(EP_SLIC3R "Slic3r")
ExternalProject_Add(
        ${EP_SLIC3R}

        PREFIX ${EP_SLIC3R}
        GIT_REPOSITORY https://github.com/ClemensElflein/Slic3r
        GIT_TAG 026c1380e0ad12176dd2fc8cdf8f6f181deeb071
        GIT_SHALLOW OFF

        BUILD_ALWAYS OFF

        INSTALL_COMMAND cmake -E echo "Skipping install step."
        UPDATE_COMMAND ""

        # Patch missing #include <fstream> needed by GCC 13+
        PATCH_COMMAND sed -i "1s/^/#include <fstream>\\n/" <SOURCE_DIR>/src/slic3r.cpp

        CMAKE_CACHE_ARGS
        SOURCE_SUBDIR src
        CMAKE_ARGS -DSLIC3R_BUILD_TESTS=OFF
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --parallel ${NPROC}
)

ExternalProject_Get_property(${EP_SLIC3R} SOURCE_DIR)
message("Source dir of ${EP_SLIC3R} = ${SOURCE_DIR}")

ExternalProject_Get_property(${EP_SLIC3R} BINARY_DIR)

# Build the executable
add_executable(slic3r_coverage_planner_node
        src/coverage_planner.cpp
        src/OutlinePoly.h
        src/OutlinePoly.cpp
)

target_include_directories(slic3r_coverage_planner_node SYSTEM PRIVATE
        ${SOURCE_DIR}/xs/src/libslic3r
        ${SOURCE_DIR}/xs/src
)

ament_target_dependencies(slic3r_coverage_planner_node
        rclcpp
        geometry_msgs
        nav_msgs
        std_msgs
        visualization_msgs
        tf2
        tf2_geometry_msgs
        builtin_interfaces
)

# Link against the generated interfaces from this package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(slic3r_coverage_planner_node
        "${cpp_typesupport_target}"
        ${BINARY_DIR}/liblibslic3r.a
        ${BINARY_DIR}/libadmesh.a
        ${BINARY_DIR}/libBSpline.a
        ${BINARY_DIR}/libexpat.a
        ${BINARY_DIR}/libminiz.a
        ${BINARY_DIR}/libpoly2tri.a
        ${BINARY_DIR}/libpolypartition.a
        ${BINARY_DIR}/libZipArchive.a
        ${BINARY_DIR}/libclipper.a
        ${Boost_LIBRARIES}
)

add_dependencies(slic3r_coverage_planner_node ${EP_SLIC3R})

# Install targets
install(TARGETS slic3r_coverage_planner_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
