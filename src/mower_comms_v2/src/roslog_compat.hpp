// roslog_compat.hpp
// Compatibility shim: maps ROS1 ROS_* logging macros to ROS2 RCLCPP_* equivalents.
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sstream>

// Standard logger name (used when not inside a Node class)
#ifndef ROSLOG_COMPAT_LOGGER_NAME
#define ROSLOG_COMPAT_LOGGER_NAME "mower_comms_v2"
#endif

#ifndef ROSLOG_COMPAT_LOGGER
#define ROSLOG_COMPAT_LOGGER ::rclcpp::get_logger(ROSLOG_COMPAT_LOGGER_NAME)
#endif

// ===== printf-style (ROS1: ROS_*) =====
#define ROS_DEBUG(...) RCLCPP_DEBUG(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_INFO(...)  RCLCPP_INFO(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)

// ===== *_ONCE variants =====
#define ROS_DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_INFO_ONCE(...)  RCLCPP_INFO_ONCE(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_WARN_ONCE(...)  RCLCPP_WARN_ONCE(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_FATAL_ONCE(...) RCLCPP_FATAL_ONCE(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)

// ===== stream-style (ROS1: ROS_*_STREAM) =====
#define ROS_DEBUG_STREAM(x) do { std::ostringstream _oss; _oss<<x; \
  RCLCPP_DEBUG(ROSLOG_COMPAT_LOGGER, "%s", _oss.str().c_str()); } while(0)
#define ROS_INFO_STREAM(x)  do { std::ostringstream _oss; _oss<<x; \
  RCLCPP_INFO(ROSLOG_COMPAT_LOGGER,  "%s", _oss.str().c_str()); } while(0)
#define ROS_WARN_STREAM(x)  do { std::ostringstream _oss; _oss<<x; \
  RCLCPP_WARN(ROSLOG_COMPAT_LOGGER,  "%s", _oss.str().c_str()); } while(0)
#define ROS_ERROR_STREAM(x) do { std::ostringstream _oss; _oss<<x; \
  RCLCPP_ERROR(ROSLOG_COMPAT_LOGGER, "%s", _oss.str().c_str()); } while(0)
#define ROS_FATAL_STREAM(x) do { std::ostringstream _oss; _oss<<x; \
  RCLCPP_FATAL(ROSLOG_COMPAT_LOGGER, "%s", _oss.str().c_str()); } while(0)
