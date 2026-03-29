// roslog_compat.hpp - ROS1 logging macros mapped to ROS2 RCLCPP logging
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sstream>

#ifndef ROSLOG_COMPAT_LOGGER_NAME
#define ROSLOG_COMPAT_LOGGER_NAME "mower_comms_v1"
#endif

#ifndef ROSLOG_COMPAT_LOGGER
#define ROSLOG_COMPAT_LOGGER ::rclcpp::get_logger(ROSLOG_COMPAT_LOGGER_NAME)
#endif

// ===== printf-Style (ROS1: ROS_*) =====
#define ROS_DEBUG(...) RCLCPP_DEBUG(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_INFO(...)  RCLCPP_INFO(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)

// ===== *_ONCE =====
#define ROS_DEBUG_ONCE(...) RCLCPP_DEBUG_ONCE(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_INFO_ONCE(...)  RCLCPP_INFO_ONCE(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_WARN_ONCE(...)  RCLCPP_WARN_ONCE(ROSLOG_COMPAT_LOGGER,  __VA_ARGS__)
#define ROS_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)
#define ROS_FATAL_ONCE(...) RCLCPP_FATAL_ONCE(ROSLOG_COMPAT_LOGGER, __VA_ARGS__)

// ===== Stream-Style (ROS1: ROS_*_STREAM) =====
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

// ===== Stream-Style THROTTLE (ROS1: ROS_*_STREAM_THROTTLE) =====
// ROS2 does not have a built-in throttle with stream style, so we emulate it.
// The period is in seconds (double).
#define ROS_ERROR_STREAM_THROTTLE(period, x) do { \
  static rclcpp::Clock _throttle_clock(RCL_STEADY_TIME); \
  static rclcpp::Time _last_hit; \
  auto _now = _throttle_clock.now(); \
  if ((_now - _last_hit).seconds() >= (period)) { \
    _last_hit = _now; \
    std::ostringstream _oss; _oss<<x; \
    RCLCPP_ERROR(ROSLOG_COMPAT_LOGGER, "%s", _oss.str().c_str()); \
  } \
} while(0)

// ===== Stream-Style COND (ROS1: ROS_*_STREAM_COND) =====
#define ROS_WARN_STREAM_COND(cond, x) do { if (cond) { \
  std::ostringstream _oss; _oss<<x; \
  RCLCPP_WARN(ROSLOG_COMPAT_LOGGER, "%s", _oss.str().c_str()); \
} } while(0)
