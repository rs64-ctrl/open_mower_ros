// ROS2 replacement for dynamic_reconfigure MowerOdometry.cfg
// Generated config struct with parameter declaration support.
#ifndef MOWER_LOGIC__MOWER_ODOMETRY_CONFIG_H
#define MOWER_LOGIC__MOWER_ODOMETRY_CONFIG_H

#include <rclcpp/rclcpp.hpp>

namespace mower_logic {

struct MowerOdometryConfig {
  double imu_offset = 0.0;
  double gps_antenna_offset = 0.0;
  double gps_filter_factor = 0.01;
  bool simulate_gps_outage = false;

  static void declareParameters(rclcpp::Node::SharedPtr node) {
    node->declare_parameter("imu_offset", 0.0);
    node->declare_parameter("gps_antenna_offset", 0.0);
    node->declare_parameter("gps_filter_factor", 0.01);
    node->declare_parameter("simulate_gps_outage", false);
  }

  void fromNode(rclcpp::Node::SharedPtr node) {
    node->get_parameter("imu_offset", imu_offset);
    node->get_parameter("gps_antenna_offset", gps_antenna_offset);
    node->get_parameter("gps_filter_factor", gps_filter_factor);
    node->get_parameter("simulate_gps_outage", simulate_gps_outage);
  }
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__MOWER_ODOMETRY_CONFIG_H
