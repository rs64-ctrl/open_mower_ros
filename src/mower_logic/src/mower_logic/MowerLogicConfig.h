// ROS2 replacement for dynamic_reconfigure MowerLogic.cfg
// Generated config struct with parameter declaration support.
#ifndef MOWER_LOGIC__MOWER_LOGIC_CONFIG_H
#define MOWER_LOGIC__MOWER_LOGIC_CONFIG_H

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mower_logic {

struct MowerLogicConfig {
  int automatic_mode = 0;
  double undock_distance = 2.0;
  double undock_angled_distance = 0.0;
  double undock_angle = 0.0;
  bool undock_fixed_angle = true;
  bool undock_use_curve = true;
  double docking_distance = 2.0;
  double docking_approach_distance = 1.5;
  int docking_retry_count = 4;
  double docking_extra_time = 0.0;
  bool docking_redock = false;
  double docking_waiting_time = 0.0;
  double undocking_waiting_time = 0.0;
  int perimeter_signal = 0;
  int outline_count = 3;
  int outline_overlap_count = 0;
  double outline_offset = 0.0;
  double mow_angle_offset = 0.0;
  bool mow_angle_offset_is_absolute = false;
  double mow_angle_increment = 0.0;
  double tool_width = 0.14;
  bool enable_mower = false;
  bool manual_pause_mowing = false;
  double motor_hot_temperature = 70.0;
  double motor_cold_temperature = 40.0;
  double max_position_accuracy = 0.2;
  double gps_wait_time = 10.0;
  double gps_timeout = 10.0;
  bool add_fake_obstacle = false;
  bool ignore_gps_errors = false;
  int max_first_point_attempts = 3;
  int max_first_point_trim_attempts = 3;
  int rain_mode = 0;
  int rain_delay_minutes = 1;
  int rain_check_seconds = 0;
  int cu_rain_threshold = -1;
  int emergency_lift_period = -1;
  int emergency_tilt_period = -1;
  std::string emergency_input_config = "";
  int shutdown_esc_max_pitch = 0;

  static void declareParameters(rclcpp::Node::SharedPtr node) {
    node->declare_parameter("automatic_mode", 0);
    node->declare_parameter("undock_distance", 2.0);
    node->declare_parameter("undock_angled_distance", 0.0);
    node->declare_parameter("undock_angle", 0.0);
    node->declare_parameter("undock_fixed_angle", true);
    node->declare_parameter("undock_use_curve", true);
    node->declare_parameter("docking_distance", 2.0);
    node->declare_parameter("docking_approach_distance", 1.5);
    node->declare_parameter("docking_retry_count", 4);
    node->declare_parameter("docking_extra_time", 0.0);
    node->declare_parameter("docking_redock", false);
    node->declare_parameter("docking_waiting_time", 0.0);
    node->declare_parameter("undocking_waiting_time", 0.0);
    node->declare_parameter("perimeter_signal", 0);
    node->declare_parameter("outline_count", 3);
    node->declare_parameter("outline_overlap_count", 0);
    node->declare_parameter("outline_offset", 0.0);
    node->declare_parameter("mow_angle_offset", 0.0);
    node->declare_parameter("mow_angle_offset_is_absolute", false);
    node->declare_parameter("mow_angle_increment", 0.0);
    node->declare_parameter("tool_width", 0.14);
    node->declare_parameter("enable_mower", false);
    node->declare_parameter("manual_pause_mowing", false);
    node->declare_parameter("motor_hot_temperature", 70.0);
    node->declare_parameter("motor_cold_temperature", 40.0);
    node->declare_parameter("max_position_accuracy", 0.2);
    node->declare_parameter("gps_wait_time", 10.0);
    node->declare_parameter("gps_timeout", 10.0);
    node->declare_parameter("add_fake_obstacle", false);
    node->declare_parameter("ignore_gps_errors", false);
    node->declare_parameter("max_first_point_attempts", 3);
    node->declare_parameter("max_first_point_trim_attempts", 3);
    node->declare_parameter("rain_mode", 0);
    node->declare_parameter("rain_delay_minutes", 1);
    node->declare_parameter("rain_check_seconds", 0);
    node->declare_parameter("cu_rain_threshold", -1);
    node->declare_parameter("emergency_lift_period", -1);
    node->declare_parameter("emergency_tilt_period", -1);
    node->declare_parameter("emergency_input_config", std::string(""));
    node->declare_parameter("shutdown_esc_max_pitch", 0);
  }

  void fromNode(rclcpp::Node::SharedPtr node) {
    node->get_parameter("automatic_mode", automatic_mode);
    node->get_parameter("undock_distance", undock_distance);
    node->get_parameter("undock_angled_distance", undock_angled_distance);
    node->get_parameter("undock_angle", undock_angle);
    node->get_parameter("undock_fixed_angle", undock_fixed_angle);
    node->get_parameter("undock_use_curve", undock_use_curve);
    node->get_parameter("docking_distance", docking_distance);
    node->get_parameter("docking_approach_distance", docking_approach_distance);
    node->get_parameter("docking_retry_count", docking_retry_count);
    node->get_parameter("docking_extra_time", docking_extra_time);
    node->get_parameter("docking_redock", docking_redock);
    node->get_parameter("docking_waiting_time", docking_waiting_time);
    node->get_parameter("undocking_waiting_time", undocking_waiting_time);
    node->get_parameter("perimeter_signal", perimeter_signal);
    node->get_parameter("outline_count", outline_count);
    node->get_parameter("outline_overlap_count", outline_overlap_count);
    node->get_parameter("outline_offset", outline_offset);
    node->get_parameter("mow_angle_offset", mow_angle_offset);
    node->get_parameter("mow_angle_offset_is_absolute", mow_angle_offset_is_absolute);
    node->get_parameter("mow_angle_increment", mow_angle_increment);
    node->get_parameter("tool_width", tool_width);
    node->get_parameter("enable_mower", enable_mower);
    node->get_parameter("manual_pause_mowing", manual_pause_mowing);
    node->get_parameter("motor_hot_temperature", motor_hot_temperature);
    node->get_parameter("motor_cold_temperature", motor_cold_temperature);
    node->get_parameter("max_position_accuracy", max_position_accuracy);
    node->get_parameter("gps_wait_time", gps_wait_time);
    node->get_parameter("gps_timeout", gps_timeout);
    node->get_parameter("add_fake_obstacle", add_fake_obstacle);
    node->get_parameter("ignore_gps_errors", ignore_gps_errors);
    node->get_parameter("max_first_point_attempts", max_first_point_attempts);
    node->get_parameter("max_first_point_trim_attempts", max_first_point_trim_attempts);
    node->get_parameter("rain_mode", rain_mode);
    node->get_parameter("rain_delay_minutes", rain_delay_minutes);
    node->get_parameter("rain_check_seconds", rain_check_seconds);
    node->get_parameter("cu_rain_threshold", cu_rain_threshold);
    node->get_parameter("emergency_lift_period", emergency_lift_period);
    node->get_parameter("emergency_tilt_period", emergency_tilt_period);
    node->get_parameter("emergency_input_config", emergency_input_config);
    node->get_parameter("shutdown_esc_max_pitch", shutdown_esc_max_pitch);
  }

  void toNode(rclcpp::Node::SharedPtr node) const {
    node->set_parameter(rclcpp::Parameter("automatic_mode", automatic_mode));
    node->set_parameter(rclcpp::Parameter("undock_distance", undock_distance));
    node->set_parameter(rclcpp::Parameter("undock_angled_distance", undock_angled_distance));
    node->set_parameter(rclcpp::Parameter("undock_angle", undock_angle));
    node->set_parameter(rclcpp::Parameter("undock_fixed_angle", undock_fixed_angle));
    node->set_parameter(rclcpp::Parameter("undock_use_curve", undock_use_curve));
    node->set_parameter(rclcpp::Parameter("docking_distance", docking_distance));
    node->set_parameter(rclcpp::Parameter("docking_approach_distance", docking_approach_distance));
    node->set_parameter(rclcpp::Parameter("docking_retry_count", docking_retry_count));
    node->set_parameter(rclcpp::Parameter("docking_extra_time", docking_extra_time));
    node->set_parameter(rclcpp::Parameter("docking_redock", docking_redock));
    node->set_parameter(rclcpp::Parameter("docking_waiting_time", docking_waiting_time));
    node->set_parameter(rclcpp::Parameter("undocking_waiting_time", undocking_waiting_time));
    node->set_parameter(rclcpp::Parameter("perimeter_signal", perimeter_signal));
    node->set_parameter(rclcpp::Parameter("outline_count", outline_count));
    node->set_parameter(rclcpp::Parameter("outline_overlap_count", outline_overlap_count));
    node->set_parameter(rclcpp::Parameter("outline_offset", outline_offset));
    node->set_parameter(rclcpp::Parameter("mow_angle_offset", mow_angle_offset));
    node->set_parameter(rclcpp::Parameter("mow_angle_offset_is_absolute", mow_angle_offset_is_absolute));
    node->set_parameter(rclcpp::Parameter("mow_angle_increment", mow_angle_increment));
    node->set_parameter(rclcpp::Parameter("tool_width", tool_width));
    node->set_parameter(rclcpp::Parameter("enable_mower", enable_mower));
    node->set_parameter(rclcpp::Parameter("manual_pause_mowing", manual_pause_mowing));
    node->set_parameter(rclcpp::Parameter("motor_hot_temperature", motor_hot_temperature));
    node->set_parameter(rclcpp::Parameter("motor_cold_temperature", motor_cold_temperature));
    node->set_parameter(rclcpp::Parameter("max_position_accuracy", max_position_accuracy));
    node->set_parameter(rclcpp::Parameter("gps_wait_time", gps_wait_time));
    node->set_parameter(rclcpp::Parameter("gps_timeout", gps_timeout));
    node->set_parameter(rclcpp::Parameter("add_fake_obstacle", add_fake_obstacle));
    node->set_parameter(rclcpp::Parameter("ignore_gps_errors", ignore_gps_errors));
    node->set_parameter(rclcpp::Parameter("max_first_point_attempts", max_first_point_attempts));
    node->set_parameter(rclcpp::Parameter("max_first_point_trim_attempts", max_first_point_trim_attempts));
    node->set_parameter(rclcpp::Parameter("rain_mode", rain_mode));
    node->set_parameter(rclcpp::Parameter("rain_delay_minutes", rain_delay_minutes));
    node->set_parameter(rclcpp::Parameter("rain_check_seconds", rain_check_seconds));
    node->set_parameter(rclcpp::Parameter("cu_rain_threshold", cu_rain_threshold));
    node->set_parameter(rclcpp::Parameter("emergency_lift_period", emergency_lift_period));
    node->set_parameter(rclcpp::Parameter("emergency_tilt_period", emergency_tilt_period));
    node->set_parameter(rclcpp::Parameter("emergency_input_config", emergency_input_config));
    node->set_parameter(rclcpp::Parameter("shutdown_esc_max_pitch", shutdown_esc_max_pitch));
  }
};

}  // namespace mower_logic

#endif  // MOWER_LOGIC__MOWER_LOGIC_CONFIG_H
