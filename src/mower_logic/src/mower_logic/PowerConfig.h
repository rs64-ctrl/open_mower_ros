// ROS2 replacement for dynamic_reconfigure Power.cfg
// Generated config struct with parameter declaration support.
#ifndef LL__POWER_CONFIG_H
#define LL__POWER_CONFIG_H

#include <rclcpp/rclcpp.hpp>

namespace ll {

struct PowerConfig {
  double battery_critical_voltage = -1.0;
  double battery_empty_voltage = -1.0;
  double battery_full_voltage = -1.0;
  double battery_critical_high_voltage = -1.0;
  double charge_critical_high_voltage = -1.0;
  double charge_critical_high_current = -1.0;

  static void declareParameters(rclcpp::Node::SharedPtr node, const std::string& prefix = "") {
    std::string p = prefix.empty() ? "" : prefix + ".";
    node->declare_parameter(p + "battery_critical_voltage", -1.0);
    node->declare_parameter(p + "battery_empty_voltage", -1.0);
    node->declare_parameter(p + "battery_full_voltage", -1.0);
    node->declare_parameter(p + "battery_critical_high_voltage", -1.0);
    node->declare_parameter(p + "charge_critical_high_voltage", -1.0);
    node->declare_parameter(p + "charge_critical_high_current", -1.0);
  }

  void fromNode(rclcpp::Node::SharedPtr node, const std::string& prefix = "") {
    std::string p = prefix.empty() ? "" : prefix + ".";
    node->get_parameter(p + "battery_critical_voltage", battery_critical_voltage);
    node->get_parameter(p + "battery_empty_voltage", battery_empty_voltage);
    node->get_parameter(p + "battery_full_voltage", battery_full_voltage);
    node->get_parameter(p + "battery_critical_high_voltage", battery_critical_high_voltage);
    node->get_parameter(p + "charge_critical_high_voltage", charge_critical_high_voltage);
    node->get_parameter(p + "charge_critical_high_current", charge_critical_high_current);
  }
};

}  // namespace ll

#endif  // LL__POWER_CONFIG_H
