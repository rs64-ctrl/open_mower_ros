// Created by Clemens Elflein on 3/28/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//

#include <rclcpp/rclcpp.hpp>
#include <mower_msgs/msg/esc_status.hpp>
#include <mower_msgs/msg/power.hpp>
#include <xbot_msgs/msg/sensor_data_string.hpp>

#include "mower_logic/MowerLogicConfig.h"
#include "mower_logic/PowerConfig.h"
#include "mower_msgs/msg/high_level_status.hpp"
#include "mower_msgs/msg/status.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include "xbot_msgs/msg/robot_state.hpp"
#include "xbot_msgs/msg/sensor_data_double.hpp"
#include "xbot_msgs/msg/sensor_info.hpp"

rclcpp::Publisher<xbot_msgs::msg::RobotState>::SharedPtr state_pub;
xbot_msgs::msg::RobotState state;

rclcpp::Node::SharedPtr n;

mower_logic::MowerLogicConfig mower_logic_config;
ll::PowerConfig power_config;

typedef const mower_msgs::msg::Status::SharedPtr StatusPtr;

// Sensor configuration
struct SensorConfig {
  std::string name;
  std::string unit;
  uint8_t value_desc;
  uint8_t sensor_type;
  std::function<double(StatusPtr)> getStatusSensorValueCB = nullptr;
  std::function<void(SensorConfig& sensor_config)> setSensorLimitsCB = nullptr;
  std::string param_path = "";
  std::function<bool()> existCB = nullptr;
  xbot_msgs::msg::SensorInfo si;
  rclcpp::Publisher<xbot_msgs::msg::SensorInfo>::SharedPtr si_pub;
  // Use variant publisher type - we'll track the type separately
  rclcpp::PublisherBase::SharedPtr data_pub_base;
  rclcpp::Publisher<xbot_msgs::msg::SensorDataDouble>::SharedPtr data_pub_double;
  rclcpp::Publisher<xbot_msgs::msg::SensorDataString>::SharedPtr data_pub_string;
};

// Forward declare
void set_limits_battery_v(SensorConfig& sensor_config);
void set_limits_charge_current(SensorConfig& sensor_config);
void set_limits_charge_v(SensorConfig& sensor_config);
void set_limits_esc_temp(SensorConfig& sensor_config);
void set_limits_mow_motor_current(SensorConfig& sensor_config);
void set_limits_mow_motor_rpm(SensorConfig& sensor_config);
void set_limits_mow_motor_temp(SensorConfig& sensor_config);

std::map<std::string, SensorConfig> sensor_configs;

void initSensorConfigs() {
  sensor_configs = {
    {"om_v_charge", {"V Charge", "V", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_VOLTAGE, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_charge_v, "", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_v_battery", {"V Battery", "V", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_VOLTAGE, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_battery_v, "", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_charge_current", {"Charge Current", "A", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_CURRENT, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_charge_current, "",
      [](){ return !n->get_parameter_or("ignore_charging_current", false); },
      {}, nullptr, nullptr, nullptr}},
    {"om_charge_state", {"Charge State", "", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::msg::SensorInfo::TYPE_STRING, nullptr, nullptr, "", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_left_esc_temp", {"Left ESC Temp", "deg.C", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_esc_temp, "left_xesc", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_right_esc_temp", {"Right ESC Temp", "deg.C", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_esc_temp, "right_xesc", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_mow_esc_temp", {"Mow ESC Temp", "deg.C", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE,
      [](StatusPtr msg) { return msg->mower_esc_temperature; }, &set_limits_esc_temp, "mower_xesc", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_mow_motor_temp", {"Mow Motor Temp", "deg.C", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE,
      [](StatusPtr msg) { return msg->mower_motor_temperature; }, &set_limits_mow_motor_temp, "mower_xesc",
      [](){ return n->get_parameter_or("mower_xesc.has_motor_temp", true); },
      {}, nullptr, nullptr, nullptr}},
    {"om_mow_motor_current", {"Mow Motor Current", "A", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_CURRENT, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE,
      [](StatusPtr msg) { return msg->mower_esc_current; }, &set_limits_mow_motor_current, "mower_xesc", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_mow_motor_rpm", {"Mow Motor Revolutions", "rpm", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_RPM, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE,
      [](StatusPtr msg) { return msg->mower_motor_rpm; }, &set_limits_mow_motor_rpm, "mower_xesc", nullptr, {}, nullptr, nullptr, nullptr}},
    {"om_gps_accuracy", {"GPS Accuracy", "m", xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_DISTANCE, xbot_msgs::msg::SensorInfo::TYPE_DOUBLE, nullptr, nullptr, "", nullptr, {}, nullptr, nullptr, nullptr}},
  };
}

void status_received(StatusPtr msg) {
  static rclcpp::Time last_update(0, 0, RCL_ROS_TIME);
  if ((rclcpp::Time(msg->stamp) - last_update).seconds() < 0.5) return;
  last_update = rclcpp::Time(msg->stamp);

  xbot_msgs::msg::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->stamp;

  for (auto& sc_pair : sensor_configs) {
    if (sc_pair.second.existCB && !sc_pair.second.existCB()) continue;

    if (sc_pair.second.getStatusSensorValueCB && sc_pair.second.data_pub_double) {
      sensor_data.data = sc_pair.second.getStatusSensorValueCB(msg);
      sc_pair.second.data_pub_double->publish(sensor_data);
    }
  }
  state.rain_detected = msg->rain_detected;
}

void high_level_status(const mower_msgs::msg::HighLevelStatus::SharedPtr msg) {
  state.gps_percentage = msg->gps_quality_percent;
  state.current_state = msg->state_name;
  state.current_sub_state = msg->sub_state_name;
  state.current_area = msg->current_area;
  state.current_path = msg->current_path;
  state.current_path_index = msg->current_path_index;
  state.battery_percentage = msg->battery_percent;
  state.emergency = msg->emergency;
  state.is_charging = msg->is_charging;

  state_pub->publish(state);
}

void pose_received(const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
  state.robot_pose = *msg;

  static rclcpp::Time last_update(0, 0, RCL_ROS_TIME);
  if ((rclcpp::Time(msg->header.stamp) - last_update).seconds() < 0.5) return;
  last_update = rclcpp::Time(msg->header.stamp);

  xbot_msgs::msg::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->header.stamp;
  sensor_data.data = msg->position_accuracy;

  auto sc_it = sensor_configs.find("om_gps_accuracy");
  if (sc_it != std::end(sensor_configs) && sc_it->second.data_pub_double) {
    sc_it->second.data_pub_double->publish(sensor_data);
  }
}

void power_received(const mower_msgs::msg::Power::SharedPtr msg) {
  static rclcpp::Time last_update(0, 0, RCL_ROS_TIME);
  if ((rclcpp::Time(msg->stamp) - last_update).seconds() < 0.5) return;
  last_update = rclcpp::Time(msg->stamp);
  {
    xbot_msgs::msg::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->v_charge;
    auto sc_it = sensor_configs.find("om_v_charge");
    if (sc_it != std::end(sensor_configs) && sc_it->second.data_pub_double) {
      sc_it->second.data_pub_double->publish(sensor_data);
    }
  }
  {
    xbot_msgs::msg::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->v_battery;
    auto sc_it = sensor_configs.find("om_v_battery");
    if (sc_it != std::end(sensor_configs) && sc_it->second.data_pub_double) {
      sc_it->second.data_pub_double->publish(sensor_data);
    }
  }
  {
    xbot_msgs::msg::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->charge_current;
    auto sc_it = sensor_configs.find("om_charge_current");
    if (sc_it != std::end(sensor_configs) && sc_it->second.data_pub_double) {
      sc_it->second.data_pub_double->publish(sensor_data);
    }
  }
  {
    xbot_msgs::msg::SensorDataString sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->charger_status;
    auto sc_it = sensor_configs.find("om_charge_state");
    if (sc_it != std::end(sensor_configs) && sc_it->second.data_pub_string) {
      sc_it->second.data_pub_string->publish(sensor_data);
    }
  }
}

void left_esc_status_received(const mower_msgs::msg::ESCStatus::SharedPtr msg) {
  static rclcpp::Time last_update(0, 0, RCL_ROS_TIME);
  auto now = n->get_clock()->now();
  if ((now - last_update).seconds() < 0.5) return;
  last_update = now;
  {
    xbot_msgs::msg::SensorDataDouble sensor_data;
    sensor_data.stamp = n->get_clock()->now();
    sensor_data.data = msg->temperature_pcb;
    auto sc_it = sensor_configs.find("om_left_esc_temp");
    if (sc_it != std::end(sensor_configs) && sc_it->second.data_pub_double) {
      sc_it->second.data_pub_double->publish(sensor_data);
    }
  }
}

void right_esc_status_received(const mower_msgs::msg::ESCStatus::SharedPtr msg) {
  static rclcpp::Time last_update(0, 0, RCL_ROS_TIME);
  auto now = n->get_clock()->now();
  if ((now - last_update).seconds() < 0.5) return;
  last_update = now;
  {
    xbot_msgs::msg::SensorDataDouble sensor_data;
    sensor_data.stamp = n->get_clock()->now();
    sensor_data.data = msg->temperature_pcb;
    auto sc_it = sensor_configs.find("om_right_esc_temp");
    if (sc_it != std::end(sensor_configs) && sc_it->second.data_pub_double) {
      sc_it->second.data_pub_double->publish(sensor_data);
    }
  }
}

void set_limits_battery_v(SensorConfig& sensor_config) {
  sensor_config.si.lower_critical_value = power_config.battery_critical_voltage;
  sensor_config.si.min_value = power_config.battery_empty_voltage;
  sensor_config.si.max_value = power_config.battery_full_voltage;
  sensor_config.si.upper_critical_value = power_config.battery_critical_high_voltage;
}

void set_limits_charge_v(SensorConfig& sensor_config) {
  sensor_config.si.upper_critical_value = power_config.charge_critical_high_voltage;
}

void set_limits_charge_current(SensorConfig& sensor_config) {
  sensor_config.si.upper_critical_value = power_config.charge_critical_high_current;
}

void set_limits_esc_temp(SensorConfig& sensor_config) {
  sensor_config.si.max_value = n->get_parameter_or(sensor_config.param_path + ".max_pcb_temp", 0);
}

void set_limits_mow_motor_current(SensorConfig& sensor_config) {
  sensor_config.si.upper_critical_value = n->get_parameter_or(sensor_config.param_path + ".motor_current_limit", 0.0);
}

void set_limits_mow_motor_rpm(SensorConfig& sensor_config) {
  sensor_config.si.lower_critical_value = n->get_parameter_or(sensor_config.param_path + ".min_motor_rpm_critical", 2300);
  sensor_config.si.min_value = n->get_parameter_or(sensor_config.param_path + ".min_motor_rpm", 2800);
  sensor_config.si.max_value = n->get_parameter_or(sensor_config.param_path + ".max_motor_rpm", 3800);
}

void set_limits_mow_motor_temp(SensorConfig& sensor_config) {
  sensor_config.si.max_value = mower_logic_config.motor_hot_temperature;
  sensor_config.si.min_value = mower_logic_config.motor_cold_temperature;
}

void registerSensors() {
  for (auto& sc_pair : sensor_configs) {
    if (sc_pair.second.existCB && !sc_pair.second.existCB()) {
      RCLCPP_INFO(n->get_logger(), "Skipped monitoring of sensor %s", sc_pair.first.c_str());
      continue;
    }

    sc_pair.second.si.sensor_id = sc_pair.first;
    sc_pair.second.si.sensor_name = sc_pair.second.name;
    sc_pair.second.si.unit = sc_pair.second.unit;
    sc_pair.second.si.value_type = sc_pair.second.sensor_type;
    sc_pair.second.si.value_description = sc_pair.second.value_desc;

    if (sc_pair.second.setSensorLimitsCB) sc_pair.second.setSensorLimitsCB(sc_pair.second);

    if (sc_pair.second.si.min_value && sc_pair.second.si.max_value) sc_pair.second.si.has_min_max = true;
    if (sc_pair.second.si.lower_critical_value) sc_pair.second.si.has_critical_low = true;
    if (sc_pair.second.si.upper_critical_value) sc_pair.second.si.has_critical_high = true;

    sc_pair.second.si_pub =
        n->create_publisher<xbot_msgs::msg::SensorInfo>("xbot_monitoring/sensors/" + sc_pair.first + "/info", rclcpp::QoS(1).transient_local());
    switch (sc_pair.second.si.value_type) {
      case xbot_msgs::msg::SensorInfo::TYPE_DOUBLE:
        sc_pair.second.data_pub_double =
            n->create_publisher<xbot_msgs::msg::SensorDataDouble>("xbot_monitoring/sensors/" + sc_pair.first + "/data", 10);
        break;
      case xbot_msgs::msg::SensorInfo::TYPE_STRING:
        sc_pair.second.data_pub_string =
            n->create_publisher<xbot_msgs::msg::SensorDataString>("xbot_monitoring/sensors/" + sc_pair.first + "/data", 10);
        break;
      default: RCLCPP_ERROR(n->get_logger(), "Invalid Sensor Data Type: %d", (int)sc_pair.second.si.value_type);
    }
    sc_pair.second.si_pub->publish(sc_pair.second.si);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  n = std::make_shared<rclcpp::Node>("monitoring");

  // Declare parameters for configs that monitoring reads
  mower_logic::MowerLogicConfig::declareParameters(n);
  mower_logic_config.fromNode(n);

  ll::PowerConfig::declareParameters(n, "power");
  power_config.fromNode(n, "power");

  // Parameter change callbacks
  auto param_cb = n->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter>&) -> rcl_interfaces::msg::SetParametersResult {
        RCLCPP_INFO(n->get_logger(), "Monitoring received new config");
        mower_logic_config.fromNode(n);
        power_config.fromNode(n, "power");
        registerSensors();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  initSensorConfigs();
  registerSensors();

  auto state_sub = n->create_subscription<mower_msgs::msg::HighLevelStatus>(
      "mower_logic/current_state", 10, high_level_status);
  auto status_state_subscriber = n->create_subscription<mower_msgs::msg::Status>(
      "/ll/mower_status", 10, status_received);
  auto power_state_subscriber = n->create_subscription<mower_msgs::msg::Power>(
      "/ll/power", 10, power_received);
  auto left_esc_status_state_subscriber = n->create_subscription<mower_msgs::msg::ESCStatus>(
      "/ll/diff_drive/left_esc_status", 10, left_esc_status_received);
  auto right_esc_status_state_subscriber = n->create_subscription<mower_msgs::msg::ESCStatus>(
      "/ll/diff_drive/right_esc_status", 10, right_esc_status_received);
  auto pose_state_subscriber = n->create_subscription<xbot_msgs::msg::AbsolutePose>(
      "/xbot_positioning/xb_pose", 10, pose_received);

  state_pub = n->create_publisher<xbot_msgs::msg::RobotState>("xbot_monitoring/robot_state", 10);

  rclcpp::spin(n);

  rclcpp::shutdown();
  return 0;
}
