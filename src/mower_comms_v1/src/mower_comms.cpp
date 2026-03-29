//
// Created by Clemens Elflein on 15.03.22.
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
#include "roslog_compat.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mower_msgs/msg/esc_status.hpp>
#include <mower_msgs/msg/emergency.hpp>
#include <mower_msgs/msg/high_level_status.hpp>
#include <mower_msgs/msg/imu_raw.hpp>
#include <mower_msgs/msg/power.hpp>
#include <mower_msgs/msg/status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <xbot_msgs/msg/wheel_tick.hpp>
#include <xesc_msgs/msg/xesc_state.hpp>
#include <xesc_msgs/msg/xesc_state_stamped.hpp>
#include <xesc_driver/xesc_driver.h>

#include <serial/serial.h>

#include <algorithm>
#include <bitset>

#include "COBS.h"
#include "boost/crc.hpp"
#include "ll_datatypes.h"

#include <mower_msgs/srv/emergency_stop_srv.hpp>
#include <mower_msgs/srv/high_level_control_srv.hpp>
#include <mower_msgs/srv/mower_control_srv.hpp>

using namespace std::chrono_literals;

// Forward declarations
static rclcpp::Node::SharedPtr g_node;

rclcpp::Publisher<mower_msgs::msg::Status>::SharedPtr status_pub;
rclcpp::Publisher<mower_msgs::msg::Power>::SharedPtr power_pub;
rclcpp::Publisher<mower_msgs::msg::Emergency>::SharedPtr emergency_pub;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr actual_twist_pub;
rclcpp::Publisher<mower_msgs::msg::ESCStatus>::SharedPtr status_left_esc_pub;
rclcpp::Publisher<mower_msgs::msg::ESCStatus>::SharedPtr status_right_esc_pub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr sensor_imu_pub;

COBS cobs;

// True, if ROS thinks there sould be an emergency
bool emergency_high_level = false;
// True, if the LL board thinks there should be an emergency
bool emergency_low_level = false;
// Bits are set showing which emergency is active
uint8_t active_low_level_emergency = 0;

// True, if the LL emergency should be cleared in the next request
bool ll_clear_emergency = false;

// True, if we can send to the low level board
bool allow_send = false;

// Current speeds (duty cycle) for the three ESCs
float speed_l = 0, speed_r = 0, speed_mow = 0, target_speed_mow = 0;

// Ticks / m and wheel distance for this robot
double wheel_ticks_per_m = 0.0;
double wheel_distance_m = 0.0;

// LL/HL configuration
struct ll_high_level_config llhl_config;

// Dynamic reconfigure replaced by ROS2 parameters.
// Struct to hold the mower_logic config values locally.
struct MowerLogicParams {
  int cu_rain_threshold = 0xffff;
  int emergency_lift_period = 0xffff;
  int emergency_tilt_period = 0xffff;
  int shutdown_esc_max_pitch = 0xff;
  std::string emergency_input_config;
};

struct PowerParams {
  double charge_critical_high_voltage = 0.0;
  double charge_critical_high_current = 0.0;
  double battery_critical_high_voltage = 0.0;
  double battery_empty_voltage = 0.0;
  double battery_full_voltage = 0.0;
};

MowerLogicParams mower_logic_config;
PowerParams power_config;

// Serial port and buffer for the low level connection
serial::Serial serial_port;
uint8_t out_buf[1000];
rclcpp::Time last_cmd_vel;

boost::crc_ccitt_type crc;

mower_msgs::msg::HighLevelStatus last_high_level_status;

xesc_driver::XescDriver* mow_xesc_interface;
xesc_driver::XescDriver* left_xesc_interface;
xesc_driver::XescDriver* right_xesc_interface;

// True, if we have wheel ticks (i.e. last_ticks is valid)
bool has_ticks;
uint32_t last_ticks_l = 0;
uint32_t last_ticks_r = 0;
rclcpp::Time last_ticks_stamp;
geometry_msgs::msg::TwistStamped measured_twist_msg{};

std::mutex ll_status_mutex;
struct ll_status last_ll_status = {0};

sensor_msgs::msg::Imu sensor_imu_msg;

rclcpp::Client<mower_msgs::srv::HighLevelControlSrv>::SharedPtr highLevelClient;

bool is_emergency() {
  return emergency_high_level || emergency_low_level;
}

void publishActuators() {
  speed_mow = target_speed_mow;

  // emergency or timeout -> send 0 speeds
  if (is_emergency()) {
    speed_l = 0;
    speed_r = 0;
    speed_mow = 0;
  }
  auto now = g_node->get_clock()->now();
  if ((now - last_cmd_vel).seconds() > 1.0) {
    speed_l = 0;
    speed_r = 0;
  }
  if ((now - last_cmd_vel).seconds() > 25.0) {
    speed_l = 0;
    speed_r = 0;
    speed_mow = 0;
  }

  if (mow_xesc_interface) {
    mow_xesc_interface->setDutyCycle(speed_mow);
  }
  // We need to invert the speed, because the ESC has the same config as the left one, so the motor is running in the
  // "wrong" direction
  left_xesc_interface->setDutyCycle(speed_l);
  right_xesc_interface->setDutyCycle(-speed_r);

  struct ll_heartbeat heartbeat = {.type = PACKET_ID_LL_HEARTBEAT,
                                   // If high level has emergency and LL does not know yet, we set it
                                   .emergency_requested = static_cast<uint8_t>(!emergency_low_level && emergency_high_level),
                                   .emergency_release_requested = static_cast<uint8_t>(ll_clear_emergency)};

  crc.reset();
  crc.process_bytes(&heartbeat, sizeof(struct ll_heartbeat) - 2);
  heartbeat.crc = crc.checksum();

  size_t encoded_size = cobs.encode((uint8_t*)&heartbeat, sizeof(struct ll_heartbeat), out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  if (serial_port.isOpen() && allow_send) {
    try {
      serial_port.write(out_buf, encoded_size);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Error writing to serial port");
    }
  }
}

void convertStatus(xesc_msgs::msg::XescStateStamped& vesc_status, mower_msgs::msg::ESCStatus& ros_esc_status) {
  if (vesc_status.state.connection_state != xesc_msgs::msg::XescState::XESC_CONNECTION_STATE_CONNECTED &&
      vesc_status.state.connection_state != xesc_msgs::msg::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW) {
    // ESC is disconnected
    ros_esc_status.status = mower_msgs::msg::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (vesc_status.state.fault_code) {
    ROS_ERROR_STREAM_THROTTLE(1, "Motor controller fault code: " << vesc_status.state.fault_code);
    // ESC has a fault
    ros_esc_status.status = mower_msgs::msg::ESCStatus::ESC_STATUS_ERROR;
  } else {
    // ESC is OK but standing still
    ros_esc_status.status = mower_msgs::msg::ESCStatus::ESC_STATUS_OK;
  }
  ros_esc_status.tacho = vesc_status.state.tacho;
  ros_esc_status.rpm = vesc_status.state.rpm;
  ros_esc_status.current = vesc_status.state.current_input;
  ros_esc_status.temperature_motor = vesc_status.state.temperature_motor;
  ros_esc_status.temperature_pcb = vesc_status.state.temperature_pcb;
}

void convertStatus(xesc_msgs::msg::XescStateStamped& vesc_status, uint8_t& esc_status, double& esc_temperature,
                   double& esc_current, double& motor_temperature, double& motor_rpm) {
  if (vesc_status.state.connection_state != xesc_msgs::msg::XescState::XESC_CONNECTION_STATE_CONNECTED &&
      vesc_status.state.connection_state != xesc_msgs::msg::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW) {
    // ESC is disconnected
    esc_status = mower_msgs::msg::ESCStatus::ESC_STATUS_DISCONNECTED;
  } else if (vesc_status.state.fault_code) {
    ROS_ERROR_STREAM_THROTTLE(1, "Motor controller fault code: " << vesc_status.state.fault_code);
    // ESC has a fault
    esc_status = mower_msgs::msg::ESCStatus::ESC_STATUS_ERROR;
  } else {
    // ESC is OK but standing still
    esc_status = mower_msgs::msg::ESCStatus::ESC_STATUS_OK;
  }
  motor_rpm = vesc_status.state.rpm;
  esc_current = vesc_status.state.current_input;
  motor_temperature = vesc_status.state.temperature_motor;
  esc_temperature = vesc_status.state.temperature_pcb;
}

void publishStatus() {
  mower_msgs::msg::Status status_msg;
  status_msg.stamp = g_node->get_clock()->now();

  if (last_ll_status.status_bitmask & 1) {
    // LL OK, fill the message
    status_msg.mower_status = mower_msgs::msg::Status::MOWER_STATUS_OK;
  } else {
    // LL initializing
    status_msg.mower_status = mower_msgs::msg::Status::MOWER_STATUS_INITIALIZING;
  }

  status_msg.raspberry_pi_power = (last_ll_status.status_bitmask & 0b00000010) != 0;
  status_msg.is_charging = (last_ll_status.status_bitmask & 0b00000100) != 0;
  status_msg.esc_power = (last_ll_status.status_bitmask & 0b00001000) != 0;
  status_msg.rain_detected = (last_ll_status.status_bitmask & 0b00010000) != 0;
  status_msg.sound_module_available = (last_ll_status.status_bitmask & 0b00100000) != 0;
  status_msg.sound_module_busy = (last_ll_status.status_bitmask & 0b01000000) != 0;
  status_msg.ui_board_available = (last_ll_status.status_bitmask & 0b10000000) != 0;
  status_msg.mow_enabled = !(target_speed_mow == 0);

  // overwrite emergency with the LL value.
  emergency_low_level = last_ll_status.emergency_bitmask > 0;
  active_low_level_emergency = last_ll_status.emergency_bitmask & 0xFE;
  if (!emergency_low_level) {
    // it obviously worked, reset the request
    ll_clear_emergency = false;
  } else {
    ROS_ERROR_STREAM_THROTTLE(1, "Low Level Emergency. Bitmask was: " << (int)last_ll_status.emergency_bitmask);
  }

  // True, if high or low level emergency condition is present
  mower_msgs::msg::Emergency emergency_msg{};
  emergency_msg.stamp = status_msg.stamp;
  emergency_msg.active_emergency = active_low_level_emergency > 0;
  emergency_msg.latched_emergency = is_emergency();
  emergency_msg.reason = "";
  emergency_pub->publish(emergency_msg);

  mower_msgs::msg::Power power_msg{};
  power_msg.stamp = status_msg.stamp;
  power_msg.v_battery = last_ll_status.v_system;
  power_msg.v_charge = last_ll_status.v_charge;
  power_msg.charge_current = last_ll_status.charging_current;
  power_msg.charger_enabled = (last_ll_status.status_bitmask & LL_STATUS_BIT_CHARGING) != 0;
  power_msg.charger_status = "N/A";
  power_pub->publish(power_msg);

  xesc_msgs::msg::XescStateStamped mow_status{}, left_status{}, right_status{};
  if (mow_xesc_interface) {
    mow_xesc_interface->getStatus(mow_status);
  } else {
    mow_status.state.connection_state = xesc_msgs::msg::XescState::XESC_CONNECTION_STATE_DISCONNECTED;
  }
  left_xesc_interface->getStatus(left_status);
  right_xesc_interface->getStatus(right_status);

  mower_msgs::msg::ESCStatus left_esc_status{};
  mower_msgs::msg::ESCStatus right_esc_status{};

  // convertStatus(mow_status, status_msg.mow_esc_status);
  convertStatus(left_status, left_esc_status);
  convertStatus(right_status, right_esc_status);

  status_msg.mower_esc_current = static_cast<float>(mow_status.state.current_input);
  status_msg.mower_esc_status = mow_status.state.connection_state;
  status_msg.mower_motor_rpm = mow_status.state.rpm;
  status_msg.mower_esc_temperature = static_cast<float>(mow_status.state.temperature_pcb);
  status_msg.mower_motor_temperature = static_cast<float>(mow_status.state.temperature_motor);

  status_pub->publish(status_msg);
  status_left_esc_pub->publish(left_esc_status);
  status_right_esc_pub->publish(right_esc_status);

  if (!has_ticks) {
    last_ticks_stamp = status_msg.stamp;
    last_ticks_l = left_status.state.tacho_absolute;
    last_ticks_r = right_status.state.tacho_absolute;
    has_ticks = true;
  } else {
    bool wheel_direction_l = left_status.state.direction && abs(left_status.state.duty_cycle) > 0;
    bool wheel_direction_r = !right_status.state.direction && abs(right_status.state.duty_cycle) > 0;

    double dt = (rclcpp::Time(status_msg.stamp) - last_ticks_stamp).seconds();

    double d_wheel_l = (double)(left_status.state.tacho_absolute - last_ticks_l) * (1 / wheel_ticks_per_m);
    double d_wheel_r = (double)(right_status.state.tacho_absolute - last_ticks_r) * (1 / wheel_ticks_per_m);

    if (wheel_direction_l) {
      d_wheel_l *= -1.0;
    }
    if (wheel_direction_r) {
      d_wheel_r *= -1.0;
    }

    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    double vx = d_ticks / dt;
    double vr = -(d_wheel_l + d_wheel_r) / (2.0f * dt);
    last_ticks_stamp = status_msg.stamp;
    last_ticks_l = left_status.state.tacho_absolute;
    last_ticks_r = right_status.state.tacho_absolute;

    measured_twist_msg.header.frame_id = "base_link";
    measured_twist_msg.header.stamp = status_msg.stamp;
    // ROS2: no header.seq
    measured_twist_msg.twist.linear.x = vx;
    measured_twist_msg.twist.angular.z = vr;

    actual_twist_pub->publish(measured_twist_msg);
  }
}

std::string getHallConfigsString(const HallConfig* hall_configs, const size_t size) {
  std::string str;

  // Parse hall_configs and build a readable string
  for (size_t i = 0; i < size; i++) {
    if (str.length()) str.append(", ");
    if (hall_configs->active_low) str.append("!");
    switch (hall_configs->mode) {
      case HallMode::OFF: str.append("I"); break;
      case HallMode::LIFT_TILT: str.append("L"); break;
      case HallMode::STOP: str.append("S"); break;
      case HallMode::UNDEFINED: str.append("U"); break;
      default: break;
    }
    hall_configs++;
  }

  return str;
}

void publishLowLevelConfig(const uint8_t pkt_type) {
  if (!serial_port.isOpen() || !allow_send) return;

  // Prepare the pkt
  size_t size = sizeof(struct ll_high_level_config) + 3;  // +1 type, +2 crc
  uint8_t buf[size];

  // Send config and request a config answer
  buf[0] = pkt_type;

  // Copy our live config into the message (behind type)
  memcpy(&buf[1], &llhl_config, sizeof(struct ll_high_level_config));

  // Member access to buffer
  struct ll_high_level_config* buf_config = (struct ll_high_level_config*)&buf[1];

  // CRC
  crc.reset();
  crc.process_bytes(buf, sizeof(struct ll_high_level_config) + 1);  // + type
  buf[size - 1] = (crc.checksum() >> 8) & 0xFF;
  buf[size - 2] = crc.checksum() & 0xFF;

  // COBS
  size_t encoded_size = cobs.encode(buf, size, out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  // Send
  try {
    // Let's be verbose for easier follow-up
    ROS_INFO(
        "Send ll_high_level_config packet %#04x\n"
        "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
        "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
        "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
        "\t lift_period=%d, tilt_period=%d,\n"
        "\t shutdown_esc_max_pitch=%d,\n"
        "\t language=\"%.2s\", volume=%d\n"
        "\t hall_configs=\"%s\"",
        buf[0], (int)buf_config->options.dfp_is_5v, (int)buf_config->options.background_sounds,
        (int)buf_config->options.ignore_charging_current, buf_config->v_charge_cutoff, buf_config->i_charge_cutoff,
        buf_config->v_battery_cutoff, buf_config->v_battery_empty, buf_config->v_battery_full, buf_config->lift_period,
        buf_config->tilt_period, buf_config->shutdown_esc_max_pitch, buf_config->language, buf_config->volume,
        getHallConfigsString(buf_config->hall_configs, MAX_HALL_INPUTS).c_str());

    serial_port.write(out_buf, encoded_size);
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Error writing to serial port");
  }
}

/**
 * @brief A simple config tracker (struct-class) for managing lost response packets as well as simpler handling of
 * LowLevel reboots or flash period.
 */
struct {
  rclcpp::Time last_config_req;    // Time when last config request was sent
  unsigned int tries_left = 0;  // Remaining request tries before giving up

  void ackResponse() {
    // Call this on receive of a response packet to stop monitoring
    tries_left = 0;
  };

  void setDirty() {
    // Call this for indicating that config packet need to be resend, i.e. die to LL-reboot
    tries_left = 5;
  };

  void check() {
    if (!tries_left ||                                            // No request tries left (probably old LL-FW)
        !serial_port.isOpen() || !allow_send ||                   // Serial not ready
        (g_node->get_clock()->now() - last_config_req).seconds() < 0.5)  // Timeout waiting for response not reached
      return;
    publishLowLevelConfig(PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);
    last_config_req = g_node->get_clock()->now();
    tries_left--;
    ROS_WARN_STREAM_COND(
        !tries_left, "Didn't received a config packet from LowLevel in time. Is your LowLevel firmware up-to-date?");
  };
} configTracker;

void publishActuatorsTimerTask() {
  publishActuators();
  publishStatus();
  configTracker.check();
}

void setMowEnabled(
    const std::shared_ptr<mower_msgs::srv::MowerControlSrv::Request> req,
    std::shared_ptr<mower_msgs::srv::MowerControlSrv::Response> /*res*/) {
  if (req->mow_enabled && !is_emergency()) {
    target_speed_mow = req->mow_direction ? 1 : -1;
  } else {
    target_speed_mow = 0;
  }
  ROS_INFO_STREAM("Setting mow enabled to " << target_speed_mow);
}

void setEmergencyStop(
    const std::shared_ptr<mower_msgs::srv::EmergencyStopSrv::Request> req,
    std::shared_ptr<mower_msgs::srv::EmergencyStopSrv::Response> /*res*/) {
  if (req->emergency) {
    ROS_ERROR_STREAM("Setting emergency!!");
    ll_clear_emergency = false;
  } else {
    ll_clear_emergency = true;
  }
  // Set the high level emergency instantly. Low level value will be set on next update.
  emergency_high_level = req->emergency;
  publishActuators();
}

void highLevelStatusReceived(const mower_msgs::msg::HighLevelStatus::SharedPtr msg) {
  struct ll_high_level_state hl_state = {.type = PACKET_ID_LL_HIGH_LEVEL_STATE,
                                         .current_mode = msg->state,
                                         .gps_quality = static_cast<uint8_t>(msg->gps_quality_percent * 100.0)};

  crc.reset();
  crc.process_bytes(&hl_state, sizeof(struct ll_high_level_state) - 2);
  hl_state.crc = crc.checksum();

  size_t encoded_size = cobs.encode((uint8_t*)&hl_state, sizeof(struct ll_high_level_state), out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  if (serial_port.isOpen() && allow_send) {
    try {
      serial_port.write(out_buf, encoded_size);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Error writing to serial port");
    }
  }
}

void velReceived(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // TODO: update this to rad/s values and implement xESC speed control
  last_cmd_vel = g_node->get_clock()->now();
  speed_r = msg->linear.x + 0.5 * wheel_distance_m * msg->angular.z;
  speed_l = msg->linear.x - 0.5 * wheel_distance_m * msg->angular.z;

  if (speed_l >= 1.0) {
    speed_l = 1.0;
  } else if (speed_l <= -1.0) {
    speed_l = -1.0;
  }
  if (speed_r >= 1.0) {
    speed_r = 1.0;
  } else if (speed_r <= -1.0) {
    speed_r = -1.0;
  }
}

void handleLowLevelUIEvent(struct ll_ui_event* ui_event) {
  ROS_INFO_STREAM("Got UI button with code:" << +ui_event->button_id << " and duration: " << +ui_event->press_duration);

  auto srv_request = std::make_shared<mower_msgs::srv::HighLevelControlSrv::Request>();

  switch (ui_event->button_id) {
    case 2:
      // Home
      srv_request->command = mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_HOME;
      break;
    case 3:
      // Play
      srv_request->command = mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_START;
      break;
    case 4:
      // S1
      srv_request->command = mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_S1;
      break;
    case 5:
      // S2
      if (ui_event->press_duration == 2) {
        srv_request->command = mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_DELETE_MAPS;
      } else {
        srv_request->command = mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_S2;
      }
      break;
    case 6:
      // LOCK
      if (ui_event->press_duration == 2) {
        // very long press on lock
        srv_request->command = mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_RESET_EMERGENCY;
      }
      break;
    default:
      // Return, don't call the service.
      return;
  }

  // Async service call (non-blocking)
  auto result_future = highLevelClient->async_send_request(srv_request,
      [](rclcpp::Client<mower_msgs::srv::HighLevelControlSrv>::SharedFuture future) {
        (void)future;  // We don't use the result
      });
}

/**
 * @brief getNewSetChanged return t_new and checks if the value changed in comparison to t_cur.
 * t_new can't be a reference because the same function is also used for packed structures.
 * @param t_cur source value
 * @param t_new reference
 * @return &bool get set to true if t_cur and t_new differ, otherwise changed doesn't get touched
 */
template <typename T>
T getNewSetChanged(const T t_cur, const T t_new, bool& changed) {
  bool equal;
  if (std::is_floating_point<T>::value)
    equal = fabs(t_cur - t_new) < std::numeric_limits<T>::epsilon();
  else
    equal = t_cur == t_new;

  if (!equal) changed = true;

  return t_new;
}

/**
 * Handle config packet on receive from LL (LL->HL config packet response)
 */
void handleLowLevelConfig(const uint8_t* buffer, const size_t size) {
  // This is a flexible length packet where the size may vary when ll_high_level_config struct got enhanced only on one
  // side. If payload size is larger than our struct size, ensure that we only copy those we know of = our struct size.
  // If payload size is smaller than our struct size, copy only the payload we got, but ensure that the unsent member(s)
  // have reasonable defaults.
  size_t payload_size = std::min(sizeof(ll_high_level_config), size - 3);  // exclude type & crc

  // Copy payload to separated ll_config
  memcpy(&llhl_config, buffer + 1, payload_size);

  // Let's be verbose for easier follow-up
  ROS_INFO(
      "Received ll_high_level_config packet %#04x\n"
      "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
      "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
      "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
      "\t lift_period=%d, tilt_period=%d,\n"
      "\t shutdown_esc_max_pitch=%d,\n"
      "\t language=\"%.2s\", volume=%d\n"
      "\t hall_configs=\"%s\"",
      *buffer, (int)llhl_config.options.dfp_is_5v, (int)llhl_config.options.background_sounds,
      (int)llhl_config.options.ignore_charging_current, llhl_config.v_charge_cutoff, llhl_config.i_charge_cutoff,
      llhl_config.v_battery_cutoff, llhl_config.v_battery_empty, llhl_config.v_battery_full, llhl_config.lift_period,
      llhl_config.tilt_period, llhl_config.shutdown_esc_max_pitch, llhl_config.language, llhl_config.volume,
      getHallConfigsString(llhl_config.hall_configs, MAX_HALL_INPUTS).c_str());

  // Inform config packet tracker about the response
  configTracker.ackResponse();

  // Copy received config values from LL to our local config params and
  // decide if dynamic parameters need to be updated
  bool logic_config_dirty = false;
  bool power_config_dirty = false;
  // clang-format off
  power_config.charge_critical_high_voltage = getNewSetChanged<double>(power_config.charge_critical_high_voltage, (double)llhl_config.v_charge_cutoff, power_config_dirty);
  power_config.charge_critical_high_current = getNewSetChanged<double>(power_config.charge_critical_high_current, (double)llhl_config.i_charge_cutoff, power_config_dirty);
  power_config.battery_critical_high_voltage = getNewSetChanged<double>(power_config.battery_critical_high_voltage, (double)llhl_config.v_battery_cutoff, power_config_dirty);
  power_config.battery_empty_voltage = getNewSetChanged<double>(power_config.battery_empty_voltage, (double)llhl_config.v_battery_empty, power_config_dirty);
  power_config.battery_full_voltage = getNewSetChanged<double>(power_config.battery_full_voltage, (double)llhl_config.v_battery_full, power_config_dirty);
  mower_logic_config.cu_rain_threshold = getNewSetChanged<int>(mower_logic_config.cu_rain_threshold, (int)llhl_config.rain_threshold, logic_config_dirty);
  mower_logic_config.emergency_lift_period = getNewSetChanged<int>(mower_logic_config.emergency_lift_period, (int)llhl_config.lift_period, logic_config_dirty);
  mower_logic_config.emergency_tilt_period = getNewSetChanged<int>(mower_logic_config.emergency_tilt_period, (int)llhl_config.tilt_period, logic_config_dirty);
  mower_logic_config.shutdown_esc_max_pitch = getNewSetChanged<int>(mower_logic_config.shutdown_esc_max_pitch, (int)llhl_config.shutdown_esc_max_pitch, logic_config_dirty);
  // clang-format on

  // In ROS2, update the node parameters so they can be read by other nodes if needed
  if (logic_config_dirty) {
    g_node->set_parameter(rclcpp::Parameter("mower_logic.cu_rain_threshold", mower_logic_config.cu_rain_threshold));
    g_node->set_parameter(rclcpp::Parameter("mower_logic.emergency_lift_period", mower_logic_config.emergency_lift_period));
    g_node->set_parameter(rclcpp::Parameter("mower_logic.emergency_tilt_period", mower_logic_config.emergency_tilt_period));
    g_node->set_parameter(rclcpp::Parameter("mower_logic.shutdown_esc_max_pitch", mower_logic_config.shutdown_esc_max_pitch));
  }
  if (power_config_dirty) {
    g_node->set_parameter(rclcpp::Parameter("services.power.charge_critical_high_voltage", power_config.charge_critical_high_voltage));
    g_node->set_parameter(rclcpp::Parameter("services.power.charge_critical_high_current", power_config.charge_critical_high_current));
    g_node->set_parameter(rclcpp::Parameter("services.power.battery_critical_high_voltage", power_config.battery_critical_high_voltage));
    g_node->set_parameter(rclcpp::Parameter("services.power.battery_empty_voltage", power_config.battery_empty_voltage));
    g_node->set_parameter(rclcpp::Parameter("services.power.battery_full_voltage", power_config.battery_full_voltage));
  }
}

void handleLowLevelStatus(struct ll_status* status) {
  static rclcpp::Time last_ll_status_update;

  std::unique_lock<std::mutex> lk(ll_status_mutex);
  last_ll_status = *status;

  // LL status get send at 100ms cycle. If we miss 10 packets, we can assume that it got restarted or flashed with a
  // new FW. In either case we should ensure that it has the right config and update/re-align with us.
  auto now = g_node->get_clock()->now();
  if ((now - last_ll_status_update).seconds() > 1.0) configTracker.setDirty();
  last_ll_status_update = now;
}

void handleLowLevelIMU(struct ll_imu* imu) {
  mower_msgs::msg::ImuRaw imu_msg;
  imu_msg.dt = imu->dt_millis;
  imu_msg.ax = imu->acceleration_mss[0];
  imu_msg.ay = imu->acceleration_mss[1];
  imu_msg.az = imu->acceleration_mss[2];
  imu_msg.gx = imu->gyro_rads[0];
  imu_msg.gy = imu->gyro_rads[1];
  imu_msg.gz = imu->gyro_rads[2];
  imu_msg.mx = imu->mag_uT[0];
  imu_msg.my = imu->mag_uT[1];
  imu_msg.mz = imu->mag_uT[2];

  sensor_imu_msg.header.stamp = g_node->get_clock()->now();
  // ROS2: no header.seq
  sensor_imu_msg.header.frame_id = "base_link";
  sensor_imu_msg.linear_acceleration.x = imu_msg.ax;
  sensor_imu_msg.linear_acceleration.y = imu_msg.ay;
  sensor_imu_msg.linear_acceleration.z = imu_msg.az;
  sensor_imu_msg.angular_velocity.x = imu_msg.gx;
  sensor_imu_msg.angular_velocity.y = imu_msg.gy;
  sensor_imu_msg.angular_velocity.z = imu_msg.gz;

  sensor_imu_pub->publish(sensor_imu_msg);
}

void checkAndSendConfig() {
  // Copy changed mower_config's values to the related llhl_config values and
  // decide if LL need to be informed with a new config packet
  bool dirty = false;

  // clang-format off
  llhl_config.rain_threshold = getNewSetChanged<int>(llhl_config.rain_threshold, mower_logic_config.cu_rain_threshold, dirty);
  llhl_config.v_charge_cutoff = getNewSetChanged<double>(llhl_config.v_charge_cutoff, power_config.charge_critical_high_voltage, dirty);
  llhl_config.i_charge_cutoff = getNewSetChanged<double>(llhl_config.i_charge_cutoff, power_config.charge_critical_high_current, dirty);
  llhl_config.v_battery_cutoff = getNewSetChanged<double>(llhl_config.v_battery_cutoff, power_config.battery_critical_high_voltage, dirty);
  llhl_config.v_battery_empty = getNewSetChanged<double>(llhl_config.v_battery_empty, power_config.battery_empty_voltage, dirty);
  llhl_config.v_battery_full = getNewSetChanged<double>(llhl_config.v_battery_full, power_config.battery_full_voltage, dirty);
  llhl_config.lift_period = getNewSetChanged<int>(llhl_config.lift_period, mower_logic_config.emergency_lift_period, dirty);
  llhl_config.tilt_period = getNewSetChanged<int>(llhl_config.tilt_period, mower_logic_config.emergency_tilt_period, dirty);
  llhl_config.shutdown_esc_max_pitch = getNewSetChanged<int>(llhl_config.shutdown_esc_max_pitch, mower_logic_config.shutdown_esc_max_pitch, dirty);
  // clang-format on

  // Parse emergency_input_config and set hall_configs
  char* token = strtok(strdup(mower_logic_config.emergency_input_config.c_str()), ",");
  bool low_active;
  unsigned int hall_idx = 0;
  while (token != NULL) {
    low_active = false;
    while (*token != 0) {
      switch (std::toupper(*token)) {
        case '!': low_active = true; break;
        case 'I': llhl_config.hall_configs[hall_idx] = {HallMode::OFF, low_active}; break;
        case 'L': llhl_config.hall_configs[hall_idx] = {HallMode::LIFT_TILT, low_active}; break;
        case 'S': llhl_config.hall_configs[hall_idx] = {HallMode::STOP, low_active}; break;
        case 'U': llhl_config.hall_configs[hall_idx] = {HallMode::UNDEFINED, low_active}; break;
        default: break;
      }
      token++;
    }
    token = strtok(NULL, ",");
    hall_idx++;
  }

  if (dirty) configTracker.setDirty();
}

// ROS2 parameter change callback (replaces dynamic_reconfigure)
rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    const auto& name = param.get_name();
    // mower_logic params
    if (name == "mower_logic.cu_rain_threshold") {
      mower_logic_config.cu_rain_threshold = param.as_int();
    } else if (name == "mower_logic.emergency_lift_period") {
      mower_logic_config.emergency_lift_period = param.as_int();
    } else if (name == "mower_logic.emergency_tilt_period") {
      mower_logic_config.emergency_tilt_period = param.as_int();
    } else if (name == "mower_logic.shutdown_esc_max_pitch") {
      mower_logic_config.shutdown_esc_max_pitch = param.as_int();
    } else if (name == "mower_logic.emergency_input_config") {
      mower_logic_config.emergency_input_config = param.as_string();
    }
    // power params
    else if (name == "services.power.charge_critical_high_voltage") {
      power_config.charge_critical_high_voltage = param.as_double();
    } else if (name == "services.power.charge_critical_high_current") {
      power_config.charge_critical_high_current = param.as_double();
    } else if (name == "services.power.battery_critical_high_voltage") {
      power_config.battery_critical_high_voltage = param.as_double();
    } else if (name == "services.power.battery_empty_voltage") {
      power_config.battery_empty_voltage = param.as_double();
    } else if (name == "services.power.battery_full_voltage") {
      power_config.battery_full_voltage = param.as_double();
    }
  }

  checkAndSendConfig();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create a node with allow_undeclared_parameters for sub-nodes (xesc_driver)
  auto node_options = rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  g_node = rclcpp::Node::make_shared("mower_comms_v1", node_options);

  // Initialize timestamps with current clock
  last_cmd_vel = g_node->get_clock()->now();
  last_ticks_stamp = g_node->get_clock()->now();
  configTracker.last_config_req = g_node->get_clock()->now();

  highLevelClient = g_node->create_client<mower_msgs::srv::HighLevelControlSrv>("mower_service/high_level_control");

  // Declare and get parameters (replacing dynamic_reconfigure)
  // mower_logic parameters
  g_node->declare_parameter<int>("mower_logic.cu_rain_threshold", 0xffff);
  g_node->declare_parameter<int>("mower_logic.emergency_lift_period", 0xffff);
  g_node->declare_parameter<int>("mower_logic.emergency_tilt_period", 0xffff);
  g_node->declare_parameter<int>("mower_logic.shutdown_esc_max_pitch", 0xff);
  g_node->declare_parameter<std::string>("mower_logic.emergency_input_config", "");

  mower_logic_config.cu_rain_threshold = g_node->get_parameter("mower_logic.cu_rain_threshold").as_int();
  mower_logic_config.emergency_lift_period = g_node->get_parameter("mower_logic.emergency_lift_period").as_int();
  mower_logic_config.emergency_tilt_period = g_node->get_parameter("mower_logic.emergency_tilt_period").as_int();
  mower_logic_config.shutdown_esc_max_pitch = g_node->get_parameter("mower_logic.shutdown_esc_max_pitch").as_int();
  mower_logic_config.emergency_input_config = g_node->get_parameter("mower_logic.emergency_input_config").as_string();

  // power parameters
  g_node->declare_parameter<double>("services.power.charge_critical_high_voltage", 0.0);
  g_node->declare_parameter<double>("services.power.charge_critical_high_current", 0.0);
  g_node->declare_parameter<double>("services.power.battery_critical_high_voltage", 0.0);
  g_node->declare_parameter<double>("services.power.battery_empty_voltage", 0.0);
  g_node->declare_parameter<double>("services.power.battery_full_voltage", 0.0);

  power_config.charge_critical_high_voltage = g_node->get_parameter("services.power.charge_critical_high_voltage").as_double();
  power_config.charge_critical_high_current = g_node->get_parameter("services.power.charge_critical_high_current").as_double();
  power_config.battery_critical_high_voltage = g_node->get_parameter("services.power.battery_critical_high_voltage").as_double();
  power_config.battery_empty_voltage = g_node->get_parameter("services.power.battery_empty_voltage").as_double();
  power_config.battery_full_voltage = g_node->get_parameter("services.power.battery_full_voltage").as_double();

  // Register parameter change callback (replaces dynamic_reconfigure callback)
  auto param_callback_handle = g_node->add_on_set_parameters_callback(onParameterChange);

  // Serial port parameter
  std::string ll_serial_port_name;
  g_node->declare_parameter<std::string>("ll_serial_port", "");
  ll_serial_port_name = g_node->get_parameter("ll_serial_port").as_string();
  if (ll_serial_port_name.empty()) {
    ROS_ERROR_STREAM("Error getting low level serial port parameter. Quitting.");
    return 1;
  }

  g_node->declare_parameter<double>("services.diff_drive.ticks_per_m", 0.0);
  g_node->declare_parameter<double>("services.diff_drive.wheel_distance_m", 0.0);
  wheel_ticks_per_m = g_node->get_parameter("services.diff_drive.ticks_per_m").as_double();
  wheel_distance_m = g_node->get_parameter("services.diff_drive.wheel_distance_m").as_double();

  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);

  speed_l = speed_r = speed_mow = target_speed_mow = 0;

  // Some generic settings from param server (non-dynamic)
  g_node->declare_parameter<bool>("mower_logic.ignore_charging_current", false);
  g_node->declare_parameter<bool>("services.sound.dfp_is_5v", false);
  g_node->declare_parameter<int>("services.sound.volume", -1);
  g_node->declare_parameter<bool>("services.sound.background_sounds", false);
  g_node->declare_parameter<std::string>("services.sound.language", "en");

  llhl_config.options.ignore_charging_current =
      g_node->get_parameter("mower_logic.ignore_charging_current").as_bool() ? OptionState::ON : OptionState::OFF;
  llhl_config.options.dfp_is_5v = g_node->get_parameter("services.sound.dfp_is_5v").as_bool() ? OptionState::ON : OptionState::OFF;
  llhl_config.volume = g_node->get_parameter("services.sound.volume").as_int();
  llhl_config.options.background_sounds =
      g_node->get_parameter("services.sound.background_sounds").as_bool() ? OptionState::ON : OptionState::OFF;
  // ISO-639-1 (2 char) language code
  strncpy(llhl_config.language, g_node->get_parameter("services.sound.language").as_string().c_str(), 2);

  // Setup XESC interfaces
  // In ROS2, xesc_driver takes a Node::SharedPtr. We create sub-nodes for each ESC
  // so parameters are namespaced correctly.
  g_node->declare_parameter<std::string>("services.diff_drive.mower_xesc.xesc_type", "");
  std::string mower_xesc_type = g_node->get_parameter("services.diff_drive.mower_xesc.xesc_type").as_string();
  if (!mower_xesc_type.empty()) {
    auto mower_sub_node = rclcpp::Node::make_shared("mower_xesc", g_node->get_namespace(), node_options);
    mow_xesc_interface = new xesc_driver::XescDriver(mower_sub_node);
  } else {
    mow_xesc_interface = nullptr;
  }

  auto left_sub_node = rclcpp::Node::make_shared("left_xesc", g_node->get_namespace(), node_options);
  left_xesc_interface = new xesc_driver::XescDriver(left_sub_node);

  auto right_sub_node = rclcpp::Node::make_shared("right_xesc", g_node->get_namespace(), node_options);
  right_xesc_interface = new xesc_driver::XescDriver(right_sub_node);

  emergency_pub = g_node->create_publisher<mower_msgs::msg::Emergency>("ll/emergency", rclcpp::QoS(1));

  // Diff drive service
  actual_twist_pub = g_node->create_publisher<geometry_msgs::msg::TwistStamped>("ll/diff_drive/measured_twist", rclcpp::SensorDataQoS());
  status_left_esc_pub = g_node->create_publisher<mower_msgs::msg::ESCStatus>("ll/diff_drive/left_esc_status", rclcpp::QoS(1));
  status_right_esc_pub = g_node->create_publisher<mower_msgs::msg::ESCStatus>("ll/diff_drive/right_esc_status", rclcpp::QoS(1));

  status_pub = g_node->create_publisher<mower_msgs::msg::Status>("ll/mower_status", rclcpp::QoS(1));
  sensor_imu_pub = g_node->create_publisher<sensor_msgs::msg::Imu>("ll/imu/data_raw", rclcpp::SensorDataQoS());
  power_pub = g_node->create_publisher<mower_msgs::msg::Power>("ll/power", rclcpp::QoS(1));

  auto mow_service = g_node->create_service<mower_msgs::srv::MowerControlSrv>("ll/_service/mow_enabled", setMowEnabled);
  auto emergency_service = g_node->create_service<mower_msgs::srv::EmergencyStopSrv>("ll/_service/emergency", setEmergencyStop);

  auto cmd_vel_sub = g_node->create_subscription<geometry_msgs::msg::Twist>(
      "ll/cmd_vel", rclcpp::QoS(0).best_effort(), velReceived);

  auto high_level_status_sub = g_node->create_subscription<mower_msgs::msg::HighLevelStatus>(
      "/mower_logic/current_state", rclcpp::QoS(0).best_effort(), highLevelStatusReceived);

  auto publish_timer = g_node->create_wall_timer(20ms, publishActuatorsTimerTask);

  size_t buflen = 1000;
  uint8_t buffer[buflen];
  uint8_t buffer_decoded[buflen];
  size_t read = 0;
  // don't change, we need to wait for arduino to boot before actually sending stuff
  rclcpp::Rate retry_rate(0.2);  // 5 second period

  // Use a MultiThreadedExecutor to allow the serial loop to run alongside callbacks
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(g_node);

  // Spin in a background thread
  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  while (rclcpp::ok()) {
    if (!serial_port.isOpen()) {
      ROS_INFO_STREAM("connecting serial interface: " << ll_serial_port_name);
      allow_send = false;
      try {
        serial_port.setPort(ll_serial_port_name);
        serial_port.setBaudrate(115200);
        auto to = serial::Timeout::simpleTimeout(100);
        serial_port.setTimeout(to);
        serial_port.open();

        // wait for controller to boot
        retry_rate.sleep();
        // this will only be set if no error was set

        allow_send = true;
      } catch (std::exception& e) {
        retry_rate.sleep();
        ROS_ERROR_STREAM("Error during reconnect.");
      }
    }
    size_t bytes_read = 0;
    try {
      bytes_read = serial_port.read(buffer + read, 1);
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Error reading serial_port. Closing Connection.");
      serial_port.close();
      retry_rate.sleep();
    }
    if (read + bytes_read >= buflen) {
      read = 0;
      bytes_read = 0;
      ROS_ERROR_STREAM("Prevented buffer overflow. There is a problem with the serial comms.");
    }
    if (bytes_read) {
      if (buffer[read] == 0) {
        // end of packet found
        size_t data_size = cobs.decode(buffer, read, buffer_decoded);

        // first, check the CRC
        if (data_size < 3) {
          // We don't even have one byte of data
          // (type + crc = 3 bytes already)
          ROS_INFO_STREAM("Got empty packet from Low Level Board");
        } else {
          // We have at least 1 byte of data, check the CRC
          crc.reset();
          // We start at the second byte (ignore the type) and process (data_size- byte for type - 2 bytes for CRC)
          // bytes.
          crc.process_bytes(buffer_decoded, data_size - 2);
          uint16_t checksum = crc.checksum();
          uint16_t received_checksum = *(uint16_t*)(buffer_decoded + data_size - 2);
          if (checksum == received_checksum) {
            // Packet checksum is OK, process it
            switch (buffer_decoded[0]) {
              case PACKET_ID_LL_STATUS:
                if (data_size == sizeof(struct ll_status)) {
                  handleLowLevelStatus((struct ll_status*)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was STATUS");
                }
                break;
              case PACKET_ID_LL_IMU:
                if (data_size == sizeof(struct ll_imu)) {
                  handleLowLevelIMU((struct ll_imu*)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was IMU");
                }
                break;
              case PACKET_ID_LL_UI_EVENT:
                if (data_size == sizeof(struct ll_ui_event)) {
                  handleLowLevelUIEvent((struct ll_ui_event*)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was UI_EVENT");
                }
                break;
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ:
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP: handleLowLevelConfig(buffer_decoded, data_size); break;
              default: ROS_INFO_STREAM("Got unknown packet from Low Level Board"); break;
            }
          } else {
            ROS_INFO_STREAM("Got invalid checksum from Low Level Board");
          }
        }

        read = 0;
      } else {
        read += bytes_read;
      }
    }
  }

  // Stop executor
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  if (mow_xesc_interface) {
    mow_xesc_interface->setDutyCycle(0.0);
    mow_xesc_interface->stop();
  }
  left_xesc_interface->setDutyCycle(0.0);
  right_xesc_interface->setDutyCycle(0.0);
  left_xesc_interface->stop();
  right_xesc_interface->stop();

  if (mow_xesc_interface) {
    delete mow_xesc_interface;
  }
  delete left_xesc_interface;
  delete right_xesc_interface;

  rclcpp::shutdown();
  return 0;
}
