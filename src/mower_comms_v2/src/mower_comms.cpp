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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mower_msgs/msg/esc_status.hpp>
#include <mower_msgs/msg/emergency.hpp>
#include <mower_msgs/msg/status.hpp>
#include <mower_msgs/msg/power.hpp>
#include <mower_msgs/srv/emergency_stop_srv.hpp>
#include <mower_msgs/srv/high_level_control_srv.hpp>
#include <mower_msgs/srv/mower_control_srv.hpp>
#include <nmea_msgs/msg/sentence.hpp>
#include <rtcm_msgs/msg/message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>

#include "../../../services/service_ids.h"
#include "DiffDriveServiceInterface.h"
#include "EmergencyServiceInterface.h"
#include "GpsServiceInterface.h"
#include "ImuServiceInterface.h"
#include "MowerServiceInterface.h"
#include "PowerServiceInterface.h"

using namespace std::chrono_literals;

std::unique_ptr<EmergencyServiceInterface> emergency_service = nullptr;
std::unique_ptr<DiffDriveServiceInterface> diff_drive_service = nullptr;
std::unique_ptr<MowerServiceInterface> mower_service = nullptr;
std::unique_ptr<ImuServiceInterface> imu_service = nullptr;
std::unique_ptr<PowerServiceInterface> power_service = nullptr;
std::unique_ptr<GpsServiceInterface> gps_service = nullptr;

xbot::serviceif::Context ctx{};

static rclcpp::Logger g_logger = rclcpp::get_logger("mower_comms_v2");

static void spdlog_cb(const spdlog::details::log_msg& msg) {
  std::string s(msg.payload.begin(), msg.payload.end());
  switch (msg.level) {
    case spdlog::level::trace:
    case spdlog::level::debug: RCLCPP_DEBUG(g_logger, "%s", s.c_str()); break;
    case spdlog::level::info:  RCLCPP_INFO(g_logger, "%s", s.c_str()); break;
    case spdlog::level::warn:  RCLCPP_WARN(g_logger, "%s", s.c_str()); break;
    case spdlog::level::err:   RCLCPP_ERROR(g_logger, "%s", s.c_str()); break;
    case spdlog::level::critical: RCLCPP_FATAL(g_logger, "%s", s.c_str()); break;
    case spdlog::level::off: default: break;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mower_comms_v2");

  {
    auto sink = std::make_shared<spdlog::sinks::callback_sink_mt>(spdlog_cb);
    auto logger = std::make_shared<spdlog::logger>("", std::move(sink));
    spdlog::set_default_logger(logger);
  }

  // ---------- Parameters (ROS1 had /ll namespace; we use ll. prefix) ----------
  node->declare_parameter<std::string>("ll.bind_ip", "0.0.0.0");
  std::string bind_ip = node->get_parameter("ll.bind_ip").as_string();
  RCLCPP_INFO(node->get_logger(), "Bind IP (Robot Internal): %s", bind_ip.c_str());

  // High level control service client
  auto highLevelClient = node->create_client<mower_msgs::srv::HighLevelControlSrv>("mower_service/high_level_control");

  // Services
  auto srv_mow_enabled = node->create_service<mower_msgs::srv::MowerControlSrv>(
      "ll/_service/mow_enabled",
      [](const std::shared_ptr<mower_msgs::srv::MowerControlSrv::Request> req,
         std::shared_ptr<mower_msgs::srv::MowerControlSrv::Response> /*res*/) {
        if (mower_service) mower_service->SetMowerEnabled(req->mow_enabled);
      });

  auto srv_emergency = node->create_service<mower_msgs::srv::EmergencyStopSrv>(
      "ll/_service/emergency",
      [](const std::shared_ptr<mower_msgs::srv::EmergencyStopSrv::Request> req,
         std::shared_ptr<mower_msgs::srv::EmergencyStopSrv::Response> /*res*/) {
        if (emergency_service) emergency_service->SetEmergency(req->emergency);
      });

  // Subscriptions
  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "ll/cmd_vel", rclcpp::QoS(10),
      [](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (diff_drive_service) diff_drive_service->SendTwist(msg);
      });

  auto rtcm_sub = node->create_subscription<rtcm_msgs::msg::Message>(
      "ll/position/gps/rtcm", rclcpp::QoS(100),
      [node](const rtcm_msgs::msg::Message::SharedPtr msg) {
        if (!gps_service) return;
        static std::vector<uint8_t> rtcm_buffer{};
        static rclcpp::Time last_time_sent(0, 0, node->get_clock()->get_clock_type());
        const auto now = node->get_clock()->now();
        // Append the bytes to the buffer
        rtcm_buffer.insert(rtcm_buffer.end(), msg->message.begin(), msg->message.end());
        // In order to not spam after each received byte, limit packets to 5Hz and to max 1k of size
        if (rtcm_buffer.size() < 1000 && (now - last_time_sent).seconds() < 0.2) return;
        last_time_sent = now;
        gps_service->SendRTCM(rtcm_buffer.data(), rtcm_buffer.size());
        rtcm_buffer.clear();
      });

  // Timers
  auto t_heartbeat = node->create_wall_timer(500ms, []() {
    if (emergency_service) emergency_service->Heartbeat();
  });

  auto t_mower_tick = node->create_wall_timer(5s, []() {
    if (mower_service) mower_service->Tick();
  });

  ctx = xbot::serviceif::Start(true, bind_ip);

  // Emergency service
  auto emergency_pub = node->create_publisher<mower_msgs::msg::Emergency>("ll/emergency", rclcpp::QoS(1));
  emergency_service = std::make_unique<EmergencyServiceInterface>(xbot::service_ids::EMERGENCY, ctx, emergency_pub, node);
  emergency_service->Start();

  // Diff drive service
  auto actual_twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("ll/diff_drive/measured_twist", rclcpp::QoS(1));
  auto status_left_esc_pub = node->create_publisher<mower_msgs::msg::ESCStatus>("ll/diff_drive/left_esc_status", rclcpp::QoS(1));
  auto status_right_esc_pub = node->create_publisher<mower_msgs::msg::ESCStatus>("ll/diff_drive/right_esc_status", rclcpp::QoS(1));

  node->declare_parameter<double>("ll.services.diff_drive.ticks_per_m", 0.0);
  node->declare_parameter<double>("ll.services.diff_drive.wheel_distance_m", 0.0);
  double wheel_ticks_per_m = node->get_parameter("ll.services.diff_drive.ticks_per_m").as_double();
  double wheel_distance_m = node->get_parameter("ll.services.diff_drive.wheel_distance_m").as_double();
  if (wheel_ticks_per_m == 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Need to provide param ll.services.diff_drive.ticks_per_m");
    return 1;
  }
  if (wheel_distance_m == 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Need to provide param ll.services.diff_drive.wheel_distance_m");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Wheel ticks [1/m]: %f", wheel_ticks_per_m);
  RCLCPP_INFO(node->get_logger(), "Wheel distance [m]: %f", wheel_distance_m);

  node->declare_parameter<int>("ll.services.gps.baud_rate", 0);
  node->declare_parameter<std::string>("ll.services.gps.protocol", "");
  node->declare_parameter<int>("ll.services.gps.port_index", 0);
  int baud_rate = node->get_parameter("ll.services.gps.baud_rate").as_int();
  std::string protocol = node->get_parameter("ll.services.gps.protocol").as_string();
  int gps_port_index = node->get_parameter("ll.services.gps.port_index").as_int();

  if (baud_rate == 0 || protocol.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Need to specify GPS protocol and baud rate!");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "GPS protocol: %s, baud rate: %d, gps port index: %d",
              protocol.c_str(), baud_rate, gps_port_index);

  diff_drive_service = std::make_unique<DiffDriveServiceInterface>(
      xbot::service_ids::DIFF_DRIVE, ctx, node,
      actual_twist_pub, status_left_esc_pub, status_right_esc_pub,
      wheel_ticks_per_m, wheel_distance_m);
  diff_drive_service->Start();

  // Mower service
  auto status_pub = node->create_publisher<mower_msgs::msg::Status>("ll/mower_status", rclcpp::QoS(1));
  mower_service = std::make_unique<MowerServiceInterface>(xbot::service_ids::MOWER, ctx, node, status_pub);
  mower_service->Start();

  // IMU service
  node->declare_parameter<std::string>("ll.services.imu.axis_config", "");
  std::string imu_axis_config = node->get_parameter("ll.services.imu.axis_config").as_string();
  RCLCPP_INFO(node->get_logger(), "IMU axis config: %s", imu_axis_config.c_str());
  auto sensor_imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("ll/imu/data_raw", rclcpp::QoS(1));
  imu_service = std::make_unique<ImuServiceInterface>(xbot::service_ids::IMU, ctx, sensor_imu_pub, imu_axis_config, node);
  imu_service->Start();

  // Power service
  auto power_pub = node->create_publisher<mower_msgs::msg::Power>("ll/power", rclcpp::QoS(1));
  node->declare_parameter<double>("ll.services.power.battery_full_voltage", 0.0);
  node->declare_parameter<double>("ll.services.power.battery_empty_voltage", 0.0);
  node->declare_parameter<double>("ll.services.power.battery_critical_voltage", 0.0);
  node->declare_parameter<double>("ll.services.power.battery_critical_high_voltage", 0.0);
  node->declare_parameter<double>("ll.services.power.charge_current", -1.0);
  float battery_full_voltage = static_cast<float>(node->get_parameter("ll.services.power.battery_full_voltage").as_double());
  float battery_empty_voltage = static_cast<float>(node->get_parameter("ll.services.power.battery_empty_voltage").as_double());
  float battery_critical_voltage = static_cast<float>(node->get_parameter("ll.services.power.battery_critical_voltage").as_double());
  float battery_critical_high_voltage = static_cast<float>(node->get_parameter("ll.services.power.battery_critical_high_voltage").as_double());
  float charge_current = static_cast<float>(node->get_parameter("ll.services.power.charge_current").as_double());
  if (battery_full_voltage == 0.0f) {
    RCLCPP_ERROR(node->get_logger(), "Need to set param: ll.services.power.battery_full_voltage");
    return 1;
  }
  if (battery_empty_voltage == 0.0f) {
    RCLCPP_ERROR(node->get_logger(), "Need to set param: ll.services.power.battery_empty_voltage");
    return 1;
  }
  if (battery_critical_voltage == 0.0f) {
    RCLCPP_ERROR(node->get_logger(), "Need to set param: ll.services.power.battery_critical_voltage");
    return 1;
  }
  if (battery_critical_high_voltage == 0.0f) {
    RCLCPP_ERROR(node->get_logger(), "Need to set param: ll.services.power.battery_critical_high_voltage");
    return 1;
  }
  power_service = std::make_unique<PowerServiceInterface>(
      xbot::service_ids::POWER, ctx, power_pub, battery_full_voltage, battery_empty_voltage, battery_critical_voltage,
      battery_critical_high_voltage, charge_current, node);
  power_service->Start();

  // GPS service
  node->declare_parameter<double>("ll.services.gps.datum_lat", 0.0);
  node->declare_parameter<double>("ll.services.gps.datum_long", 0.0);
  node->declare_parameter<double>("ll.services.gps.datum_height", 0.0);
  node->declare_parameter<bool>("ll.services.gps.absolute_coords", true);
  double datum_lat = node->get_parameter("ll.services.gps.datum_lat").as_double();
  double datum_long = node->get_parameter("ll.services.gps.datum_long").as_double();
  double datum_height = node->get_parameter("ll.services.gps.datum_height").as_double();
  bool has_datum = (datum_lat != 0.0) && (datum_long != 0.0) && (datum_height != 0.0);
  if (!has_datum) {
    RCLCPP_ERROR(node->get_logger(), "You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
    return 2;
  }
  RCLCPP_INFO(node->get_logger(), "Datum: %.7f, %.7f, %.3f", datum_lat, datum_long, datum_height);
  auto gps_position_pub = node->create_publisher<xbot_msgs::msg::AbsolutePose>("ll/position/gps", rclcpp::QoS(1));
  auto nmea_pub = node->create_publisher<nmea_msgs::msg::Sentence>("ll/position/gps/nmea", rclcpp::QoS(1));
  bool absolute_coords = node->get_parameter("ll.services.gps.absolute_coords").as_bool();
  gps_service = std::make_unique<GpsServiceInterface>(
      xbot::service_ids::GPS, ctx, gps_position_pub, nmea_pub,
      datum_lat, datum_long, datum_height,
      static_cast<uint32_t>(baud_rate), protocol, static_cast<uint8_t>(gps_port_index),
      absolute_coords, node);
  gps_service->Start();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
