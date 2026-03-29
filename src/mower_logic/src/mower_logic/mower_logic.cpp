// Created by Clemens Elflein on 2/21/22.
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

// #define VERBOSE_DEBUG   1

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "mower_logic/PowerConfig.h"
#include <mower_msgs/msg/esc_status.hpp>
#include <mower_msgs/msg/emergency.hpp>
#include <mower_msgs/msg/power.hpp>
#include <tf2/LinearMath/Transform.h>

#include <atomic>
#include <ios>
#include <mutex>
#include <sstream>

#include "StateSubscriber.h"
#include "behaviors/AreaRecordingBehavior.h"
#include "behaviors/Behavior.h"
#include "behaviors/IdleBehavior.h"
#include "ftc_local_planner/srv/planner_get_progress.hpp"
#include "mbf_msgs/action/exe_path.hpp"
#include "mbf_msgs/action/move_base.hpp"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_map/srv/clear_map_srv.hpp"
#include "mower_map/srv/clear_nav_point_srv.hpp"
#include "mower_map/srv/get_docking_point_srv.hpp"
#include "mower_map/srv/get_mowing_area_srv.hpp"
#include "mower_map/srv/set_nav_point_srv.hpp"
#include "mower_msgs/srv/emergency_stop_srv.hpp"
#include "mower_msgs/srv/high_level_control_srv.hpp"
#include "mower_msgs/msg/high_level_status.hpp"
#include "mower_msgs/srv/mower_control_srv.hpp"
#include "mower_msgs/msg/status.hpp"
#include "slic3r_coverage_planner/srv/plan_path.hpp"
#include "std_msgs/msg/string.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include "xbot_msgs/srv/register_actions_srv.hpp"
#include "xbot_positioning/srv/gps_control_srv.hpp"
#include "xbot_positioning/srv/set_pose_srv.hpp"

// Global node pointer used by all behaviors
rclcpp::Node::SharedPtr rosNode;

rclcpp::Client<slic3r_coverage_planner::srv::PlanPath>::SharedPtr pathClient;
rclcpp::Client<mower_map::srv::GetMowingAreaSrv>::SharedPtr mapClient;
rclcpp::Client<mower_map::srv::GetDockingPointSrv>::SharedPtr dockingPointClient;
rclcpp::Client<xbot_positioning::srv::GPSControlSrv>::SharedPtr gpsClient;
rclcpp::Client<mower_msgs::srv::MowerControlSrv>::SharedPtr mowClient;
rclcpp::Client<mower_msgs::srv::EmergencyStopSrv>::SharedPtr emergencyClient;
rclcpp::Client<ftc_local_planner::srv::PlannerGetProgress>::SharedPtr pathProgressClient;
rclcpp::Client<mower_map::srv::SetNavPointSrv>::SharedPtr setNavPointClient;
rclcpp::Client<mower_map::srv::ClearNavPointSrv>::SharedPtr clearNavPointClient;
rclcpp::Client<mower_map::srv::ClearMapSrv>::SharedPtr clearMapClient;
rclcpp::Client<xbot_positioning::srv::SetPoseSrv>::SharedPtr positioningClient;
rclcpp::Client<xbot_msgs::srv::RegisterActionsSrv>::SharedPtr actionRegistrationClient;

rclcpp_action::Client<mbf_msgs::action::MoveBase>::SharedPtr mbfClient;
rclcpp_action::Client<mbf_msgs::action::ExePath>::SharedPtr mbfClientExePath;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
rclcpp::Publisher<mower_msgs::msg::HighLevelStatus>::SharedPtr high_level_state_publisher;
mower_logic::MowerLogicConfig last_config;
ll::PowerConfig last_power_config;

StateSubscriber<mower_msgs::msg::Emergency> emergency_state_subscriber{"/ll/emergency"};
StateSubscriber<mower_msgs::msg::Status> status_state_subscriber{"/ll/mower_status"};
StateSubscriber<mower_msgs::msg::Power> power_state_subscriber{"/ll/power"};
StateSubscriber<mower_msgs::msg::ESCStatus> left_esc_status_state_subscriber{"/ll/diff_drive/left_esc_status"};
StateSubscriber<mower_msgs::msg::ESCStatus> right_esc_status_state_subscriber{"/ll/diff_drive/right_esc_status"};
StateSubscriber<xbot_msgs::msg::AbsolutePose> pose_state_subscriber{"/xbot_positioning/xb_pose"};
rclcpp::Time joy_vel_time(0, 0, RCL_ROS_TIME);

rclcpp::Time last_good_gps(0, 0, RCL_ROS_TIME);

std::recursive_mutex mower_logic_mutex;

mower_msgs::msg::HighLevelStatus high_level_status;

std::atomic<bool> mowerAllowed;

Behavior* currentBehavior = &IdleBehavior::INSTANCE;

std::vector<xbot_msgs::msg::ActionInfo> rootActions;
rclcpp::Time last_v_battery_check(0, 0, RCL_ROS_TIME);
double max_v_battery_seen = 0.0;

rclcpp::Time last_rain_check(0, 0, RCL_ROS_TIME);
bool rain_detected = true;
rclcpp::Time rain_resume(0, 0, RCL_ROS_TIME);

// Parameter change callback handle
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle;

/**
 * Some thread safe methods to get a copy of the logic state
 */
rclcpp::Time getLastGoodGPS() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_good_gps;
}

void setLastGoodGPS(rclcpp::Time time) {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  last_good_gps = time;
}

mower_logic::MowerLogicConfig getConfig() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_config;
}

ll::PowerConfig getPowerConfig() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  return last_power_config;
}

void setConfig(mower_logic::MowerLogicConfig c) {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  last_config = c;
  c.toNode(rosNode);
}

mower_msgs::msg::Status getStatus() {
  return status_state_subscriber.getMessage();
}

mower_msgs::msg::Power getPower() {
  return power_state_subscriber.getMessage();
}

xbot_msgs::msg::AbsolutePose getPose() {
  return pose_state_subscriber.getMessage();
}

void setEmergencyMode(bool emergency);

void registerActions(std::string prefix, const std::vector<xbot_msgs::msg::ActionInfo>& actions) {
  auto req = std::make_shared<xbot_msgs::srv::RegisterActionsSrv::Request>();
  req->node_prefix = prefix;
  req->actions = actions;

  rclcpp::Rate retry_delay(1);
  for (int i = 0; i < 10; i++) {
    auto result = actionRegistrationClient->async_send_request(req);
    if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(2)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rosNode->get_logger(), "successfully registered actions for %s", prefix.c_str());
      break;
    }
    RCLCPP_ERROR(rosNode->get_logger(), "Error registering actions for %s. Retrying.", prefix.c_str());
    retry_delay.sleep();
  }
}

void setRobotPose(geometry_msgs::msg::Pose& pose) {
  auto last_pose = pose_state_subscriber.getMessage();
  last_pose.pose.pose = pose;
  pose_state_subscriber.setMessage(last_pose);

  auto req = std::make_shared<xbot_positioning::srv::SetPoseSrv::Request>();
  req->robot_pose = pose;

  rclcpp::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    auto result = positioningClient->async_send_request(req);
    if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(2)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      success = true;
      break;
    }
    RCLCPP_ERROR(rosNode->get_logger(), "Error setting robot pose. Retrying.");
    retry_delay.sleep();
  }

  if (!success) {
    RCLCPP_ERROR(rosNode->get_logger(), "Error setting robot pose. Going to emergency. THIS SHOULD NEVER HAPPEN");
    setEmergencyMode(true);
  }
}

// Abort the currently running behaviour
void abortExecution() {
  if (currentBehavior != nullptr) {
    currentBehavior->abort();
  }
}

bool setGPS(bool enabled) {
  auto req = std::make_shared<xbot_positioning::srv::GPSControlSrv::Request>();
  req->gps_enabled = enabled;

  rclcpp::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    auto result = gpsClient->async_send_request(req);
    if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(2)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rosNode->get_logger(), "successfully set GPS to %s", enabled ? "true" : "false");
      success = true;
      break;
    }
    RCLCPP_ERROR(rosNode->get_logger(), "Error setting GPS to %s. Retrying.", enabled ? "true" : "false");
    retry_delay.sleep();
  }

  if (!success) {
    RCLCPP_ERROR(rosNode->get_logger(), "Error setting GPS. Going to emergency. THIS SHOULD NEVER HAPPEN");
    setEmergencyMode(true);
  }

  return success;
}

bool setMowerEnabled(bool enabled) {
  const auto last_config = getConfig();

  if (!last_config.enable_mower && enabled) {
    enabled = false;
  }

  const auto last_status = status_state_subscriber.getMessage();
  if (last_status.mow_enabled != enabled) {
    rclcpp::Time started = rosNode->get_clock()->now();
    auto req = std::make_shared<mower_msgs::srv::MowerControlSrv::Request>();
    req->mow_enabled = enabled;
    req->mow_direction = static_cast<int>(started.seconds()) & 0x1;
    RCLCPP_WARN(rosNode->get_logger(), "#### om_mower_logic: setMowerEnabled(%s, %u) call",
                enabled ? "true" : "false", req->mow_direction);

    rclcpp::Rate retry_delay(1);
    bool success = false;
    for (int i = 0; i < 10; i++) {
      auto result = mowClient->async_send_request(req);
      if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(2)) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rosNode->get_logger(), "successfully set mower enabled to %s (direction %u)",
                    enabled ? "true" : "false", req->mow_direction);
        success = true;
        break;
      }
      RCLCPP_ERROR(rosNode->get_logger(), "Error setting mower enabled to %s. Retrying.", enabled ? "true" : "false");
      retry_delay.sleep();
    }

    if (!success) {
      RCLCPP_ERROR(rosNode->get_logger(), "Error setting mower enabled. THIS SHOULD NEVER HAPPEN");
    }

    RCLCPP_WARN(rosNode->get_logger(), "#### om_mower_logic: setMowerEnabled(%s, %u) call completed within %fs",
                enabled ? "true" : "false", req->mow_direction,
                (rosNode->get_clock()->now() - started).seconds());
  }

  return true;
}

void stopMoving() {
  geometry_msgs::msg::Twist stop;
  stop.angular.z = 0;
  stop.linear.x = 0;
  cmd_vel_pub->publish(stop);
}

void stopBlade() {
  setMowerEnabled(false);
  mowerAllowed = false;
}

void setEmergencyMode(bool emergency) {
  stopBlade();
  stopMoving();
  auto req = std::make_shared<mower_msgs::srv::EmergencyStopSrv::Request>();
  req->emergency = emergency;

  rclcpp::Rate retry_delay(1);
  bool success = false;
  for (int i = 0; i < 10; i++) {
    auto result = emergencyClient->async_send_request(req);
    if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(2)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rosNode->get_logger(), "successfully set emergency enabled to %s", emergency ? "true" : "false");
      success = true;
      break;
    }
    RCLCPP_ERROR(rosNode->get_logger(), "Error setting emergency enabled to %s. Retrying.", emergency ? "true" : "false");
    retry_delay.sleep();
  }

  if (!success) {
    RCLCPP_ERROR(rosNode->get_logger(), "Error setting emergency. THIS SHOULD NEVER HAPPEN");
  }
}

void updateUI() {
  if (currentBehavior == &MowingBehavior::INSTANCE) {
    try {
      high_level_status.current_area = MowingBehavior::INSTANCE.get_current_area();
    } catch (const std::runtime_error& re) {
      RCLCPP_ERROR(rosNode->get_logger(), "Error getting current area: %s", re.what());
    }
    try {
      high_level_status.current_path = MowingBehavior::INSTANCE.get_current_path();
    } catch (const std::runtime_error& re) {
      RCLCPP_ERROR(rosNode->get_logger(), "Error getting current path: %s", re.what());
    }
    try {
      high_level_status.current_path_index = MowingBehavior::INSTANCE.get_current_path_index();
    } catch (const std::runtime_error& re) {
      RCLCPP_ERROR(rosNode->get_logger(), "Error getting current path index: %s", re.what());
    }
  } else {
    high_level_status.current_area = -1;
    high_level_status.current_path = -1;
    high_level_status.current_path_index = -1;
  }

  if (currentBehavior) {
    high_level_status.state_name = currentBehavior->state_name();
    high_level_status.state = (currentBehavior->get_state() & 0b11111) |
                              (currentBehavior->get_sub_state() << mower_msgs::msg::HighLevelStatus::SUBSTATE_SHIFT);
    high_level_status.sub_state_name = currentBehavior->sub_state_name();
  } else {
    high_level_status.state_name = "NULL";
    high_level_status.sub_state_name = "";
    high_level_status.state = mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
  }
  high_level_state_publisher->publish(high_level_status);
}

bool isGpsGood() {
  std::lock_guard<std::recursive_mutex> lk{mower_logic_mutex};
  const auto last_pose = pose_state_subscriber.getMessage();
  return last_pose.orientation_valid && last_pose.position_accuracy < last_config.max_position_accuracy &&
         (last_pose.flags & xbot_msgs::msg::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE);
}

void checkSafety() {
  const auto last_status = status_state_subscriber.getMessage();
  const auto last_emergency = emergency_state_subscriber.getMessage();
  const auto last_config = getConfig();
  const auto last_pose = pose_state_subscriber.getMessage();
  const auto last_power = power_state_subscriber.getMessage();
  const auto last_left_esc_state = left_esc_status_state_subscriber.getMessage();
  const auto last_left_esc_state_time = left_esc_status_state_subscriber.getMessageTime();
  const auto last_right_esc_state = right_esc_status_state_subscriber.getMessage();
  const auto last_right_esc_state_time = right_esc_status_state_subscriber.getMessageTime();
  const auto pose_time = pose_state_subscriber.getMessageTime();
  const auto status_time = status_state_subscriber.getMessageTime();
  const auto power_time = power_state_subscriber.getMessageTime();
  const auto last_good_gps = getLastGoodGPS();

  high_level_status.emergency = last_emergency.latched_emergency;
  high_level_status.is_charging = last_power.v_charge > 10.0;

  mowerAllowed = true;

  // send to idle if emergency and we're not recording
  if (currentBehavior != nullptr) {
    if (last_emergency.latched_emergency) {
      currentBehavior->requestPause(pauseType::PAUSE_EMERGENCY);
      if (currentBehavior == &AreaRecordingBehavior::INSTANCE || currentBehavior == &IdleBehavior::INSTANCE ||
          currentBehavior == &IdleBehavior::DOCKED_INSTANCE) {
        if (last_power.v_charge > 10.0) {
          setEmergencyMode(false);
        }
      }
    } else {
      currentBehavior->requestContinue(pauseType::PAUSE_EMERGENCY);
    }
  }

  auto now = rosNode->get_clock()->now();

  if ((now - pose_time).seconds() > 1.0) {
    stopBlade();
    stopMoving();
    static auto last_warn = now;
    if ((now - last_warn).seconds() > 5) {
      RCLCPP_WARN(rosNode->get_logger(), "om_mower_logic: EMERGENCY pose values stopped. dt was: %f",
                  (now - pose_time).seconds());
      last_warn = now;
    }
    return;
  }

  if ((now - status_time).seconds() > 3.0 || (now - power_time).seconds() > 3.0) {
    setEmergencyMode(true);
    static auto last_warn = now;
    if ((now - last_warn).seconds() > 5) {
      RCLCPP_WARN(rosNode->get_logger(), "om_mower_logic: EMERGENCY /mower/status values stopped. dt was: %f",
                  (now - status_time).seconds());
      last_warn = now;
    }
    return;
  }

  if (last_left_esc_state.status <= mower_msgs::msg::ESCStatus::ESC_STATUS_ERROR ||
      last_right_esc_state.status <= mower_msgs::msg::ESCStatus::ESC_STATUS_ERROR) {
    setEmergencyMode(true);
    RCLCPP_ERROR(rosNode->get_logger(), "EMERGENCY: at least one motor control errored. errors left: %d, status right: %d",
                 last_left_esc_state.status, last_right_esc_state.status);
    return;
  }

  bool gpsGoodNow = isGpsGood();
  if (gpsGoodNow || last_config.ignore_gps_errors) {
    setLastGoodGPS(now);
    high_level_status.gps_quality_percent =
        1.0 - fmin(1.0, last_pose.position_accuracy / last_config.max_position_accuracy);
    static auto last_gps_log = now;
    if ((now - last_gps_log).seconds() > 10) {
      RCLCPP_INFO(rosNode->get_logger(), "GPS quality: %f", high_level_status.gps_quality_percent);
      last_gps_log = now;
    }
  } else {
    high_level_status.gps_quality_percent = 0;
    if (last_pose.orientation_valid) {
      high_level_status.gps_quality_percent = -1;
    }
    static auto last_gps_warn = now;
    if ((now - last_gps_warn).seconds() > 1) {
      RCLCPP_WARN(rosNode->get_logger(), "Low quality GPS");
      last_gps_warn = now;
    }
  }

  bool gpsTimeout = (now - last_good_gps).seconds() > last_config.gps_timeout;

  if (gpsTimeout) {
    high_level_status.gps_quality_percent = 0;
    static auto last_timeout_warn = now;
    if ((now - last_timeout_warn).seconds() > 1) {
      RCLCPP_WARN(rosNode->get_logger(), "GPS timeout");
      last_timeout_warn = now;
    }
  }

  if (currentBehavior != nullptr && currentBehavior->needs_gps()) {
    currentBehavior->setGoodGPS(!gpsTimeout);
    if (gpsTimeout) {
      stopBlade();
      stopMoving();
      return;
    }
  }

  if (currentBehavior != nullptr && currentBehavior->redirect_joystick()) {
    if ((now - joy_vel_time).seconds() > 10) {
      stopMoving();
    }
  }

  setMowerEnabled(currentBehavior != nullptr && mowerAllowed && currentBehavior->mower_enabled());

  double battery_percent = (last_power.v_battery - last_power_config.battery_empty_voltage) /
                           (last_power_config.battery_full_voltage - last_power_config.battery_empty_voltage);
  if (battery_percent > 1.0) {
    battery_percent = 1.0;
  } else if (battery_percent < 0.0) {
    battery_percent = 0.0;
  }
  high_level_status.battery_percent = battery_percent;

  bool dockingNeeded = false;

  std::stringstream dockingReason("Docking: ", std::ios_base::ate | std::ios_base::in | std::ios_base::out);

  if (last_config.manual_pause_mowing) {
    dockingReason << "Manual pause";
    dockingNeeded = true;
  }

  if (!dockingNeeded && (last_power.v_battery < last_power_config.battery_critical_voltage)) {
    dockingReason << "Battery voltage min critical: " << last_power.v_battery;
    dockingNeeded = true;
  }

  max_v_battery_seen = std::max<double>(max_v_battery_seen, last_power.v_battery);
  if ((now - last_v_battery_check).seconds() > 20.0) {
    if (!dockingNeeded && (max_v_battery_seen < last_power_config.battery_empty_voltage)) {
      dockingReason << "Battery average voltage low: " << max_v_battery_seen;
      dockingNeeded = true;
    }
    max_v_battery_seen = 0.0;
    last_v_battery_check = now;
  }

  if (!dockingNeeded && last_status.mower_motor_temperature >= last_config.motor_hot_temperature) {
    dockingReason << "Mow motor over temp: " << last_status.mower_motor_temperature;
    dockingNeeded = true;
  }

  rain_detected = rain_detected && last_status.rain_detected;
  if (last_config.rain_check_seconds == 0 ||
      (now - last_rain_check).seconds() > last_config.rain_check_seconds) {
    if (rain_detected) {
      rain_resume = now + rclcpp::Duration(
          static_cast<int32_t>(last_config.rain_check_seconds + last_config.rain_delay_minutes * 60), 0);
    }
    if (!dockingNeeded && rain_detected && last_config.rain_mode) {
      dockingReason << "Rain detected";
      dockingNeeded = true;
      if (last_config.rain_mode == 3) {
        auto new_config = getConfig();
        new_config.manual_pause_mowing = true;
        setConfig(new_config);
      }
    }
    last_rain_check = now;
    rain_detected = true;
  }

  if (dockingNeeded && currentBehavior != &DockingBehavior::INSTANCE &&
      currentBehavior != &UndockingBehavior::RETRY_INSTANCE && currentBehavior != &IdleBehavior::INSTANCE &&
      currentBehavior != &IdleBehavior::DOCKED_INSTANCE) {
    RCLCPP_INFO(rosNode->get_logger(), "%s", dockingReason.str().c_str());
    abortExecution();
  }
}

bool highLevelCommand(const std::shared_ptr<mower_msgs::srv::HighLevelControlSrv::Request> req,
                      std::shared_ptr<mower_msgs::srv::HighLevelControlSrv::Response> res) {
  switch (req->command) {
    case mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_HOME:
      RCLCPP_INFO(rosNode->get_logger(), "COMMAND_HOME");
      if (currentBehavior) {
        currentBehavior->command_home();
      }
      break;
    case mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_START:
      RCLCPP_INFO(rosNode->get_logger(), "COMMAND_START");
      if (currentBehavior) {
        currentBehavior->command_start();
      }
      break;
    case mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_S1:
      RCLCPP_INFO(rosNode->get_logger(), "COMMAND_S1");
      if (currentBehavior) {
        currentBehavior->command_s1();
      }
      break;
    case mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_S2:
      RCLCPP_INFO(rosNode->get_logger(), "COMMAND_S2");
      if (currentBehavior) {
        currentBehavior->command_s2();
      }
      break;
    case mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_DELETE_MAPS: {
      RCLCPP_WARN(rosNode->get_logger(), "COMMAND_DELETE_MAPS");
      if (currentBehavior != &AreaRecordingBehavior::INSTANCE && currentBehavior != &IdleBehavior::INSTANCE &&
          currentBehavior != &IdleBehavior::DOCKED_INSTANCE && currentBehavior != nullptr) {
        RCLCPP_ERROR(rosNode->get_logger(), "Deleting maps is only allowed during IDLE or AreaRecording!");
        return true;
      }
      auto clearReq = std::make_shared<mower_map::srv::ClearMapSrv::Request>();
      clearMapClient->async_send_request(clearReq);

      currentBehavior->abort();
    } break;
    case mower_msgs::srv::HighLevelControlSrv::Request::COMMAND_RESET_EMERGENCY:
      RCLCPP_WARN(rosNode->get_logger(), "COMMAND_RESET_EMERGENCY");
      setEmergencyMode(false);
      break;
  }
  return true;
}

void actionReceived(const std_msgs::msg::String::SharedPtr action) {
  if (action->data == "mower_logic/reset_emergency") {
    RCLCPP_WARN(rosNode->get_logger(), "Got reset emergency action.");
    setEmergencyMode(false);
    return;
  }

  if (currentBehavior) {
    currentBehavior->handle_action(action->data);
  }
}

void joyVelReceived(const geometry_msgs::msg::Twist::SharedPtr joy_vel) {
  joy_vel_time = rosNode->get_clock()->now();
  if (currentBehavior && currentBehavior->redirect_joystick()) {
    cmd_vel_pub->publish(*joy_vel);
  }
}

void buildRootActions() {
  xbot_msgs::msg::ActionInfo reset_emergency_action;
  reset_emergency_action.action_id = "reset_emergency";
  reset_emergency_action.enabled = true;
  reset_emergency_action.action_name = "Reset Emergency";
  rootActions.push_back(reset_emergency_action);
}

int main(int argc, char** argv) {
  buildRootActions();

  rclcpp::init(argc, argv);

  rosNode = std::make_shared<rclcpp::Node>("mower_logic");
  mowerAllowed = false;

  // Declare and load parameters
  mower_logic::MowerLogicConfig::declareParameters(rosNode);
  last_config.fromNode(rosNode);

  // Parameter change callback
  param_cb_handle = rosNode->add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter>& /*params*/) -> rcl_interfaces::msg::SetParametersResult {
        RCLCPP_INFO(rosNode->get_logger(), "om_mower_logic: Setting mower_logic config");
        last_config.fromNode(rosNode);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

  // Load power config from parameters
  ll::PowerConfig::declareParameters(rosNode, "power");
  last_power_config.fromNode(rosNode, "power");

  cmd_vel_pub = rosNode->create_publisher<geometry_msgs::msg::Twist>("/logic_vel", 1);

  high_level_state_publisher =
      rosNode->create_publisher<mower_msgs::msg::HighLevelStatus>("mower_logic/current_state", 100);

  pathClient = rosNode->create_client<slic3r_coverage_planner::srv::PlanPath>("slic3r_coverage_planner/plan_path");
  mapClient = rosNode->create_client<mower_map::srv::GetMowingAreaSrv>("mower_map_service/get_mowing_area");
  clearMapClient = rosNode->create_client<mower_map::srv::ClearMapSrv>("mower_map_service/clear_map");

  gpsClient = rosNode->create_client<xbot_positioning::srv::GPSControlSrv>("xbot_positioning/set_gps_state");
  positioningClient = rosNode->create_client<xbot_positioning::srv::SetPoseSrv>("xbot_positioning/set_robot_pose");
  actionRegistrationClient = rosNode->create_client<xbot_msgs::srv::RegisterActionsSrv>("xbot/register_actions");

  mowClient = rosNode->create_client<mower_msgs::srv::MowerControlSrv>("ll/_service/mow_enabled");
  emergencyClient = rosNode->create_client<mower_msgs::srv::EmergencyStopSrv>("ll/_service/emergency");

  dockingPointClient =
      rosNode->create_client<mower_map::srv::GetDockingPointSrv>("mower_map_service/get_docking_point");

  pathProgressClient =
      rosNode->create_client<ftc_local_planner::srv::PlannerGetProgress>("/move_base_flex/FTCPlanner/planner_get_progress");

  setNavPointClient = rosNode->create_client<mower_map::srv::SetNavPointSrv>("mower_map_service/set_nav_point");
  clearNavPointClient = rosNode->create_client<mower_map::srv::ClearNavPointSrv>("mower_map_service/clear_nav_point");

  mbfClient = rclcpp_action::create_client<mbf_msgs::action::MoveBase>(rosNode, "/move_base_flex/move_base");
  mbfClientExePath = rclcpp_action::create_client<mbf_msgs::action::ExePath>(rosNode, "/move_base_flex/exe_path");

  emergency_state_subscriber.Start(rosNode);
  status_state_subscriber.Start(rosNode);
  power_state_subscriber.Start(rosNode);
  left_esc_status_state_subscriber.Start(rosNode);
  right_esc_status_state_subscriber.Start(rosNode);
  pose_state_subscriber.Start(rosNode);

  auto joy_cmd = rosNode->create_subscription<geometry_msgs::msg::Twist>(
      "/joy_vel", rclcpp::QoS(1).best_effort(), joyVelReceived);
  auto action_sub = rosNode->create_subscription<std_msgs::msg::String>(
      "xbot/action", rclcpp::QoS(1).best_effort(), actionReceived);

  auto high_level_control_srv = rosNode->create_service<mower_msgs::srv::HighLevelControlSrv>(
      "mower_service/high_level_control", highLevelCommand);

  // Spin in a separate thread for callbacks
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(rosNode);
  std::thread spinner([executor]() { executor->spin(); });

  rclcpp::Rate r(1.0);

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for emergency message");
  while (!emergency_state_subscriber.hasMessage()) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    r.sleep();
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for a power message");
  while (!power_state_subscriber.hasMessage()) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    r.sleep();
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for a status message");
  while (!status_state_subscriber.hasMessage()) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    r.sleep();
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for a pose message");
  while (!pose_state_subscriber.hasMessage()) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    r.sleep();
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for left ESC status message");
  while (!left_esc_status_state_subscriber.hasMessage()) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    r.sleep();
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for right ESC status message");
  while (!right_esc_status_state_subscriber.hasMessage()) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      spinner.join();
      return 1;
    }
    r.sleep();
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for emergency service");
  if (!emergencyClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Emergency server not found.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for path server");
  if (!pathClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Path service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for mower service");
  if (!mowClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Mower service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for gps service");
  if (!gpsClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "GPS service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for positioning service");
  if (!positioningClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "positioning service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for map server");
  if (!mapClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Map server service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 2;
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for docking point server");
  if (!dockingPointClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Docking server service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 2;
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for nav point server");
  if (!setNavPointClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Set Nav Point server service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 2;
  }
  RCLCPP_INFO(rosNode->get_logger(), "Waiting for clear nav point server");
  if (!clearNavPointClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Clear Nav Point server service not found.");
    rclcpp::shutdown();
    spinner.join();
    return 2;
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for move base flex");
  if (!mbfClient->wait_for_action_server(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "Move base flex not found.");
    rclcpp::shutdown();
    spinner.join();
    return 3;
  }

  RCLCPP_INFO(rosNode->get_logger(), "Waiting for mowing path progress server");
  if (!pathProgressClient->wait_for_service(std::chrono::seconds(60))) {
    RCLCPP_ERROR(rosNode->get_logger(), "FTCLocalPlanner progress server not found.");
    rclcpp::shutdown();
    spinner.join();
    return 3;
  }

  rclcpp::Time started = rosNode->get_clock()->now();
  while ((rosNode->get_clock()->now() - started).seconds() < 10.0) {
    RCLCPP_INFO(rosNode->get_logger(), "Waiting for an emergency status message");
    r.sleep();
    if (emergency_state_subscriber.getMessage().latched_emergency) {
      RCLCPP_INFO(rosNode->get_logger(), "Got emergency, resetting it");
      setEmergencyMode(false);
      break;
    }
  }

  RCLCPP_INFO(rosNode->get_logger(), "registering actions");
  registerActions("mower_logic", rootActions);

  RCLCPP_INFO(rosNode->get_logger(), "om_mower_logic: Got all servers, we can mow");

  auto now = rosNode->get_clock()->now();
  rain_resume = last_rain_check = last_v_battery_check = now;

  auto safety_timer = rosNode->create_wall_timer(
      std::chrono::milliseconds(500), checkSafety);
  auto ui_timer = rosNode->create_wall_timer(
      std::chrono::seconds(1), updateUI);

  // release emergency if it was set
  setEmergencyMode(false);

  // initialise the shared state object to be passed into the behaviors
  auto shared_state = std::make_shared<sSharedState>();
  shared_state->active_semiautomatic_task = false;

  // Behavior execution loop
  while (rclcpp::ok()) {
    if (currentBehavior != nullptr) {
      currentBehavior->start(last_config, shared_state);
      Behavior* newBehavior = currentBehavior->execute();
      currentBehavior->exit();
      currentBehavior = newBehavior;
    } else {
      high_level_status.state_name = "NULL";
      high_level_status.state = mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_NULL;
      high_level_state_publisher->publish(high_level_status);
      RCLCPP_ERROR(rosNode->get_logger(), "null behavior - emergency mode");
      setEmergencyMode(true);
      rclcpp::Rate r(1.0);
      r.sleep();
    }
  }

  executor->cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
