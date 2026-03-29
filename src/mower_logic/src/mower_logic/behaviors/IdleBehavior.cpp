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
#include "IdleBehavior.h"

#include "mower_logic/PowerConfig.h"
#include <mower_msgs/msg/power.hpp>

#include "PerimeterDocking.h"

extern void stopMoving();
extern void stopBlade();
extern void setEmergencyMode(bool emergency);
extern void setGPS(bool enabled);
extern void setRobotPose(geometry_msgs::msg::Pose& pose);
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::msg::ActionInfo>& actions);
extern rclcpp::Time rain_resume;

extern rclcpp::Client<mower_map::srv::GetDockingPointSrv>::SharedPtr dockingPointClient;
extern mower_msgs::msg::Status getStatus();
extern mower_msgs::msg::Power getPower();
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);
extern ll::PowerConfig getPowerConfig();

extern rclcpp::Client<mower_map::srv::GetMowingAreaSrv>::SharedPtr mapClient;

IdleBehavior IdleBehavior::INSTANCE(false);
IdleBehavior IdleBehavior::DOCKED_INSTANCE(true);

std::string IdleBehavior::state_name() {
  return "IDLE";
}

Behavior* IdleBehavior::execute() {
  // Check, if we have a configured map. If not, print info and go to area recorder
  auto mapReq = std::make_shared<mower_map::srv::GetMowingAreaSrv::Request>();
  mapReq->index = 0;
  auto mapResult = mapClient->async_send_request(mapReq);
  if (rclcpp::spin_until_future_complete(rosNode, mapResult, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(rosNode->get_logger(), "We don't have a map configured. Starting Area Recorder!");
    return &AreaRecordingBehavior::INSTANCE;
  }

  // Check, if we have a docking position. If not, print info and go to area recorder
  auto dockReq = std::make_shared<mower_map::srv::GetDockingPointSrv::Request>();
  auto dockResult = dockingPointClient->async_send_request(dockReq);
  if (rclcpp::spin_until_future_complete(rosNode, dockResult, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(rosNode->get_logger(), "We don't have a docking point configured. Starting Area Recorder!");
    return &AreaRecordingBehavior::INSTANCE;
  }

  auto dockResp = dockResult.get();

  setGPS(false);
  geometry_msgs::msg::PoseStamped docking_pose_stamped;
  docking_pose_stamped.pose = dockResp->docking_pose;
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = rosNode->get_clock()->now();

  rclcpp::Rate r(25);
  while (rclcpp::ok()) {
    stopMoving();
    stopBlade();
    const auto last_config = getConfig();
    const auto last_power_config = getPowerConfig();
    const auto last_status = getStatus();
    const auto last_power = getPower();

    const bool automatic_mode = last_config.automatic_mode == eAutoMode::AUTO;
    const bool active_semiautomatic_task =
        last_config.automatic_mode == eAutoMode::SEMIAUTO && shared_state->active_semiautomatic_task;
    const bool rain_delay = last_config.rain_mode == 2 && rosNode->get_clock()->now() < rain_resume;
    if (rain_delay) {
      static auto last_log = rosNode->get_clock()->now();
      if ((rosNode->get_clock()->now() - last_log).seconds() > 300) {
        RCLCPP_INFO(rosNode->get_logger(), "Rain delay: %d minutes",
                    int((rain_resume - rosNode->get_clock()->now()).seconds() / 60));
        last_log = rosNode->get_clock()->now();
      }
    }
    const bool mower_ready = last_power.v_battery > last_power_config.battery_full_voltage &&
                             last_status.mower_motor_temperature < last_config.motor_cold_temperature &&
                             !last_config.manual_pause_mowing && !rain_delay;

    if (manual_start_mowing || ((automatic_mode || active_semiautomatic_task) && mower_ready)) {
      // set the robot's position to the dock if we're actually docked
      if (last_power.v_charge > 5.0) {
        if (PerimeterUndockingBehavior::configured(config)) return &PerimeterUndockingBehavior::INSTANCE;
        RCLCPP_INFO(rosNode->get_logger(), "Currently inside the docking station, we set the robot's pose to the docks pose.");
        setRobotPose(docking_pose_stamped.pose);
        return &UndockingBehavior::INSTANCE;
      }
      // Not docked, so just mow
      setGPS(true);
      return &MowingBehavior::INSTANCE;
    }

    if (start_area_recorder) {
      return &AreaRecordingBehavior::INSTANCE;
    }

    // This gets called if we need to refresh, e.g. on clearing maps
    if (aborted) {
      return &IdleBehavior::INSTANCE;
    }

    if (last_config.docking_redock && stay_docked && last_power.v_charge < 5.0) {
      RCLCPP_WARN(rosNode->get_logger(), "We docked but seem to have lost contact with the charger.  Undocking and trying again!");
      return &UndockingBehavior::RETRY_INSTANCE;
    }

    r.sleep();
  }

  return nullptr;
}

void IdleBehavior::enter() {
  start_area_recorder = false;
  // Reset the docking behavior, to allow docking
  DockingBehavior::INSTANCE.reset();

  // disable it, so that we don't start mowing immediately
  manual_start_mowing = false;

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::reset() {
}

bool IdleBehavior::needs_gps() {
  return false;
}

bool IdleBehavior::mower_enabled() {
  return false;
}

void IdleBehavior::command_home() {
  // IdleBehavior == docked, don't do anything.
}

void IdleBehavior::command_start() {
  // We got start, so we can reset the last manual pause
  auto config = getConfig();
  config.manual_pause_mowing = false;
  setConfig(config);

  manual_start_mowing = true;
}

void IdleBehavior::command_s1() {
  start_area_recorder = true;
}

void IdleBehavior::command_s2() {
}

bool IdleBehavior::redirect_joystick() {
  return false;
}

uint8_t IdleBehavior::get_sub_state() {
  return 0;
}

uint8_t IdleBehavior::get_state() {
  return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
}

IdleBehavior::IdleBehavior(bool stayDocked) {
  this->stay_docked = stayDocked;

  xbot_msgs::msg::ActionInfo start_mowing_action;
  start_mowing_action.action_id = "start_mowing";
  start_mowing_action.enabled = false;
  start_mowing_action.action_name = "Start Mowing";

  xbot_msgs::msg::ActionInfo start_area_recording_action;
  start_area_recording_action.action_id = "start_area_recording";
  start_area_recording_action.enabled = false;
  start_area_recording_action.action_name = "Start Area Recording";

  actions.clear();
  actions.push_back(start_mowing_action);
  actions.push_back(start_area_recording_action);
}

void IdleBehavior::handle_action(std::string action) {
  if (action == "mower_logic:idle/start_mowing") {
    RCLCPP_INFO(rosNode->get_logger(), "Got start_mowing command");
    command_start();
  } else if (action == "mower_logic:idle/start_area_recording") {
    RCLCPP_INFO(rosNode->get_logger(), "Got start_area_recording command");
    command_s1();
  }
}
