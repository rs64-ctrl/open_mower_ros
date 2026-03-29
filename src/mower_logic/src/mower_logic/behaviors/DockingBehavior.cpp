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
#include "DockingBehavior.h"

#include <mower_msgs/msg/power.hpp>

#include "PerimeterDocking.h"

extern rclcpp::Client<mower_map::srv::GetDockingPointSrv>::SharedPtr dockingPointClient;
extern rclcpp_action::Client<mbf_msgs::action::MoveBase>::SharedPtr mbfClient;
extern rclcpp_action::Client<mbf_msgs::action::ExePath>::SharedPtr mbfClientExePath;
extern mower_msgs::msg::Status getStatus();
extern mower_msgs::msg::Power getPower();

extern void stopMoving();
extern bool setGPS(bool enabled);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::msg::ActionInfo>& actions);

DockingBehavior DockingBehavior::INSTANCE;

DockingBehavior::DockingBehavior() {
  xbot_msgs::msg::ActionInfo abort_docking_action;
  abort_docking_action.action_id = "abort_docking";
  abort_docking_action.enabled = true;
  abort_docking_action.action_name = "Stop Docking";

  actions.clear();
  actions.push_back(abort_docking_action);
}

bool DockingBehavior::approach_docking_point() {
  RCLCPP_INFO(rosNode->get_logger(), "Calculating approach path");

  // Calculate a docking approaching point behind the actual docking point
  tf2::Quaternion quat;
  tf2::fromMsg(docking_pose_stamped.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Get the approach start point
  {
    geometry_msgs::msg::PoseStamped docking_approach_point = docking_pose_stamped;
    docking_approach_point.pose.position.x -= cos(yaw) * config.docking_approach_distance;
    docking_approach_point.pose.position.y -= sin(yaw) * config.docking_approach_distance;
    mbf_msgs::action::MoveBase::Goal moveBaseGoal;
    moveBaseGoal.target_pose = docking_approach_point;
    moveBaseGoal.controller = "FTCPlanner";

    auto result = sendGoalAndWaitUnlessAborted<mbf_msgs::action::MoveBase>(mbfClient, moveBaseGoal);
    if (aborted || result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      return false;
    }
  }

  {
    mbf_msgs::action::ExePath::Goal exePathGoal;

    nav_msgs::msg::Path path;

    rclcpp::Time start_wait_time = rosNode->get_clock()->now();
    rclcpp::Rate loop_rate(100);
    while (rclcpp::ok() && (rosNode->get_clock()->now() - start_wait_time).seconds() < config.docking_waiting_time) {
      loop_rate.sleep();
    }

    int dock_point_count = config.docking_approach_distance * 10.0;
    for (int i = 0; i <= dock_point_count; i++) {
      geometry_msgs::msg::PoseStamped docking_pose_stamped_front = docking_pose_stamped;
      docking_pose_stamped_front.pose.position.x -= cos(yaw) * ((dock_point_count - i) / 10.0);
      docking_pose_stamped_front.pose.position.y -= sin(yaw) * ((dock_point_count - i) / 10.0);
      path.poses.push_back(docking_pose_stamped_front);
    }

    exePathGoal.path = path;
    exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
    exePathGoal.dist_tolerance = 0.1;
    exePathGoal.tolerance_from_action = true;
    exePathGoal.controller = "FTCPlanner";
    RCLCPP_INFO(rosNode->get_logger(), "Executing Docking Approach");

    auto approachResult = sendGoalAndWaitUnlessAborted<mbf_msgs::action::ExePath>(mbfClientExePath, exePathGoal);
    if (aborted || approachResult.code != rclcpp_action::ResultCode::SUCCEEDED) {
      return false;
    }
  }

  return true;
}

bool DockingBehavior::dock_straight() {
  tf2::Quaternion quat;
  tf2::fromMsg(docking_pose_stamped.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  mbf_msgs::action::ExePath::Goal exePathGoal;

  nav_msgs::msg::Path path;

  int dock_point_count = config.docking_distance * 10.0;
  for (int i = 0; i < dock_point_count; i++) {
    geometry_msgs::msg::PoseStamped docking_pose_stamped_front = docking_pose_stamped;
    docking_pose_stamped_front.pose.position.x += cos(yaw) * (i / 10.0);
    docking_pose_stamped_front.pose.position.y += sin(yaw) * (i / 10.0);
    path.poses.push_back(docking_pose_stamped_front);
  }

  exePathGoal.path = path;
  exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
  exePathGoal.dist_tolerance = 0.1;
  exePathGoal.tolerance_from_action = true;
  exePathGoal.controller = "DockingFTCPlanner";

  auto send_goal_options = rclcpp_action::Client<mbf_msgs::action::ExePath>::SendGoalOptions();
  auto goal_handle_future = mbfClientExePath->async_send_goal(exePathGoal, send_goal_options);

  if (rclcpp::spin_until_future_complete(rosNode, goal_handle_future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return false;
  }

  auto result_future = mbfClientExePath->async_get_result(goal_handle);

  bool dockingSuccess = false;
  bool waitingForResult = true;

  rclcpp::Rate r(10);

  while (waitingForResult) {
    r.sleep();

    const auto last_status = getStatus();
    const auto last_power = getPower();

    if (aborted) {
      RCLCPP_INFO(rosNode->get_logger(), "Docking aborted.");
      mbfClientExePath->async_cancel_goal(goal_handle);
      stopMoving();
      dockingSuccess = false;
      waitingForResult = false;
      continue;
    }

    // Check if result is available
    bool result_ready = result_future.valid() &&
                        result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;

    if (!result_ready) {
      // currently moving. Cancel as soon as we're in the station
      if (last_power.v_charge > 5.0) {
        RCLCPP_INFO(rosNode->get_logger(), "Got a voltage of %f V. Cancelling docking.", last_power.v_charge);
        std::this_thread::sleep_for(std::chrono::duration<double>(config.docking_extra_time));
        mbfClientExePath->async_cancel_goal(goal_handle);
        stopMoving();
        dockingSuccess = true;
        waitingForResult = false;
      }
    } else {
      auto wrapped_result = result_future.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(rosNode->get_logger(), "Docking stopped, because we reached end pose. Voltage was %f V.", last_power.v_charge);
        if (last_power.v_charge > 5.0) {
          dockingSuccess = true;
          stopMoving();
        }
      } else {
        RCLCPP_WARN(rosNode->get_logger(), "Some error during path execution. Docking failed.");
        stopMoving();
      }
      waitingForResult = false;
    }
  }

  // to be safe if the planner sent additional commands after cancel
  stopMoving();

  return dockingSuccess;
}

std::string DockingBehavior::state_name() {
  return "DOCKING";
}

Behavior* DockingBehavior::execute() {
  // Check if already docked (e.g. carried to base during emergency) and skip
  if (getPower().v_charge > 5.0) {
    RCLCPP_INFO(rosNode->get_logger(), "Already inside docking station, going directly to idle.");
    stopMoving();
    return &IdleBehavior::DOCKED_INSTANCE;
  }

  while (!isGPSGood) {
    if (aborted) {
      RCLCPP_INFO(rosNode->get_logger(), "Docking aborted.");
      stopMoving();
      return &IdleBehavior::INSTANCE;
    }

    RCLCPP_WARN(rosNode->get_logger(), "Waiting for good GPS");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  bool approachSuccess = approach_docking_point();

  if (aborted) {
    RCLCPP_INFO(rosNode->get_logger(), "Docking aborted.");
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  if (!approachSuccess) {
    RCLCPP_ERROR(rosNode->get_logger(), "Error during docking approach.");

    retryCount++;
    if (retryCount <= static_cast<uint>(config.docking_retry_count)) {
      RCLCPP_ERROR(rosNode->get_logger(), "Retrying docking approach");
      return &DockingBehavior::INSTANCE;
    }

    RCLCPP_ERROR(rosNode->get_logger(), "Giving up on docking");
    return &IdleBehavior::INSTANCE;
  }

  // Disable GPS
  inApproachMode = false;
  setGPS(false);

  if (PerimeterSearchBehavior::configured(config)) return &PerimeterSearchBehavior::INSTANCE;

  bool docked = dock_straight();

  if (aborted) {
    RCLCPP_INFO(rosNode->get_logger(), "Docking aborted.");
    stopMoving();
    return &IdleBehavior::INSTANCE;
  }

  if (!docked) {
    RCLCPP_ERROR(rosNode->get_logger(), "Error during docking.");

    retryCount++;
    if (retryCount <= static_cast<uint>(config.docking_retry_count) && !aborted) {
      RCLCPP_ERROR(rosNode->get_logger(), "Retrying docking. Try %u / %d", retryCount, config.docking_retry_count);
      return &UndockingBehavior::RETRY_INSTANCE;
    }

    RCLCPP_ERROR(rosNode->get_logger(), "Giving up on docking");
    reset();
    return &IdleBehavior::INSTANCE;
  }

  reset();

  return &IdleBehavior::DOCKED_INSTANCE;
}

void DockingBehavior::enter() {
  paused = aborted = false;
  inApproachMode = true;

  // Get the docking pose in map
  auto req = std::make_shared<mower_map::srv::GetDockingPointSrv::Request>();
  auto result = dockingPointClient->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto resp = result.get();
    docking_pose_stamped.pose = resp->docking_pose;
  }
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = rosNode->get_clock()->now();

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:docking", actions);
}

void DockingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:docking", actions);
}

void DockingBehavior::reset() {
  retryCount = 0;
}

bool DockingBehavior::needs_gps() {
  return inApproachMode;
}

bool DockingBehavior::mower_enabled() {
  return false;
}

void DockingBehavior::command_home() {
}

void DockingBehavior::command_start() {
  this->abort();
}

void DockingBehavior::command_s1() {
}

void DockingBehavior::command_s2() {
}

bool DockingBehavior::redirect_joystick() {
  return false;
}

uint8_t DockingBehavior::get_sub_state() {
  return 1;
}

uint8_t DockingBehavior::get_state() {
  return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void DockingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:docking/abort_docking") {
    RCLCPP_INFO(rosNode->get_logger(), "Got abort docking command");
    command_start();
  }
}
