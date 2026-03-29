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
#include "MowingBehavior.h"

#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <nav_msgs/msg/path.hpp>

#include <fstream>
#include <sstream>

#include "mower_logic/msg/check_point.hpp"
#include "mower_map/srv/clear_nav_point_srv.hpp"
#include "mower_map/srv/get_mowing_area_srv.hpp"
#include "mower_map/srv/set_nav_point_srv.hpp"

extern rclcpp::Client<mower_map::srv::GetMowingAreaSrv>::SharedPtr mapClient;
extern rclcpp::Client<slic3r_coverage_planner::srv::PlanPath>::SharedPtr pathClient;
extern rclcpp::Client<ftc_local_planner::srv::PlannerGetProgress>::SharedPtr pathProgressClient;
extern rclcpp::Client<mower_map::srv::SetNavPointSrv>::SharedPtr setNavPointClient;
extern rclcpp::Client<mower_map::srv::ClearNavPointSrv>::SharedPtr clearNavPointClient;

extern rclcpp_action::Client<mbf_msgs::action::MoveBase>::SharedPtr mbfClient;
extern rclcpp_action::Client<mbf_msgs::action::ExePath>::SharedPtr mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::msg::ActionInfo>& actions);

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

Behavior* MowingBehavior::execute() {
  shared_state->active_semiautomatic_task = true;

  while (rclcpp::ok() && !aborted) {
    if (currentMowingPaths.empty() && !create_mowing_plan(currentMowingArea)) {
      RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Could not create mowing plan, docking");
      // Start again from first area next time.
      reset();
      // We cannot create a plan, so we're probably done. Go to docking station
      return &DockingBehavior::INSTANCE;
    }

    // No plan will be created if the area is skipped
    if (currentMowingPaths.empty()) {
      currentMowingArea++;
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      continue;
    }

    // We have a plan, execute it
    RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Executing mowing plan");
    bool finished = execute_mowing_plan();
    if (finished) {
      // skip to next area if current
      RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Executing mowing plan - finished");
      currentMowingArea++;
      currentMowingPaths.clear();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
    }
  }

  if (!rclcpp::ok()) {
    // something went wrong
    return nullptr;
  }
  // we got aborted, go to docking station
  return &DockingBehavior::INSTANCE;
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  paused = aborted = false;

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
  currentMowingPaths.clear();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  // increase cumulative mowing angle offset increment
  currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
  checkpoint();

  if (config.automatic_mode == eAutoMode::SEMIAUTO) {
    RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Finished semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto& a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}

bool MowingBehavior::create_mowing_plan(int area_index) {
  RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Creating mowing plan for area: %d", area_index);
  // Delete old plan and progress.
  currentMowingPaths.clear();

  // get the mowing area
  auto mapReq = std::make_shared<mower_map::srv::GetMowingAreaSrv::Request>();
  mapReq->index = area_index;
  auto mapResult = mapClient->async_send_request(mapReq);
  if (rclcpp::spin_until_future_complete(rosNode, mapResult, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: Error loading mowing area");
    return false;
  }
  auto mapResp = mapResult.get();

  if (mapResp->area.area.points.empty()) {
    RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Skipping inactive mowing area");
    return true;
  }

  // Area orientation is the same as the first point
  double angle = 0;
  auto points = mapResp->area.area.points;
  if (points.size() >= 2) {
    tf2::Vector3 first(points[0].x, points[0].y, 0);
    for (auto point : points) {
      tf2::Vector3 second(point.x, point.y, 0);
      auto diff = second - first;
      if (diff.length() > 2.0) {
        // we have found a point that has a distance of > 1 m, calculate the angle
        angle = atan2(diff.y(), diff.x());
        RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Detected mow angle: %f", angle);
        break;
      }
    }
  }

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: mowing angle offset (deg): %f", mow_angle_offset);
  if (config.mow_angle_offset_is_absolute) {
    angle = mow_angle_offset * (M_PI / 180.0);
    RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Custom mowing angle: %f", angle);
  } else {
    angle = angle + mow_angle_offset * (M_PI / 180.0);
    RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Auto-detected mowing angle + mowing angle offset: %f", angle);
  }

  // calculate coverage
  auto pathReq = std::make_shared<slic3r_coverage_planner::srv::PlanPath::Request>();
  pathReq->angle = angle;
  pathReq->outline_count = config.outline_count;
  pathReq->outline_overlap_count = config.outline_overlap_count;
  pathReq->outline = mapResp->area.area;
  pathReq->holes = mapResp->area.obstacles;
  pathReq->fill_type = slic3r_coverage_planner::srv::PlanPath::Request::FILL_LINEAR;
  pathReq->outer_offset = config.outline_offset;
  pathReq->distance = config.tool_width;
  auto pathResult = pathClient->async_send_request(pathReq);
  if (rclcpp::spin_until_future_complete(rosNode, pathResult, std::chrono::seconds(30)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: Error during coverage planning");
    return false;
  }
  auto pathResp = pathResult.get();

  currentMowingPaths = pathResp->paths;

  // Calculate mowing plan digest from the poses
  CryptoPP::SHA256 hash;
  CryptoPP::byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& path : currentMowingPaths) {
    for (const auto& pose_stamped : path.path.poses) {
      hash.Update(reinterpret_cast<const CryptoPP::byte*>(&pose_stamped.pose), sizeof(geometry_msgs::msg::Pose));
    }
  }
  hash.Final(reinterpret_cast<CryptoPP::byte*>(&digest[0]));
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Advancing to checkpoint, path: %d index: %d",
                currentMowingPath, currentMowingPathIndex);
  } else {
    RCLCPP_INFO(rosNode->get_logger(),
                "MowingBehavior: Ignoring checkpoint for plan (%s) current mowing plan is (%s)",
                currentMowingPlanDigest.c_str(), mowingPlanDigest.c_str());
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
  }

  return true;
}

int getCurrentMowPathIndex() {
  extern rclcpp::Client<ftc_local_planner::srv::PlannerGetProgress>::SharedPtr pathProgressClient;
  auto req = std::make_shared<ftc_local_planner::srv::PlannerGetProgress::Request>();
  int currentIndex = -1;
  auto result = pathProgressClient->async_send_request(req);
  if (rclcpp::spin_until_future_complete(rosNode, result, std::chrono::seconds(2)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    currentIndex = result.get()->index;
  } else {
    RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED: RCLCPP_INFO(rosNode->get_logger(), ">>> State: Succeeded <<<"); break;
    case rclcpp_action::ResultCode::ABORTED: RCLCPP_INFO(rosNode->get_logger(), ">>> State: Aborted <<<"); break;
    case rclcpp_action::ResultCode::CANCELED: RCLCPP_INFO(rosNode->get_logger(), ">>> State: Canceled <<<"); break;
    default: RCLCPP_INFO(rosNode->get_logger(), ">>> State: Unknown <<<"); break;
  }
}

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  rclcpp::Time paused_time(0, 0, RCL_ROS_TIME);

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < static_cast<int>(currentMowingPaths.size()) && rclcpp::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      uint8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          pause_reason += "waiting for CONTINUE";
        }
        static auto last_pause_log = rosNode->get_clock()->now();
        if ((rosNode->get_clock()->now() - last_pause_log).seconds() > 30) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: PAUSED (%s)", pause_reason.c_str());
          last_pause_log = rosNode->get_clock()->now();
        }
        rclcpp::Rate r(1.0);
        r.sleep();
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = rosNode->get_clock()->now();
      while (!this->hasGoodGPS() && !aborted)  // while no good GPS we wait
      {
        RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: PAUSED (%fs) (waiting for GPS)",
                    (rosNode->get_clock()->now() - paused_time).seconds());
        rclcpp::Rate r(1.0);
        r.sleep();
      }
      RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: CONTINUING");
      paused = false;
      update_actions();
    }

    auto& path = currentMowingPaths[currentMowingPath];
    RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Path segment length: %zu poses.", path.path.poses.size());

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= static_cast<int>(path.path.poses.size())) {
      RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: Skipping empty path.");
      currentMowingPath++;
      currentMowingPathIndex = 0;
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
      if (path.is_outline && getConfig().add_fake_obstacle) {
        auto setNavReq = std::make_shared<mower_map::srv::SetNavPointSrv::Request>();
        setNavReq->nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient->async_send_request(setNavReq);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      mbf_msgs::action::MoveBase::Goal moveBaseGoal;
      moveBaseGoal.target_pose = path.path.poses[currentMowingPathIndex];
      moveBaseGoal.controller = "FTCPlanner";

      auto send_goal_options = rclcpp_action::Client<mbf_msgs::action::MoveBase>::SendGoalOptions();
      auto goal_handle_future = mbfClient->async_send_goal(moveBaseGoal, send_goal_options);

      if (rclcpp::spin_until_future_complete(rosNode, goal_handle_future, std::chrono::seconds(10)) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: (FIRST POINT) Failed to send goal");
        paused = true;
        update_actions();
        continue;
      }

      auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: (FIRST POINT) Goal rejected");
        paused = true;
        update_actions();
        continue;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
      auto result_future = mbfClient->async_get_result(goal_handle);
      rclcpp::Rate r(10);
      bool goal_done = false;
      rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::ABORTED;

      // wait for path execution to finish
      while (rclcpp::ok()) {
        if (result_future.valid() &&
            result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          auto wrapped_result = result_future.get();
          result_code = wrapped_result.code;
          goal_done = true;
          break;
        }

        // path is being executed, check if we should pause or abort mowing
        if (skip_area) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
          mowerEnabled = false;
          mbfClient->async_cancel_goal(goal_handle);
          currentMowingPaths.clear();
          skip_area = false;
          return true;
        }
        if (skip_path) {
          skip_path = false;
          mbfClient->async_cancel_goal(goal_handle);
          currentMowingPath++;
          currentMowingPathIndex = 0;
          return false;
        }
        if (aborted) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
          mbfClient->async_cancel_goal(goal_handle);
          mowerEnabled = false;
          return false;
        }
        if (requested_pause_flag) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
          mbfClient->async_cancel_goal(goal_handle);
          mowerEnabled = false;
          return false;
        }
        r.sleep();
      }

      first_point_attempt_counter++;
      if (!goal_done || result_code != rclcpp_action::ResultCode::SUCCEEDED) {
        // we cannot reach the start point
        RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: (FIRST POINT) - Could not reach goal (first point).");
        if (first_point_attempt_counter < config.max_first_point_attempts) {
          RCLCPP_WARN(rosNode->get_logger(), "MowingBehavior: (FIRST POINT) - Attempt %d / %d Making a little pause ...",
                      first_point_attempt_counter, config.max_first_point_attempts);
          paused = true;
          update_actions();
        } else {
          if (first_point_trim_counter < config.max_first_point_trim_attempts) {
            RCLCPP_WARN(rosNode->get_logger(),
                        "MowingBehavior: (FIRST POINT) - Attempt %d / %d Trimming first point off the beginning of the mow path.",
                        first_point_trim_counter, config.max_first_point_trim_attempts);
            currentMowingPathIndex++;
            first_point_trim_counter++;
            first_point_attempt_counter = 0;
            paused = true;
            update_actions();
          } else {
            RCLCPP_ERROR(rosNode->get_logger(),
                         "MowingBehavior: (FIRST POINT) Max retries reached - aborting at index: %d path: %d area: %d",
                         currentMowingPathIndex, currentMowingPath, currentMowingArea);
            this->abort();
          }
        }
        continue;
      }

      auto clearNavReq = std::make_shared<mower_map::srv::ClearNavPointSrv::Request>();
      clearNavPointClient->async_send_request(clearNavReq);

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower
      mowerEnabled = true;

      mbf_msgs::action::ExePath::Goal exePathGoal;
      nav_msgs::msg::Path exePath;
      exePath.header = path.path.header;
      exePath.poses = std::vector<geometry_msgs::msg::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex,
                                                                    path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

      RCLCPP_INFO(rosNode->get_logger(),
                  "MowingBehavior: (MOW) First point reached - Executing mow path with %zu poses, from index %d",
                  path.path.poses.size(), exePathStartIndex);

      auto send_goal_options = rclcpp_action::Client<mbf_msgs::action::ExePath>::SendGoalOptions();
      auto goal_handle_future = mbfClientExePath->async_send_goal(exePathGoal, send_goal_options);

      if (rclcpp::spin_until_future_complete(rosNode, goal_handle_future, std::chrono::seconds(10)) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: (MOW) Failed to send exe_path goal");
        mowerEnabled = false;
        paused = true;
        update_actions();
        continue;
      }

      auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(rosNode->get_logger(), "MowingBehavior: (MOW) Goal rejected");
        mowerEnabled = false;
        paused = true;
        update_actions();
        continue;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
      auto result_future = mbfClientExePath->async_get_result(goal_handle);
      rclcpp::Rate r(10);
      bool goal_done = false;
      rclcpp_action::ResultCode result_code = rclcpp_action::ResultCode::ABORTED;

      // wait for path execution to finish
      while (rclcpp::ok()) {
        if (result_future.valid() &&
            result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          auto wrapped_result = result_future.get();
          result_code = wrapped_result.code;
          goal_done = true;
          break;
        }

        // path is being executed
        if (skip_area) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (MOW) SKIP AREA was requested.");
          mowerEnabled = false;
          mbfClientExePath->async_cancel_goal(goal_handle);
          currentMowingPaths.clear();
          skip_area = false;
          return true;
        }
        if (skip_path) {
          skip_path = false;
          mbfClientExePath->async_cancel_goal(goal_handle);
          currentMowingPath++;
          currentMowingPathIndex = 0;
          return false;
        }
        if (aborted) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
          mbfClientExePath->async_cancel_goal(goal_handle);
          mowerEnabled = false;
          break;  // Trim path
        }
        if (requested_pause_flag) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
          mbfClientExePath->async_cancel_goal(goal_handle);
          mowerEnabled = false;
          break;  // Trim path
        }

        // show progress
        int currentIndex = getCurrentMowPathIndex();
        if (currentIndex != -1) {
          currentMowingPathIndex = exePathStartIndex + currentIndex;
        }
        static auto last_progress_log = rosNode->get_clock()->now();
        if ((rosNode->get_clock()->now() - last_progress_log).seconds() > 5) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (MOW) Progress: %d/%zu",
                      currentMowingPathIndex, path.path.poses.size());
          last_progress_log = rosNode->get_clock()->now();
        }
        if ((rosNode->get_clock()->now() - last_checkpoint).seconds() > 30.0) checkpoint();

        r.sleep();
      }

      // Only skip/trim if goal execution began
      if (goal_done) {
        RCLCPP_INFO(rosNode->get_logger(),
                    ">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = %d of %zu",
                    currentMowingPathIndex, path.path.poses.size());
        printNavState(result_code);
        if (currentMowingPathIndex >= static_cast<int>(path.path.poses.size()) ||
            (static_cast<int>(path.path.poses.size()) - currentMowingPathIndex) < 5) {
          RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          currentMowingPath++;
          currentMowingPathIndex = 0;
        } else {
          if (currentMowingPathIndex == 0) currentMowingPathIndex++;
          if (!requested_pause_flag) {
            RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: (MOW) PAUSED due to MBF Error at %d", currentMowingPathIndex);
            paused = true;
            update_actions();
          }
        }
      }
    }
  }

  mowerEnabled = false;

  // true, if we have executed all paths
  return currentMowingPath >= static_cast<int>(currentMowingPaths.size());
}

void MowingBehavior::command_home() {
  if (shared_state->active_semiautomatic_task) {
    RCLCPP_INFO(rosNode->get_logger(), "Manually pausing semiautomatic task");
    auto config = getConfig();
    config.manual_pause_mowing = true;
    setConfig(config);
  }
  if (paused) {
    this->requestContinue();
  }
  this->abort();
}

void MowingBehavior::command_start() {
  RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: MANUAL CONTINUE");
  auto config = getConfig();
  if (shared_state->active_semiautomatic_task && config.manual_pause_mowing) {
    RCLCPP_INFO(rosNode->get_logger(), "Resuming semiautomatic task");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  this->requestContinue();
}

void MowingBehavior::command_s1() {
  RCLCPP_INFO(rosNode->get_logger(), "MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() {
  return currentMowingArea;
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = rclcpp::Time(0, 0, RCL_ROS_TIME);
  xbot_msgs::msg::ActionInfo pause_action;
  pause_action.action_id = "pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::msg::ActionInfo continue_action;
  continue_action.action_id = "continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::msg::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::msg::ActionInfo skip_area_action;
  skip_area_action.action_id = "skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::msg::ActionInfo skip_path_action;
  skip_path_action.action_id = "skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
  restore_checkpoint();
}

void MowingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:mowing/pause") {
    RCLCPP_INFO(rosNode->get_logger(), "got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    RCLCPP_INFO(rosNode->get_logger(), "got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    RCLCPP_INFO(rosNode->get_logger(), "got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/skip_area") {
    RCLCPP_INFO(rosNode->get_logger(), "got skip_area command");
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    RCLCPP_INFO(rosNode->get_logger(), "got skip_path command");
    skip_path = true;
  }
  update_actions();
}

void MowingBehavior::checkpoint() {
  // Simple file-based checkpoint (replaces rosbag checkpoint)
  try {
    std::ofstream ofs("checkpoint.txt");
    if (ofs.is_open()) {
      ofs << currentMowingPath << "\n"
          << currentMowingArea << "\n"
          << currentMowingPathIndex << "\n"
          << currentMowingPlanDigest << "\n"
          << currentMowingAngleIncrementSum << "\n";
      ofs.close();
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rosNode->get_logger(), "Error writing checkpoint: %s", e.what());
  }
  last_checkpoint = rosNode->get_clock()->now();
}

bool MowingBehavior::restore_checkpoint() {
  try {
    std::ifstream ifs("checkpoint.txt");
    if (!ifs.is_open()) {
      currentMowingArea = 0;
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      currentMowingAngleIncrementSum = 0;
      return false;
    }

    std::string line;
    if (std::getline(ifs, line)) currentMowingPath = std::stoi(line);
    if (std::getline(ifs, line)) currentMowingArea = std::stoi(line);
    if (std::getline(ifs, line)) currentMowingPathIndex = std::stoi(line);
    if (std::getline(ifs, line)) currentMowingPlanDigest = line;
    if (std::getline(ifs, line)) currentMowingAngleIncrementSum = std::stod(line);
    ifs.close();

    RCLCPP_INFO(rosNode->get_logger(),
                "Restoring checkpoint for plan (%s) area: %d path: %d index: %d angle increment sum: %f",
                currentMowingPlanDigest.c_str(), currentMowingArea, currentMowingPath,
                currentMowingPathIndex, currentMowingAngleIncrementSum);
    return true;
  } catch (const std::exception& e) {
    currentMowingArea = 0;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
    currentMowingAngleIncrementSum = 0;
    return false;
  }
}
