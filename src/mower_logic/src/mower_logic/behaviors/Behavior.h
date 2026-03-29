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
#ifndef SRC_BEHAVIOR_H
#define SRC_BEHAVIOR_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <atomic>
#include <memory>

#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/msg/high_level_status.hpp"

enum eAutoMode { MANUAL = 0, SEMIAUTO = 1, AUTO = 2 };

enum pauseType { PAUSE_MANUAL = 0b1, PAUSE_EMERGENCY = 0b10 };

struct sSharedState {
  // True, if the semiautomatic task is still in progress
  bool active_semiautomatic_task;
};

// Forward-declare the global node pointer used by behaviors
extern rclcpp::Node::SharedPtr rosNode;

/**
 * Behavior definition
 */
class Behavior {
 private:
  rclcpp::Time startTime;

 protected:
  std::atomic<bool> aborted;
  std::atomic<bool> paused;

  std::atomic<uint8_t> requested_pause_flag;

  std::atomic<bool> isGPSGood;
  std::atomic<uint8_t> sub_state;

  double time_in_state() {
    return (rosNode->get_clock()->now() - startTime).seconds();
  }

  mower_logic::MowerLogicConfig config;
  std::shared_ptr<sSharedState> shared_state;

  /**
   * Called ONCE on state enter.
   */
  virtual void enter() = 0;

 public:
  virtual std::string state_name() = 0;

  virtual std::string sub_state_name() {
    return "";
  }

  bool hasGoodGPS() {
    return isGPSGood;
  }

  void setGoodGPS(bool isGood) {
    isGPSGood = isGood;
  }

  void requestContinue(pauseType reason = pauseType::PAUSE_MANUAL) {
    requested_pause_flag &= ~reason;
  }

  void requestPause(pauseType reason = pauseType::PAUSE_MANUAL) {
    requested_pause_flag |= reason;
  }

  void start(mower_logic::MowerLogicConfig& c, std::shared_ptr<sSharedState> s) {
    RCLCPP_INFO(rosNode->get_logger(), "");
    RCLCPP_INFO(rosNode->get_logger(), "");
    RCLCPP_INFO(rosNode->get_logger(), "--------------------------------------");
    RCLCPP_INFO(rosNode->get_logger(), "- Entered state: %s", state_name().c_str());
    RCLCPP_INFO(rosNode->get_logger(), "--------------------------------------");
    aborted = false;
    paused = false;
    requested_pause_flag = 0;
    this->config = c;
    this->shared_state = std::move(s);
    startTime = rosNode->get_clock()->now();
    isGPSGood = false;
    sub_state = 0;
    enter();
  }

  template <typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult sendGoalAndWaitUnlessAborted(
      typename rclcpp_action::Client<ActionT>::SharedPtr client,
      const typename ActionT::Goal& goal,
      double poll_rate = 10) {
    rclcpp::Rate rate(poll_rate);

    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    auto goal_handle_future = client->async_send_goal(goal, send_goal_options);

    // Wait for goal to be accepted
    if (rclcpp::spin_until_future_complete(rosNode, goal_handle_future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult wrapped;
      wrapped.code = rclcpp_action::ResultCode::ABORTED;
      return wrapped;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult wrapped;
      wrapped.code = rclcpp_action::ResultCode::ABORTED;
      return wrapped;
    }

    auto result_future = client->async_get_result(goal_handle);

    while (rclcpp::ok()) {
      rate.sleep();

      if (aborted) {
        client->async_cancel_goal(goal_handle);
        // Wait briefly for cancellation
        rclcpp::spin_until_future_complete(rosNode, result_future, std::chrono::seconds(2));
        if (result_future.valid() &&
            result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          return result_future.get();
        }
        typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult wrapped;
        wrapped.code = rclcpp_action::ResultCode::CANCELED;
        return wrapped;
      }

      if (result_future.valid() &&
          result_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        return result_future.get();
      }
    }

    // Should not reach here
    typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult wrapped;
    wrapped.code = rclcpp_action::ResultCode::ABORTED;
    return wrapped;
  }

  /**
   * Execute the behavior. This call should block until the behavior is executed fully.
   * @returns the pointer to the next behavior (can return itself).
   */
  virtual Behavior* execute() = 0;

  /**
   * Called ONCE before state exits
   */
  virtual void exit() = 0;

  /**
   * Reset the internal state of the behavior.
   */
  virtual void reset() = 0;

  /**
   * If called, save state internally and return the execute() method asap.
   * Execution should resume on the next execute() call.
   */
  void abort() {
    if (!aborted) {
      RCLCPP_INFO(rosNode->get_logger(), "- Behaviour.h: abort() called");
    }
    aborted = true;
  }

  // Return true, if this state needs absolute positioning.
  // The state will be aborted if GPS is lost and resumed at some later point in time.
  virtual bool needs_gps() = 0;

  // return true, if the mower motor should currently be running.
  virtual bool mower_enabled() = 0;

  // return true to redirect joystick speeds to the controller
  virtual bool redirect_joystick() = 0;

  virtual void command_home() = 0;
  virtual void command_start() = 0;
  virtual void command_s1() = 0;
  virtual void command_s2() = 0;

  virtual uint8_t get_sub_state() = 0;
  virtual uint8_t get_state() = 0;

  virtual void handle_action(std::string action) = 0;
};

#endif  // SRC_BEHAVIOR_H
