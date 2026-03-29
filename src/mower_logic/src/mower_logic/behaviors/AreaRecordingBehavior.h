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
#ifndef SRC_AREA_RECORDING_BEHAVIOR_H
#define SRC_AREA_RECORDING_BEHAVIOR_H

#include <rclcpp_action/rclcpp_action.hpp>
#include <mbf_msgs/action/exe_path.hpp>
#include <mbf_msgs/action/move_base.hpp>
#include <mower_map/srv/get_docking_point_srv.hpp>
#include <tf2/LinearMath/Transform.h>

#include "Behavior.h"
#include "DockingBehavior.h"
#include "IdleBehavior.h"
#include "geometry_msgs/msg/twist.hpp"
#include "mower_map/srv/add_mowing_area_srv.hpp"
#include "mower_map/msg/map_area.hpp"
#include "mower_map/srv/set_docking_point_srv.hpp"
#include "mower_msgs/srv/emergency_stop_srv.hpp"
#include "mower_msgs/msg/status.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "xbot_msgs/msg/map_overlay.hpp"

#define NEW_POINT_MIN_DISTANCE 0.1

class AreaRecordingBehavior : public Behavior {
 public:
  static AreaRecordingBehavior INSTANCE;

  AreaRecordingBehavior();

 private:
  bool has_odom = false;

  std::vector<xbot_msgs::msg::ActionInfo> actions;

  sensor_msgs::msg::Joy last_joy;
  xbot_msgs::msg::AbsolutePose last_pose;

  rclcpp::Publisher<xbot_msgs::msg::MapOverlay>::SharedPtr map_overlay_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Subscription<xbot_msgs::msg::AbsolutePose>::SharedPtr pose_sub;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dock_sub, polygon_sub, mow_area_sub, nav_area_sub,
      auto_point_collecting_sub, collect_point_sub;

  rclcpp::Client<mower_map::srv::AddMowingAreaSrv>::SharedPtr add_mowing_area_client;
  rclcpp::Client<mower_map::srv::SetDockingPointSrv>::SharedPtr set_docking_point_client;

  bool has_first_docking_pos = false;
  geometry_msgs::msg::Pose first_docking_pos;

  bool poly_recording_enabled = false;

  bool is_mowing_area = false;
  bool is_navigation_area = false;
  bool finished_all = false;
  bool set_docking_position = false;
  bool has_outline = false;

  bool auto_point_collecting = true;
  bool collect_point = false;

  bool manual_mowing = false;

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;

 private:
  bool recordNewPolygon(geometry_msgs::msg::Polygon& polygon, xbot_msgs::msg::MapOverlay& resultOverlay);
  bool getDockingPosition(geometry_msgs::msg::Pose& pos);
  void pose_received(const xbot_msgs::msg::AbsolutePose::SharedPtr msg);
  void joy_received(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  void record_dock_received(const std_msgs::msg::Bool::SharedPtr state_msg);
  void record_polygon_received(const std_msgs::msg::Bool::SharedPtr state_msg);
  void record_mowing_received(const std_msgs::msg::Bool::SharedPtr state_msg);
  void record_navigation_received(const std_msgs::msg::Bool::SharedPtr state_msg);
  void record_auto_point_collecting(const std_msgs::msg::Bool::SharedPtr state_msg);
  void record_collect_point(const std_msgs::msg::Bool::SharedPtr state_msg);

  void update_actions();

 public:
  std::string state_name() override;

  std::string sub_state_name() override;

  Behavior* execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  bool needs_gps() override;

  bool mower_enabled() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;

  uint8_t get_sub_state() override;

  uint8_t get_state() override;

  void handle_action(std::string action) override;
};

#endif  // SRC_AREA_RECORDING_BEHAVIOR_H
