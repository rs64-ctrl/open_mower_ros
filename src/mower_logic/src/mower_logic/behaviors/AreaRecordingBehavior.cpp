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
#include "AreaRecordingBehavior.h"

extern rclcpp::Client<mower_map::srv::GetDockingPointSrv>::SharedPtr dockingPointClient;
extern rclcpp::Client<mower_msgs::srv::EmergencyStopSrv>::SharedPtr emergencyClient;
extern rclcpp_action::Client<mbf_msgs::action::MoveBase>::SharedPtr mbfClient;
extern rclcpp_action::Client<mbf_msgs::action::ExePath>::SharedPtr mbfClientExePath;
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::msg::ActionInfo>& actions);

extern void stop();

extern bool setGPS(bool enabled);

AreaRecordingBehavior AreaRecordingBehavior::INSTANCE;

std::string AreaRecordingBehavior::state_name() {
  return "AREA_RECORDING";
}

Behavior* AreaRecordingBehavior::execute() {
  setGPS(true);
  bool error = false;
  rclcpp::Rate inputDelay(10.0);  // 0.1s

  while (rclcpp::ok() && !aborted) {
    mower_map::msg::MapArea result;
    xbot_msgs::msg::MapOverlay result_overlay;

    // clear overlay
    map_overlay_pub->publish(result_overlay);

    has_outline = false;

    sub_state = 0;
    while (rclcpp::ok() && !finished_all && !error && !aborted) {
      if (set_docking_position) {
        geometry_msgs::msg::Pose pos;
        if (getDockingPosition(pos)) {
          RCLCPP_INFO(rosNode->get_logger(), "new docking pos recorded");

          auto req = std::make_shared<mower_map::srv::SetDockingPointSrv::Request>();
          req->docking_pose = pos;
          set_docking_point_client->async_send_request(req);

          has_first_docking_pos = false;
          update_actions();
        }

        set_docking_position = false;
      }

      if (poly_recording_enabled) {
        update_actions();
        geometry_msgs::msg::Polygon poly;
        if (has_outline) {
          sub_state = 1;
        } else {
          sub_state = 2;
        }
        bool success = recordNewPolygon(poly, result_overlay);
        sub_state = 0;
        if (success) {
          if (!has_outline) {
            has_outline = true;
            result.area = poly;

            std_msgs::msg::ColorRGBA color;
            color.r = 0.0f;
            color.g = 1.0f;
            color.b = 0.0f;
            color.a = 1.0f;

            marker.color = color;
            marker.id = markers.markers.size() + 1;
            marker.action = visualization_msgs::msg::Marker::ADD;
            markers.markers.push_back(marker);
          } else {
            result.obstacles.push_back(poly);

            std_msgs::msg::ColorRGBA color;
            color.r = 1.0f;
            color.g = 0.0f;
            color.b = 0.0f;
            color.a = 1.0f;

            marker.color = color;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.id = markers.markers.size() + 1;
            markers.markers.push_back(marker);
          }

        } else {
          error = true;
          RCLCPP_ERROR(rosNode->get_logger(), "Error during poly record");
        }
        marker_array_pub->publish(markers);
        update_actions();
      }

      inputDelay.sleep();
    }

    if (!error && has_outline && (is_mowing_area || is_navigation_area)) {
      if (is_mowing_area) {
        RCLCPP_INFO(rosNode->get_logger(), "Area recording completed. Adding mowing area.");
      } else if (is_navigation_area) {
        RCLCPP_INFO(rosNode->get_logger(), "Area recording completed. Adding navigation area.");
      }
      auto req = std::make_shared<mower_map::srv::AddMowingAreaSrv::Request>();
      req->is_navigation_area = !is_mowing_area;
      req->area = result;
      auto futResult = add_mowing_area_client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(rosNode, futResult, std::chrono::seconds(5)) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rosNode->get_logger(), "Area added successfully");
      } else {
        RCLCPP_ERROR(rosNode->get_logger(), "error adding area");
      }
    }

    error = false;
    finished_all = false;

    has_outline = false;
    update_actions();
  }

  return &IdleBehavior::INSTANCE;
}

void AreaRecordingBehavior::enter() {
  has_outline = false;
  is_mowing_area = false;
  is_navigation_area = false;
  manual_mowing = false;

  update_actions();

  has_first_docking_pos = false;
  has_odom = false;
  poly_recording_enabled = false;
  finished_all = false;
  set_docking_position = false;
  markers = visualization_msgs::msg::MarkerArray();
  paused = aborted = false;

  add_mowing_area_client = rosNode->create_client<mower_map::srv::AddMowingAreaSrv>("mower_map_service/add_mowing_area");
  set_docking_point_client = rosNode->create_client<mower_map::srv::SetDockingPointSrv>("mower_map_service/set_docking_point");

  marker_pub = rosNode->create_publisher<visualization_msgs::msg::Marker>("area_recorder/progress_visualization", 10);
  map_overlay_pub = rosNode->create_publisher<xbot_msgs::msg::MapOverlay>("xbot_monitoring/map_overlay", 10);
  marker_array_pub = rosNode->create_publisher<visualization_msgs::msg::MarkerArray>("area_recorder/progress_visualization_array", 10);

  RCLCPP_INFO(rosNode->get_logger(), "Starting recording area");

  RCLCPP_INFO(rosNode->get_logger(), "Subscribing to /joy for user input");

  joy_sub = rosNode->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 100, std::bind(&AreaRecordingBehavior::joy_received, this, std::placeholders::_1));

  dock_sub = rosNode->create_subscription<std_msgs::msg::Bool>(
      "/record_dock", 100, std::bind(&AreaRecordingBehavior::record_dock_received, this, std::placeholders::_1));
  polygon_sub = rosNode->create_subscription<std_msgs::msg::Bool>(
      "/record_polygon", 100, std::bind(&AreaRecordingBehavior::record_polygon_received, this, std::placeholders::_1));
  mow_area_sub = rosNode->create_subscription<std_msgs::msg::Bool>(
      "/record_mowing", 100, std::bind(&AreaRecordingBehavior::record_mowing_received, this, std::placeholders::_1));
  nav_area_sub = rosNode->create_subscription<std_msgs::msg::Bool>(
      "/record_navigation", 100, std::bind(&AreaRecordingBehavior::record_navigation_received, this, std::placeholders::_1));

  auto_point_collecting_sub = rosNode->create_subscription<std_msgs::msg::Bool>(
      "/record_auto_point_collecting", 100,
      std::bind(&AreaRecordingBehavior::record_auto_point_collecting, this, std::placeholders::_1));
  collect_point_sub = rosNode->create_subscription<std_msgs::msg::Bool>(
      "/record_collect_point", 100, std::bind(&AreaRecordingBehavior::record_collect_point, this, std::placeholders::_1));

  pose_sub = rosNode->create_subscription<xbot_msgs::msg::AbsolutePose>(
      "/xbot_positioning/xb_pose", 100, std::bind(&AreaRecordingBehavior::pose_received, this, std::placeholders::_1));
}

void AreaRecordingBehavior::exit() {
  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:area_recording", actions);

  map_overlay_pub.reset();
  marker_pub.reset();
  marker_array_pub.reset();
  joy_sub.reset();
  dock_sub.reset();
  polygon_sub.reset();
  mow_area_sub.reset();
  nav_area_sub.reset();
  auto_point_collecting_sub.reset();
  collect_point_sub.reset();
  pose_sub.reset();
  add_mowing_area_client.reset();
  set_docking_point_client.reset();
}

void AreaRecordingBehavior::reset() {
}

bool AreaRecordingBehavior::needs_gps() {
  return false;
}

bool AreaRecordingBehavior::mower_enabled() {
  return manual_mowing;
}

void AreaRecordingBehavior::pose_received(const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
  last_pose = *msg;
  has_odom = true;
}

void AreaRecordingBehavior::joy_received(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
  if (joy_msg->buttons.size() < 6 || joy_msg->axes.size() < 8 || last_joy.buttons.size() < 6) {
    last_joy = *joy_msg;
    return;
  }

  if (joy_msg->buttons[1] && !last_joy.buttons[1]) {
    RCLCPP_INFO(rosNode->get_logger(), "B PRESSED");
    poly_recording_enabled = !poly_recording_enabled;
  }
  if ((joy_msg->buttons[3] && joy_msg->axes[7] > 0.5) && !(last_joy.buttons[3] && last_joy.axes[7] > 0.5)) {
    RCLCPP_INFO(rosNode->get_logger(), "Y + UP PRESSED, recording navigation area");
    poly_recording_enabled = false;
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  }
  if ((joy_msg->buttons[3] && joy_msg->axes[7] < -0.5) && !(last_joy.buttons[3] && last_joy.axes[7] < -0.5)) {
    RCLCPP_INFO(rosNode->get_logger(), "Y + DOWN PRESSED, recording mowing area");
    poly_recording_enabled = false;
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  }

  if (joy_msg->buttons[2] && !last_joy.buttons[2]) {
    RCLCPP_INFO(rosNode->get_logger(), "X PRESSED");
    set_docking_position = true;
  }

  if (joy_msg->buttons[5] && !last_joy.buttons[5]) {
    if (joy_msg->buttons[4] && !last_joy.buttons[4]) {
      RCLCPP_INFO(rosNode->get_logger(), "LB+RB PRESSED, toggle auto point collecting");
      auto_point_collecting = !auto_point_collecting;
      RCLCPP_INFO(rosNode->get_logger(), "Auto point collecting: %s", auto_point_collecting ? "true" : "false");
    } else {
      RCLCPP_INFO(rosNode->get_logger(), "RB PRESSED, collect point");
      collect_point = true;
    }
  }

  last_joy = *joy_msg;
}

void AreaRecordingBehavior::record_dock_received(const std_msgs::msg::Bool::SharedPtr state_msg) {
  if (state_msg->data) {
    RCLCPP_INFO(rosNode->get_logger(), "Record dock position");
    set_docking_position = true;
  }
}

void AreaRecordingBehavior::record_polygon_received(const std_msgs::msg::Bool::SharedPtr state_msg) {
  if (state_msg->data) {
    RCLCPP_INFO(rosNode->get_logger(), "Toggle record polygon");
    poly_recording_enabled = !poly_recording_enabled;
  }
}

void AreaRecordingBehavior::record_navigation_received(const std_msgs::msg::Bool::SharedPtr state_msg) {
  if (state_msg->data) {
    RCLCPP_INFO(rosNode->get_logger(), "Save polygon as navigation area");
    poly_recording_enabled = false;
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  }
}

void AreaRecordingBehavior::record_mowing_received(const std_msgs::msg::Bool::SharedPtr state_msg) {
  if (state_msg->data) {
    RCLCPP_INFO(rosNode->get_logger(), "Save polygon as mowing area");
    poly_recording_enabled = false;
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  }
}

bool AreaRecordingBehavior::recordNewPolygon(geometry_msgs::msg::Polygon& polygon,
                                              xbot_msgs::msg::MapOverlay& resultOverlay) {
  RCLCPP_INFO(rosNode->get_logger(), "recordNewPolygon");

  bool success = true;
  marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "map";
  marker.ns = "area_recorder";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = 0;
  marker.pose.orientation.w = 1.0f;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.frame_locked = true;

  std_msgs::msg::ColorRGBA color;
  color.b = 1.0f;
  color.a = 1.0f;

  marker.color = color;

  rclcpp::Rate updateRate(10);

  has_odom = false;

  // push a new poly to the visualization overlay
  {
    xbot_msgs::msg::MapOverlayPolygon poly_viz;
    poly_viz.closed = false;
    poly_viz.line_width = 0.1;
    poly_viz.color = "blue";
    resultOverlay.polygons.push_back(poly_viz);
  }
  auto& poly_viz = resultOverlay.polygons.back();

  while (true) {
    if (!rclcpp::ok() || aborted) {
      RCLCPP_WARN(rosNode->get_logger(), "Preempting Area Recorder");
      success = false;
      break;
    }

    updateRate.sleep();

    if (!has_odom) continue;

    auto pose_in_map = last_pose.pose.pose;
    if (polygon.points.empty()) {
      geometry_msgs::msg::Point32 pt;
      pt.x = pose_in_map.position.x;
      pt.y = pose_in_map.position.y;
      pt.z = 0.0;

      polygon.points.push_back(pt);
      {
        geometry_msgs::msg::Point vpt;
        vpt.x = pt.x;
        vpt.y = pt.y;
        marker.points.push_back(vpt);
      }

      marker.header.stamp = rosNode->get_clock()->now();
      marker.header.frame_id = "map";

      marker_pub->publish(marker);

      polygon.points.push_back(pt);
      poly_viz.polygon.points.push_back(pt);
      map_overlay_pub->publish(resultOverlay);
    } else {
      auto last = polygon.points.back();
      tf2::Vector3 last_point(last.x, last.y, 0.0);
      tf2::Vector3 current_point(pose_in_map.position.x, pose_in_map.position.y, 0.0);

      bool is_new_point_far_enough = (current_point - last_point).length() > NEW_POINT_MIN_DISTANCE;
      bool is_point_auto_collected = auto_point_collecting && is_new_point_far_enough;
      bool is_point_manual_collected = !auto_point_collecting && collect_point && is_new_point_far_enough;

      if (is_point_auto_collected || is_point_manual_collected) {
        geometry_msgs::msg::Point32 pt;
        pt.x = pose_in_map.position.x;
        pt.y = pose_in_map.position.y;
        pt.z = 0.0;
        polygon.points.push_back(pt);
        {
          geometry_msgs::msg::Point vpt;
          vpt.x = pt.x;
          vpt.y = pt.y;
          marker.points.push_back(vpt);
        }

        marker.header.stamp = rosNode->get_clock()->now();
        marker.header.frame_id = "map";

        marker_pub->publish(marker);

        poly_viz.polygon.points.push_back(pt);
        map_overlay_pub->publish(resultOverlay);

        if (is_point_manual_collected) {
          collect_point = false;
        }
      }
    }

    if (!poly_recording_enabled) {
      if (polygon.points.size() > 2) {
        polygon.points.push_back(polygon.points.front());
      } else {
        success = false;
      }
      RCLCPP_INFO(rosNode->get_logger(), "Finished Recording polygon");
      break;
    }
  }

  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker_pub->publish(marker);

  // close poly
  poly_viz.closed = true;
  poly_viz.line_width = 0.05;
  if (resultOverlay.polygons.size() == 1) {
    poly_viz.color = "green";
  } else {
    poly_viz.color = "red";
  }
  map_overlay_pub->publish(resultOverlay);

  return success;
}

bool AreaRecordingBehavior::getDockingPosition(geometry_msgs::msg::Pose& pos) {
  if (!has_first_docking_pos) {
    RCLCPP_INFO(rosNode->get_logger(), "Recording first docking position");

    // Wait for a pose message
    bool got_pose = false;
    xbot_msgs::msg::AbsolutePose received_pose;
    auto temp_sub = rosNode->create_subscription<xbot_msgs::msg::AbsolutePose>(
        "/xbot_positioning/xb_pose", 1,
        [&](const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
          received_pose = *msg;
          got_pose = true;
        });

    rclcpp::Rate wait_rate(10);
    auto start = rosNode->get_clock()->now();
    while (!got_pose && (rosNode->get_clock()->now() - start).seconds() < 2.0) {
      rclcpp::spin_some(rosNode);
      wait_rate.sleep();
    }
    temp_sub.reset();

    if (!got_pose) return false;

    first_docking_pos = received_pose.pose.pose;
    has_first_docking_pos = true;
    update_actions();
    return false;
  } else {
    RCLCPP_INFO(rosNode->get_logger(), "Recording second docking position");

    bool got_pose = false;
    xbot_msgs::msg::AbsolutePose received_pose;
    auto temp_sub = rosNode->create_subscription<xbot_msgs::msg::AbsolutePose>(
        "/xbot_positioning/xb_pose", 1,
        [&](const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
          received_pose = *msg;
          got_pose = true;
        });

    rclcpp::Rate wait_rate(10);
    auto start = rosNode->get_clock()->now();
    while (!got_pose && (rosNode->get_clock()->now() - start).seconds() < 2.0) {
      rclcpp::spin_some(rosNode);
      wait_rate.sleep();
    }
    temp_sub.reset();

    if (!got_pose) return false;

    pos.position = received_pose.pose.pose.position;

    double yaw = atan2(pos.position.y - first_docking_pos.position.y, pos.position.x - first_docking_pos.position.x);
    tf2::Quaternion docking_orientation;
    docking_orientation.setRPY(0.0, 0.0, yaw);
    pos.orientation = tf2::toMsg(docking_orientation);

    update_actions();
    return true;
  }
}

void AreaRecordingBehavior::command_home() {
  abort();
}

void AreaRecordingBehavior::command_start() {
}

void AreaRecordingBehavior::command_s1() {
}

void AreaRecordingBehavior::command_s2() {
}

bool AreaRecordingBehavior::redirect_joystick() {
  return true;
}

uint8_t AreaRecordingBehavior::get_sub_state() {
  return sub_state;
}

uint8_t AreaRecordingBehavior::get_state() {
  return mower_msgs::msg::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING;
}

std::string AreaRecordingBehavior::sub_state_name() {
  if (has_first_docking_pos) {
    return "RECORD_DOCKING_POSITION";
  }
  switch (sub_state) {
    case 0: return "";
    case 1: return "RECORD_OUTLINE";
    case 2: return "RECORD_OBSTACLE";
    default: return "";
  }
}

void AreaRecordingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:area_recording/start_recording") {
    RCLCPP_INFO(rosNode->get_logger(), "Got start recording");
    poly_recording_enabled = true;
  } else if (action == "mower_logic:area_recording/stop_recording") {
    RCLCPP_INFO(rosNode->get_logger(), "Got stop recording");
    poly_recording_enabled = false;
  } else if (action == "mower_logic:area_recording/finish_navigation_area") {
    RCLCPP_INFO(rosNode->get_logger(), "Got save navigation area");
    poly_recording_enabled = false;
    is_mowing_area = false;
    is_navigation_area = true;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/finish_mowing_area") {
    RCLCPP_INFO(rosNode->get_logger(), "Got save mowing area");
    poly_recording_enabled = false;
    is_mowing_area = true;
    is_navigation_area = false;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/finish_discard") {
    RCLCPP_INFO(rosNode->get_logger(), "Got discard recorded area");
    poly_recording_enabled = false;
    is_mowing_area = false;
    is_navigation_area = false;
    finished_all = true;
  } else if (action == "mower_logic:area_recording/exit_recording_mode") {
    RCLCPP_INFO(rosNode->get_logger(), "Got exit without saving");
    poly_recording_enabled = false;
    is_mowing_area = false;
    is_navigation_area = false;
    finished_all = true;
    abort();
  } else if (action == "mower_logic:area_recording/record_dock") {
    RCLCPP_INFO(rosNode->get_logger(), "Got record dock");
    set_docking_position = true;
  } else if (action == "mower_logic:area_recording/auto_point_collecting_enable") {
    RCLCPP_INFO(rosNode->get_logger(), "Got enable auto point collecting");
    auto_point_collecting = true;
  } else if (action == "mower_logic:area_recording/auto_point_collecting_disable") {
    RCLCPP_INFO(rosNode->get_logger(), "Got disable auto point collecting");
    auto_point_collecting = false;
  } else if (action == "mower_logic:area_recording/collect_point") {
    RCLCPP_INFO(rosNode->get_logger(), "Got collect point");
    collect_point = true;
  } else if (action == "mower_logic:area_recording/start_manual_mowing") {
    RCLCPP_INFO(rosNode->get_logger(), "Starting manual mowing");
    manual_mowing = true;
  } else if (action == "mower_logic:area_recording/stop_manual_mowing") {
    RCLCPP_INFO(rosNode->get_logger(), "Stopping manual mowing");
    manual_mowing = false;
  }
  update_actions();
}

AreaRecordingBehavior::AreaRecordingBehavior() {
  xbot_msgs::msg::ActionInfo start_recording_action;
  start_recording_action.action_id = "start_recording";
  start_recording_action.enabled = false;
  start_recording_action.action_name = "Start Recording";

  xbot_msgs::msg::ActionInfo stop_recording_action;
  stop_recording_action.action_id = "stop_recording";
  stop_recording_action.enabled = false;
  stop_recording_action.action_name = "Stop Recording";

  xbot_msgs::msg::ActionInfo finish_navigation_area_action;
  finish_navigation_area_action.action_id = "finish_navigation_area";
  finish_navigation_area_action.enabled = false;
  finish_navigation_area_action.action_name = "Save Navigation Area";

  xbot_msgs::msg::ActionInfo finish_mowing_area_action;
  finish_mowing_area_action.action_id = "finish_mowing_area";
  finish_mowing_area_action.enabled = false;
  finish_mowing_area_action.action_name = "Save Mowing Area";

  xbot_msgs::msg::ActionInfo exit_recording_mode_action;
  exit_recording_mode_action.action_id = "exit_recording_mode";
  exit_recording_mode_action.enabled = false;
  exit_recording_mode_action.action_name = "Exit";

  xbot_msgs::msg::ActionInfo finish_discard_action;
  finish_discard_action.action_id = "finish_discard";
  finish_discard_action.enabled = false;
  finish_discard_action.action_name = "Discard Area";

  xbot_msgs::msg::ActionInfo record_dock_action;
  record_dock_action.action_id = "record_dock";
  record_dock_action.enabled = false;
  record_dock_action.action_name = "Record Docking point";

  xbot_msgs::msg::ActionInfo auto_point_collecting_enable_action;
  auto_point_collecting_enable_action.action_id = "auto_point_collecting_enable";
  auto_point_collecting_enable_action.enabled = false;
  auto_point_collecting_enable_action.action_name = "Enable automatic point collecting";

  xbot_msgs::msg::ActionInfo auto_point_collecting_disable_action;
  auto_point_collecting_disable_action.action_id = "auto_point_collecting_disable";
  auto_point_collecting_disable_action.enabled = false;
  auto_point_collecting_disable_action.action_name = "Disable automatic point collecting";

  xbot_msgs::msg::ActionInfo collect_point_action;
  collect_point_action.action_id = "collect_point";
  collect_point_action.enabled = false;
  collect_point_action.action_name = "Collect point";

  xbot_msgs::msg::ActionInfo start_manual_mowing_action;
  start_manual_mowing_action.action_id = "start_manual_mowing";
  start_manual_mowing_action.enabled = false;
  start_manual_mowing_action.action_name = "Start manual mowing";

  xbot_msgs::msg::ActionInfo stop_manual_mowing_action;
  stop_manual_mowing_action.action_id = "stop_manual_mowing";
  stop_manual_mowing_action.enabled = false;
  stop_manual_mowing_action.action_name = "Stop manual mowing";

  actions.clear();
  actions.push_back(start_recording_action);
  actions.push_back(stop_recording_action);
  actions.push_back(finish_navigation_area_action);
  actions.push_back(finish_mowing_area_action);
  actions.push_back(exit_recording_mode_action);
  actions.push_back(finish_discard_action);
  actions.push_back(record_dock_action);
  actions.push_back(auto_point_collecting_enable_action);
  actions.push_back(auto_point_collecting_disable_action);
  actions.push_back(collect_point_action);
  actions.push_back(start_manual_mowing_action);
  actions.push_back(stop_manual_mowing_action);
}

void AreaRecordingBehavior::update_actions() {
  {
    for (auto& a : actions) {
      a.enabled = false;
    }
    if (has_first_docking_pos) {
      actions[6].enabled = true;
    } else if (poly_recording_enabled) {
      actions[1].enabled = true;
      actions[2].enabled = true;
      actions[3].enabled = true;
      actions[4].enabled = true;
      actions[5].enabled = true;
      actions[7].enabled = !auto_point_collecting;
      actions[8].enabled = auto_point_collecting;
      actions[9].enabled = !auto_point_collecting;
    } else {
      if (has_outline) {
        actions[0].enabled = true;
        actions[2].enabled = true;
        actions[3].enabled = true;
        actions[4].enabled = true;
        actions[5].enabled = true;
      } else {
        actions[0].enabled = true;
        actions[4].enabled = true;
        actions[6].enabled = true;
      }
    }
    actions[10].enabled = !manual_mowing;
    actions[11].enabled = manual_mowing;

    registerActions("mower_logic:area_recording", actions);
  }
}

void AreaRecordingBehavior::record_auto_point_collecting(const std_msgs::msg::Bool::SharedPtr state_msg) {
  if (state_msg->data) {
    RCLCPP_INFO(rosNode->get_logger(), "Recording auto point collecting enabled");
    auto_point_collecting = true;
  } else {
    RCLCPP_INFO(rosNode->get_logger(), "Recording auto point collecting disabled");
    auto_point_collecting = false;
  }
}

void AreaRecordingBehavior::record_collect_point(const std_msgs::msg::Bool::SharedPtr state_msg) {
  if (state_msg->data) {
    RCLCPP_INFO(rosNode->get_logger(), "Recording collect point");
    collect_point = true;
  }
}
