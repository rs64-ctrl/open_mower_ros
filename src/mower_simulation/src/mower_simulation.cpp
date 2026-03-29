// Created by Clemens Elflein on 2/18/22, 5:37 PM.
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

// Include messages for mower control
#include <mower_msgs/msg/esc_status.hpp>
#include <mower_msgs/msg/emergency.hpp>
#include <mower_msgs/msg/power.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <xbot-service/Io.hpp>
#include <xbot-service/portable/system.hpp>

#include "../../../services/service_ids.h"
#include "SimRobot.h"
#include "mower_map/srv/get_docking_point_srv.hpp"
#include "services/diff_drive_service/diff_drive_service.hpp"
#include "services/emergency_service/emergency_service.hpp"
#include "services/gps_service/gps_service.hpp"
#include "services/imu_service/imu_service.hpp"
#include "services/mower_service/mower_service.hpp"
#include "services/power_service/power_service.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("mower_simulation");

  // Declare parameters that were previously in dynamic_reconfigure
  node->declare_parameter<double>("battery_voltage", 28.0);
  node->declare_parameter<double>("temperature_mower", 40.0);
  node->declare_parameter<bool>("is_charging", false);
  node->declare_parameter<bool>("mower_error", false);
  node->declare_parameter<bool>("mower_running", false);
  node->declare_parameter<bool>("wheels_stalled", false);
  node->declare_parameter<bool>("emergency_stop", false);
  node->declare_parameter<bool>("rain", false);

  auto docking_point_client = node->create_client<mower_map::srv::GetDockingPointSrv>(
      "mower_map_service/get_docking_point");

  xbot::service::system::initSystem();
  xbot::service::Io::start();

  SimRobot robot{node};

  // Move the robot to the docking station.
  // TODO: Use a better way to make sure that the docking position is loaded.
  rclcpp::sleep_for(std::chrono::seconds(3));
  auto request = std::make_shared<mower_map::srv::GetDockingPointSrv::Request>();
  auto future = docking_point_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
    const auto& response = future.get();
    const auto& docking_pose = response->docking_pose;
    tf2::Quaternion quat;
    tf2::fromMsg(docking_pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    robot.SetDockingPose(docking_pose.position.x, docking_pose.position.y, yaw);
    robot.SetPosition(docking_pose.position.x, docking_pose.position.y, yaw);
  }

  EmergencyService emergency_service{xbot::service_ids::EMERGENCY, robot};
  DiffDriveService diff_drive_service{xbot::service_ids::DIFF_DRIVE, robot};
  MowerService mower_service{xbot::service_ids::MOWER, robot};
  ImuService imu_service{xbot::service_ids::IMU, robot};
  PowerService power_service{xbot::service_ids::POWER, robot};
  GpsService gps_service{xbot::service_ids::GPS, robot};

  emergency_service.start();
  diff_drive_service.start();
  mower_service.start();
  imu_service.start();
  power_service.start();
  gps_service.start();

  robot.Start();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
