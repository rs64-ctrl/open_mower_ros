// Created by Clemens Elflein on 22.02.22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// OpenMower. If not, see <https://www.gnu.org/licenses/>.
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "mower_map/srv/get_mowing_area_srv.hpp"
#include "slic3r_coverage_planner/srv/plan_path.hpp"

class PlannerTestNode : public rclcpp::Node {
 public:
  PlannerTestNode() : Node("planner_test") {
    this->declare_parameter<int>("area_index", 0);
    this->declare_parameter<int>("outline_count", 4);

    int area_index = this->get_parameter("area_index").as_int();
    int outline_count = this->get_parameter("outline_count").as_int();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mower_logic/mowing_path", rclcpp::QoS(100).transient_local());

    path_client_ = this->create_client<slic3r_coverage_planner::srv::PlanPath>("slic3r_coverage_planner/plan_path");
    map_client_ = this->create_client<mower_map::srv::GetMowingAreaSrv>("mower_map_service/get_mowing_area");

    RCLCPP_INFO(this->get_logger(), "Waiting for map server");
    if (!map_client_->wait_for_service(std::chrono::seconds(60))) {
      RCLCPP_ERROR(this->get_logger(), "Map server service not found.");
      return;
    }

    auto map_request = std::make_shared<mower_map::srv::GetMowingAreaSrv::Request>();
    map_request->index = area_index;

    auto map_result = map_client_->async_send_request(map_request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), map_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Error loading mowing area");
      return;
    }

    auto map_response = map_result.get();

    auto path_request = std::make_shared<slic3r_coverage_planner::srv::PlanPath::Request>();
    path_request->angle = 0;
    path_request->outline_count = outline_count;
    path_request->outline = map_response->area.area;
    path_request->holes = map_response->area.obstacles;
    path_request->fill_type = slic3r_coverage_planner::srv::PlanPath::Request::FILL_LINEAR;
    path_request->distance = 0.13;
    path_request->outer_offset = 0.05;

    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this, path_request]() {
      auto result = path_client_->async_send_request(path_request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Error getting path area");
      } else {
        RCLCPP_INFO(this->get_logger(), "Got path");
      }
    });
  }

 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Client<slic3r_coverage_planner::srv::PlanPath>::SharedPtr path_client_;
  rclcpp::Client<mower_map::srv::GetMowingAreaSrv>::SharedPtr map_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
