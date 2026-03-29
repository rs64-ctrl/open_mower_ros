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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "xbot_msgs/msg/absolute_pose.hpp"

class XbotPoseConverterNode : public rclcpp::Node {
 public:
  XbotPoseConverterNode() : Node("xbot_pose_converter") {
    this->declare_parameter<std::string>("topic", "");
    this->declare_parameter<std::string>("frame", "frame");

    std::string topic;
    if (!this->get_parameter("topic", topic) || topic.empty()) {
      RCLCPP_ERROR(this->get_logger(), "You need to provide a topic to convert");
      return;
    }
    this->get_parameter("frame", frame_);

    std::string target_topic = topic + "/converted";

    RCLCPP_INFO(this->get_logger(), "Converting %s to %s", topic.c_str(), target_topic.c_str());

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(target_topic, 10);

    sub_ = this->create_subscription<xbot_msgs::msg::AbsolutePose>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&XbotPoseConverterNode::pose_received, this, std::placeholders::_1));
  }

 private:
  void pose_received(const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.pose = msg->pose;
    out.pose.pose.position.z = 0;
    out.header.frame_id = frame_;
    pose_pub_->publish(out);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<xbot_msgs::msg::AbsolutePose>::SharedPtr sub_;
  std::string frame_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XbotPoseConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
