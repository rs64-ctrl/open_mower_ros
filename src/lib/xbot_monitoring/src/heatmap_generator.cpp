//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include "xbot_msgs/msg/sensor_info.hpp"
#include "xbot_msgs/msg/sensor_data_double.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include "grid_map_core/GridMap.hpp"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include "xbot_msgs/msg/map_size.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <boost/algorithm/algorithm.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

rclcpp::Node::SharedPtr node;

xbot_msgs::msg::AbsolutePose last_pose{};
std::unordered_map<std::string, std::pair<grid_map::GridMap, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>> pubsub_map{};
xbot_msgs::msg::MapSize lastMap{};
bool has_map = false;
void onMapSize(const xbot_msgs::msg::MapSize::SharedPtr mapInfo) {
    if(!has_map || lastMap.map_center_y != mapInfo->map_center_y || lastMap.map_center_x != mapInfo->map_center_x || lastMap.map_width != mapInfo->map_width || lastMap.map_height != mapInfo->map_height) {
        RCLCPP_INFO(node->get_logger(), "Updated heatmap geometry.");
        for(auto & [key,value] : pubsub_map) {
                value.first.setGeometry(grid_map::Length(mapInfo->map_width, mapInfo->map_height), 0.2,
                                        grid_map::Position(mapInfo->map_center_x, mapInfo->map_center_y));
        }
        has_map = true;
        lastMap = *mapInfo;
    }
}

void onSensorData(const std::string sensor_id, const xbot_msgs::msg::SensorDataDouble::SharedPtr msg) {
    auto & [map, publisher] = pubsub_map[sensor_id];
    try {
        map.atPosition("intensity",
                       grid_map::Position(last_pose.pose.pose.position.x, last_pose.pose.pose.position.y)) = msg->data;
        map.atPosition("elevation",
                       grid_map::Position(last_pose.pose.pose.position.x, last_pose.pose.pose.position.y)) = 0;
    } catch (std::exception &e) {
        // error inserting into map, skip it
        RCLCPP_WARN(node->get_logger(), "Could not add point to heatmap: %s", e.what());
        return;
    }

    sensor_msgs::msg::PointCloud2 mapmsg;
    grid_map::GridMapRosConverter::toPointCloud(map, map.getLayers(), "elevation", mapmsg);
    publisher->publish(mapmsg);
}

void onRobotPos(const xbot_msgs::msg::AbsolutePose::SharedPtr msg) {
    last_pose = *msg;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("heatmap_generator");

    std::vector<std::string> sensor_ids;
    std::string sensor_ids_string;

    node->declare_parameter<std::string>("sensor_ids", "");
    sensor_ids_string = node->get_parameter("sensor_ids").as_string();
    if(sensor_ids_string.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Please specify sensor_ids for heatmap generation as comma separated list.");
        return 1;
    }

    boost::erase_all(sensor_ids_string, " ");
    boost::split ( sensor_ids, sensor_ids_string, boost::is_any_of(","));

    // Subscribe to robot position
    auto stateSubscriber = node->create_subscription<xbot_msgs::msg::AbsolutePose>(
        "xbot_positioning/xb_pose", 10, onRobotPos);

    // Subscribe to map info
    auto mapSubscriber = node->create_subscription<xbot_msgs::msg::MapSize>(
        "mower_map_service/map_size", 1, onMapSize);

    std::vector<rclcpp::SubscriptionBase::SharedPtr> sensorSubscribers{};
    for(const auto & sensor_id : sensor_ids)
    {
        RCLCPP_INFO(node->get_logger(), "Generating heatmap for sensor id: %s", sensor_id.c_str());
        // Add layer to map
        grid_map::GridMap map{};
        map.add("intensity");
        map.add("elevation");
        map.setFrameId("map");
        // Create a publisher for the heatmap
        auto p = node->create_publisher<sensor_msgs::msg::PointCloud2>("heatmap/"+sensor_id, 10);
        // Subscribe to sensor data
        auto s = node->create_subscription<xbot_msgs::msg::SensorDataDouble>(
            "xbot_monitoring/sensors/" + sensor_id + "/data", 10,
            [sensor_id](const xbot_msgs::msg::SensorDataDouble::SharedPtr msg) {
                onSensorData(sensor_id, msg);
            });
        // store in map, so that they don't get deconstructed
        pubsub_map[sensor_id] = std::make_pair(std::move(map), std::move(p));
        sensorSubscribers.push_back(std::move(s));
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
