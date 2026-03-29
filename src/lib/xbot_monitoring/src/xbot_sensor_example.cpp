//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include <rclcpp/rclcpp.hpp>
#include "xbot_msgs/msg/sensor_info.hpp"
#include "xbot_msgs/msg/sensor_data_double.hpp"

xbot_msgs::msg::SensorInfo my_info;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    my_info.has_critical_high = false;
    my_info.has_critical_low = false;
    my_info.has_min_max = true;
    my_info.min_value = -5;
    my_info.max_value = 105.0;
    my_info.value_type = xbot_msgs::msg::SensorInfo::TYPE_DOUBLE;
    my_info.value_description = xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    my_info.unit = "deg C";
    my_info.sensor_id = "some_temperature";
    my_info.sensor_name = "Some Temperature";

    auto node = std::make_shared<rclcpp::Node>("xbot_sensor_example");

    auto sensor_info_publisher = node->create_publisher<xbot_msgs::msg::SensorInfo>(
        "xbot_monitoring/sensors/" + my_info.sensor_id + "/info", rclcpp::QoS(1).transient_local());
    auto sensor_data_publisher = node->create_publisher<xbot_msgs::msg::SensorDataDouble>(
        "xbot_monitoring/sensors/" + my_info.sensor_id + "/data", 1);
    sensor_info_publisher->publish(my_info);

    xbot_msgs::msg::SensorDataDouble data;

    rclcpp::Rate sensorRate(1.0);
    int i = 0;
    while(rclcpp::ok()) {
        // Generate some data here
        data.stamp = node->get_clock()->now();
        data.data = (sin((i++) / 10.0) + 0.5) * 50.0;

        // Publish the data
        sensor_data_publisher->publish(data);
        rclcpp::spin_some(node);
        sensorRate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
