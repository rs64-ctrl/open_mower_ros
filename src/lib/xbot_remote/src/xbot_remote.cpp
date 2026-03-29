//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <boost/regex.hpp>
#include "xbot_msgs/msg/sensor_info.hpp"
#include "xbot_msgs/msg/sensor_data_string.hpp"
#include "xbot_msgs/msg/sensor_data_double.hpp"
#include "xbot_msgs/msg/robot_state.hpp"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "websocketpp/server.hpp"
#include <websocketpp/config/asio_no_tls.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;

using json = nlohmann::json;

std::shared_ptr<rclcpp::Node> node;

// Publisher for cmd_vel
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

// Create a server endpoint
server echo_server;

// Define a callback to handle incoming messages
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
    (void)s;
    (void)hdl;
    try {
        json json = json::from_bson(msg->get_payload());

        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500, "vx:%f vr: %f", (double)json["vx"], (double)json["vz"]);
        geometry_msgs::msg::Twist t;
        t.linear.x = json["vx"];
        t.angular.z = json["vz"];
        cmd_vel_pub->publish(t);
    } catch (std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception during remote decoding: %s", e.what());
    }
}

void* server_thread(void* arg) {
    (void)arg;
    try {
        // Set logging settings
        echo_server.set_access_channels(websocketpp::log::alevel::all);
        echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        // Initialize Asio
        echo_server.init_asio();

        // Register our message handler
        echo_server.set_message_handler(std::bind(&on_message,&echo_server,std::placeholders::_1,std::placeholders::_2));

        echo_server.set_reuse_addr(true);

        // Listen on port 9002
        echo_server.listen(9002);

        // Start the server accept loop
        echo_server.start_accept();

        // Start the ASIO io_service run loop
        while(rclcpp::ok()) {
            echo_server.run_one();
        }
    } catch (websocketpp::exception const & e) {
        std::cout << e.what() << std::endl;
        exit(1);
    } catch (...) {
        std::cout << "other exception" << std::endl;
        exit(1);
    }
    return nullptr;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("xbot_remote");

    cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("xbot_remote/cmd_vel", 1);

    pthread_t server_thread_handle;
    pthread_create(&server_thread_handle, nullptr, &server_thread, nullptr);

    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Stopping websocket server");
    echo_server.stop();

    RCLCPP_INFO(node->get_logger(), "Waiting for server thread to shutdown");
    pthread_join(server_thread_handle, nullptr);
    RCLCPP_INFO(node->get_logger(), "Server shut down");

    rclcpp::shutdown();

    return 0;
}
