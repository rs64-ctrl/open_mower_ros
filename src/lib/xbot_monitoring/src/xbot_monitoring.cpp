//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <regex>
#include "xbot_msgs/msg/sensor_info.hpp"
#include "xbot_msgs/msg/sensor_data_string.hpp"
#include "xbot_msgs/msg/sensor_data_double.hpp"
#include "xbot_msgs/msg/robot_state.hpp"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "xbot_msgs/srv/register_actions_srv.hpp"
#include "xbot_msgs/msg/action_info.hpp"
#include "xbot_msgs/msg/map_overlay.hpp"
#include "xbot_rpc/msg/rpc_error.hpp"
#include "xbot_rpc/msg/rpc_request.hpp"
#include "xbot_rpc/msg/rpc_response.hpp"
#include "xbot_rpc/constants.h"
#include "xbot_rpc/provider.h"
#include "xbot_rpc/srv/register_methods_srv.hpp"
#include "capabilities.h"

using json = nlohmann::ordered_json;

// Forward declarations
void publish_capabilities();
void publish_sensor_metadata();
void publish_map();
void publish_map_overlay();
void publish_actions();
void publish_version();
void publish_params();
void rpc_request_callback(const std::string &payload);

// Stores registered actions (prefix to vector<action>)
std::map<std::string, std::vector<xbot_msgs::msg::ActionInfo>> registered_actions;

// Stores registered RPC methods
std::map<std::string, std::vector<std::string>> registered_methods;
std::mutex registered_methods_mutex;

// Maps a topic to a subscriber.
std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> active_subscribers;
std::map<std::string, xbot_msgs::msg::SensorInfo> found_sensors;
std::vector<rclcpp::SubscriptionBase::SharedPtr> sensor_data_subscribers;

rclcpp::Node::SharedPtr node;

// The MQTT Client
std::shared_ptr<mqtt::async_client> client_;
std::shared_ptr<mqtt::async_client> client_external_;

std::mutex mqtt_callback_mutex;

// Publisher for cmd_vel and commands
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_pub;
rclcpp::Publisher<xbot_rpc::msg::RpcRequest>::SharedPtr rpc_request_pub;

// properties for external mqtt
bool external_mqtt_enable = false;
std::string external_mqtt_username = "";
std::string external_mqtt_password = "";
std::string external_mqtt_hostname = "";
std::string external_mqtt_topic_prefix = "";
std::string external_mqtt_port = "";
std::string version_string = "";

class MqttCallback : public mqtt::callback {

    void connected(const mqtt::string &string) override {
        RCLCPP_INFO(node->get_logger(), "MQTT Connected");
        publish_capabilities();
        publish_sensor_metadata();
        publish_map();
        publish_map_overlay();
        publish_actions();
        publish_version();
        publish_params();

        // BEGIN: Deprecated code (1/2)
        // Earlier implementations subscribed to "/action" and "prefix//action" topics, we do it to not break stuff as well.
        client_->subscribe(this->mqtt_topic_prefix + "/teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/action", 0);
        // END: Deprecated code (1/2)

        client_->subscribe(this->mqtt_topic_prefix + "teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "action", 0);
        client_->subscribe(this->mqtt_topic_prefix + "rpc/request", 0);
    }

public:
    void setMqttClient(std::shared_ptr<mqtt::async_client> c, const std::string &mqtt_topic_prefix) {
        this->client_ = std::move(c);
        this->mqtt_topic_prefix = mqtt_topic_prefix;
    }
    void message_arrived(mqtt::const_message_ptr ptr) override {
        if(ptr->get_topic() == this->mqtt_topic_prefix + "teleop") {
            try {
                json json = json::from_bson(ptr->get_payload().begin(), ptr->get_payload().end());
                geometry_msgs::msg::Twist t;
                t.linear.x = json["vx"];
                t.angular.z = json["vz"];
                cmd_vel_pub->publish(t);
            } catch (const json::exception &e) {
                RCLCPP_ERROR(node->get_logger(), "Error decoding teleop bson: %s", e.what());
            }
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "action") {
            RCLCPP_INFO(node->get_logger(), "Got action: %s", ptr->get_payload().c_str());
            std_msgs::msg::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub->publish(action_msg);
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "/action") {
            // BEGIN: Deprecated code (2/2)
            RCLCPP_WARN(node->get_logger(), "Got action on deprecated topic! Change your topic names!: %s", ptr->get_payload().c_str());
            std_msgs::msg::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub->publish(action_msg);
            // END: Deprecated code (2/2)
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "rpc/request") {
          std::string payload = ptr->get_payload_str();
          rpc_request_callback(payload);
        }
    }
private:
    std::shared_ptr<mqtt::async_client> client_;
    std::string mqtt_topic_prefix = "";
};

MqttCallback mqtt_callback;
MqttCallback mqtt_callback_external;

json map_json;
json map_overlay_json;
bool has_map = false;
bool has_map_overlay = false;

// RPC provider will be initialized in main after node creation
std::unique_ptr<xbot_rpc::RpcProvider> rpc_provider;

void setupMqttClient() {
    // setup mqtt client for app use
    {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + "127.0.0.1" +
                          std::string(":") + std::to_string(1883);

        try {
            client_ = std::make_shared<mqtt::async_client>(
                    uri, "xbot_monitoring");
            mqtt_callback.setMqttClient(client_, "");
            client_->set_callback(mqtt_callback);

            client_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
    // setup external mqtt client
    if(external_mqtt_enable) {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        if(!external_mqtt_username.empty()) {
            connect_options_.set_user_name(external_mqtt_username);
            connect_options_.set_password(external_mqtt_password);
        }

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + external_mqtt_hostname +
                          std::string(":") + external_mqtt_port;

        try {
            client_external_ = std::make_shared<mqtt::async_client>(
                    uri, "ext_xbot_monitoring");
            mqtt_callback_external.setMqttClient(client_external_, external_mqtt_topic_prefix);
            client_external_->set_callback(mqtt_callback_external);

            client_external_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "External Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
}

void try_publish(std::string topic, std::string data, bool retain = false) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, 1, true);
        } else {
            client_->publish(topic, data);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
    // publish external
    if(external_mqtt_enable) {
        try {
            if (retain) {
                // QOS 1 so that the data actually arrives at the client at least once.
                client_external_->publish(external_mqtt_topic_prefix + topic, data, 1, true);
            } else {
                client_external_->publish(external_mqtt_topic_prefix + topic, data);
            }
        } catch (const mqtt::exception &e) {
            // client disconnected or something, we drop it.
        }
    }
}

void try_publish_binary(std::string topic, const void *data, size_t size, bool retain = false) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, size, 1, true);
        } else {
            client_->publish(topic, data, size);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
}

void publish_version() {
    json version = {
            {"version", version_string}
    };
    try_publish("version/json", version.dump(), true);
    auto bson = json::to_bson(version);
    try_publish_binary("version", bson.data(), bson.size(), true);
}

void publish_capabilities() {
  try_publish("capabilities/json", CAPABILITIES.dump(2), true);
}

// In ROS2 there is no XmlRpc parameter server. Parameters are node-local.
// We publish all declared parameters as JSON.
void publish_params() {
    // Get all parameter names from this node
    auto param_names = node->list_parameters({}, 0).names;
    std::sort(param_names.begin(), param_names.end());

    json params = json::object();
    for (const auto &name : param_names) {
        if (name.find("password") != std::string::npos) {
            params[name] = nullptr;
            continue;
        }
        rclcpp::Parameter param;
        if (node->get_parameter(name, param)) {
            switch (param.get_type()) {
                case rclcpp::ParameterType::PARAMETER_BOOL:
                    params[name] = param.as_bool();
                    break;
                case rclcpp::ParameterType::PARAMETER_INTEGER:
                    params[name] = param.as_int();
                    break;
                case rclcpp::ParameterType::PARAMETER_DOUBLE:
                    params[name] = param.as_double();
                    break;
                case rclcpp::ParameterType::PARAMETER_STRING:
                    params[name] = param.as_string();
                    break;
                case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
                    json arr = json::array();
                    for (auto v : param.as_bool_array()) arr.push_back(v);
                    params[name] = arr;
                    break;
                }
                case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
                    json arr = json::array();
                    for (auto v : param.as_integer_array()) arr.push_back(v);
                    params[name] = arr;
                    break;
                }
                case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
                    json arr = json::array();
                    for (auto v : param.as_double_array()) arr.push_back(v);
                    params[name] = arr;
                    break;
                }
                case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
                    json arr = json::array();
                    for (const auto &v : param.as_string_array()) arr.push_back(v);
                    params[name] = arr;
                    break;
                }
                case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY: {
                    // Encode as base64
                    const auto& data = param.as_byte_array();
                    static const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                    std::string out;
                    out.reserve(((data.size() + 2) / 3) * 4);
                    for (size_t i = 0; i < data.size(); i += 3) {
                        unsigned int n = (static_cast<unsigned char>(data[i]) << 16)
                            | (i + 1 < data.size() ? static_cast<unsigned char>(data[i + 1]) << 8 : 0)
                            | (i + 2 < data.size() ? static_cast<unsigned char>(data[i + 2]) : 0);
                        out += b64[(n >> 18) & 0x3F];
                        out += b64[(n >> 12) & 0x3F];
                        out += (i + 1 < data.size()) ? b64[(n >> 6) & 0x3F] : '=';
                        out += (i + 2 < data.size()) ? b64[n & 0x3F] : '=';
                    }
                    params[name] = out;
                    break;
                }
                default:
                    params[name] = nullptr;
                    break;
            }
        }
    }
    try_publish("params/json", params.dump(), true);
}

void publish_sensor_metadata() {
    std::unique_lock<std::mutex> lk(mqtt_callback_mutex);

    if(found_sensors.empty())
        return;

    json sensor_info;
    for (const auto &kv: found_sensors) {
        json info;
        info["sensor_id"] = kv.second.sensor_id;
        info["sensor_name"] = kv.second.sensor_name;

        switch (kv.second.value_type) {
            case xbot_msgs::msg::SensorInfo::TYPE_STRING: {
                info["value_type"] = "STRING";
                break;
            }
            case xbot_msgs::msg::SensorInfo::TYPE_DOUBLE: {
                info["value_type"] = "DOUBLE";
                break;
            }
            default: {
                info["value_type"] = "UNKNOWN";
                break;
            }


        }

        switch (kv.second.value_description) {
            case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE: {
                info["value_description"] = "TEMPERATURE";
                break;
            }
            case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_VELOCITY: {
                info["value_description"] = "VELOCITY";
                break;
            }
            case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_ACCELERATION: {
                info["value_description"] = "ACCELERATION";
                break;
            }
            case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_VOLTAGE: {
                info["value_description"] = "VOLTAGE";
                break;
            }
            case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_CURRENT: {
                info["value_description"] = "CURRENT";
                break;
            }
            case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_PERCENT: {
                info["value_description"] = "PERCENT";
                break;
            }
            case xbot_msgs::msg::SensorInfo::VALUE_DESCRIPTION_RPM: {
                info["value_description"] = "REVOLUTIONS";
                break;
            }
            default: {
                info["value_description"] = "UNKNOWN";
                break;
            }
        }

        info["unit"] = kv.second.unit;
        info["has_min_max"] = kv.second.has_min_max;
        info["min_value"] = kv.second.min_value;
        info["max_value"] = kv.second.max_value;
        info["has_critical_low"] = kv.second.has_critical_low;
        info["lower_critical_value"] = kv.second.lower_critical_value;
        info["has_critical_high"] = kv.second.has_critical_high;
        info["upper_critical_value"] = kv.second.upper_critical_value;
        sensor_info.push_back(info);
    }
    try_publish("sensor_infos/json", sensor_info.dump(), true);
    json data;
    data["d"] = sensor_info;
    auto bson = json::to_bson(data);
    try_publish_binary("sensor_infos/bson", bson.data(), bson.size(), true);
}

void subscribe_to_sensor(std::string topic) {
    auto &sensor = found_sensors[topic];

    RCLCPP_INFO(node->get_logger(), "Subscribing to sensor data for sensor with name: %s", sensor.sensor_name.c_str());

    std::string data_topic = "xbot_monitoring/sensors/" + sensor.sensor_id + "/data";

    switch (sensor.value_type) {
        case xbot_msgs::msg::SensorInfo::TYPE_DOUBLE: {
            auto s = node->create_subscription<xbot_msgs::msg::SensorDataDouble>(data_topic, 10, [&info = sensor](
                    const xbot_msgs::msg::SensorDataDouble::SharedPtr msg) {
                try_publish("sensors/" + info.sensor_id + "/data", std::to_string(msg->data));

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        case xbot_msgs::msg::SensorInfo::TYPE_STRING: {
            auto s = node->create_subscription<xbot_msgs::msg::SensorDataString>(data_topic, 10, [&info = sensor](
                    const xbot_msgs::msg::SensorDataString::SharedPtr msg) {
                try_publish("sensors/" + info.sensor_id + "/data", msg->data);

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        default: {
            RCLCPP_ERROR(node->get_logger(), "Invalid Sensor Data Type: %d", (int) sensor.value_type);
        }
    }
}

void robot_state_callback(const xbot_msgs::msg::RobotState::SharedPtr msg) {
    // Build a JSON and publish it
    json j;

    j["battery_percentage"] = msg->battery_percentage;
    j["gps_percentage"] = msg->gps_percentage;
    j["current_action_progress"] = msg->current_action_progress;
    j["current_state"] = msg->current_state;
    j["current_sub_state"] = msg->current_sub_state;
    j["current_area"] = msg->current_area;
    j["current_path"] = msg->current_path;
    j["current_path_index"] = msg->current_path_index;
    j["emergency"] = msg->emergency;
    j["is_charging"] = msg->is_charging;
    j["rain_detected"] = msg->rain_detected;
    j["pose"]["x"] = msg->robot_pose.pose.pose.position.x;
    j["pose"]["y"] = msg->robot_pose.pose.pose.position.y;
    j["pose"]["heading"] = msg->robot_pose.vehicle_heading;
    j["pose"]["pos_accuracy"] = msg->robot_pose.position_accuracy;
    j["pose"]["heading_accuracy"] = msg->robot_pose.orientation_accuracy;
    j["pose"]["heading_valid"] = msg->robot_pose.orientation_valid;

    try_publish("robot_state/json", j.dump());
    json data;
    data["d"] = j;
    auto bson = json::to_bson(data);
    try_publish_binary("robot_state/bson", bson.data(), bson.size());
}

void publish_actions() {
    json actions = json::array();
    for(const auto &kv : registered_actions) {
        for(const auto &action : kv.second) {
            json action_info;
            action_info["action_id"] = kv.first + "/" + action.action_id;
            action_info["action_name"] = action.action_name;
            action_info["enabled"] = action.enabled;
            actions.push_back(action_info);
        }
    }

    try_publish("actions/json", actions.dump(), true);
    json data;
    data["d"] = actions;

    auto bson = json::to_bson(data);
    try_publish_binary("actions/bson", bson.data(), bson.size(), true);
}

void publish_map() {
    if(!has_map)
        return;
    try_publish("map/json", map_json.dump(2), true);
    json data;
    data["d"] = map_json;
    auto bson = json::to_bson(data);
    try_publish_binary("map/bson", bson.data(), bson.size(), true);
}

void publish_map_overlay() {
    if(!has_map_overlay)
        return;
    try_publish("map_overlay/json", map_overlay_json.dump(), true);
    json data;
    data["d"] = map_overlay_json;
    auto bson = json::to_bson(data);
    try_publish_binary("map_overlay/bson", bson.data(), bson.size(), true);
}

void map_callback(const std_msgs::msg::String::SharedPtr msg) {
    try {
        map_json = json::parse(msg->data);
        has_map = true;
        publish_map();
    } catch (const json::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Error processing map JSON: %s", e.what());
    }
}


void map_overlay_callback(const xbot_msgs::msg::MapOverlay::SharedPtr msg) {
    // Build a JSON and publish it

    json polys;
    for(const auto &poly : msg->polygons) {
        if(poly.polygon.points.size() < 2)
            continue;
        json poly_j;
        {
            json outline_poly_j;
            for (const auto &pt: poly.polygon.points) {
                json p_j;
                p_j["x"] = pt.x;
                p_j["y"] = pt.y;
                outline_poly_j.push_back(p_j);
            }
            poly_j["poly"] = outline_poly_j;
            poly_j["is_closed"] = poly.closed;
            poly_j["line_width"] = poly.line_width;
            poly_j["color"] = poly.color;
        }
        polys.push_back(poly_j);
    }

    json j;
    j["polygons"] = polys;
    map_overlay_json = j;
    has_map_overlay = true;

    publish_map_overlay();
}


void registerActions(const std::shared_ptr<xbot_msgs::srv::RegisterActionsSrv::Request> req,
                     std::shared_ptr<xbot_msgs::srv::RegisterActionsSrv::Response> res) {

    RCLCPP_INFO(node->get_logger(), "new actions registered: %s registered %zu actions.",
                req->node_prefix.c_str(), req->actions.size());

    registered_actions[req->node_prefix] = req->actions;

    publish_actions();
}

void rpc_publish_error(const int16_t code, const std::string &message, const nlohmann::basic_json<> &id = nullptr) {
    json err_resp = {{"jsonrpc", "2.0"},
                       {"error", {{"code", code}, {"message", message}}},
                       {"id", id}};
    try_publish("rpc/response", err_resp.dump(2));
}

void rpc_request_callback(const std::string &payload) {
    // Parse
    json req;
    try {
      req = json::parse(payload);
    } catch (const json::parse_error &e) {
      return rpc_publish_error(xbot_rpc::msg::RpcError::ERROR_INVALID_JSON, "Could not parse request JSON");
    }

    // Validate
    if (!req.is_object()) {
        return rpc_publish_error(xbot_rpc::msg::RpcError::ERROR_INVALID_REQUEST, "Request is not a JSON object");
    }
    json id = req.contains("id") ? req["id"] : nullptr;
    if (id != nullptr && !id.is_string()) {
        return rpc_publish_error(xbot_rpc::msg::RpcError::ERROR_INVALID_REQUEST, "ID is not a string", id);
    } else if (!req.contains("jsonrpc") || !req["jsonrpc"].is_string() || req["jsonrpc"] != "2.0") {
        return rpc_publish_error(xbot_rpc::msg::RpcError::ERROR_INVALID_REQUEST, "Invalid JSON-RPC version");
    } else if (!req.contains("method") || !req["method"].is_string()) {
        return rpc_publish_error(xbot_rpc::msg::RpcError::ERROR_INVALID_REQUEST, "Method is not a string", req["id"]);
    }

    // Check if the method is registered
    const std::string method = req["method"];
    if (method.compare(0, 5, "meta.") == 0) {
      // Silently ignore methods that are handled by the meta service.
      return;
    }
    bool is_registered = false;
    {
        std::lock_guard<std::mutex> lk(registered_methods_mutex);
        for (const auto& [_, method_ids] : registered_methods) {
            if (std::find(method_ids.begin(), method_ids.end(), method) != method_ids.end()) {
                is_registered = true;
                break;
            }
        }
    }
    if (!is_registered) {
        return rpc_publish_error(xbot_rpc::msg::RpcError::ERROR_METHOD_NOT_FOUND, "Method \"" + method + "\" not found", req["id"]);
    }

    // Forward to the providers as ROS message
    xbot_rpc::msg::RpcRequest msg;
    msg.method = method;
    msg.params = req.contains("params") ? req["params"].dump() : "";
    msg.id = id != nullptr ? id : "";
    rpc_request_pub->publish(msg);
}

void rpc_response_callback(const xbot_rpc::msg::RpcResponse::SharedPtr msg) {
    json result;
    try {
        result = json::parse(msg->result);
    } catch (const json::parse_error &e) {
        return rpc_publish_error(xbot_rpc::msg::RpcError::ERROR_INTERNAL, "Internal error while parsing result JSON: " + std::string(e.what()), msg->id);
    }

    json j = {{"jsonrpc", "2.0"}, {"result", result}, {"id", msg->id}};
    try_publish("rpc/response", j.dump(2));
}

void rpc_error_callback(const xbot_rpc::msg::RpcError::SharedPtr msg) {
    rpc_publish_error(msg->code, msg->message, msg->id);
}

void register_methods(const std::shared_ptr<xbot_rpc::srv::RegisterMethodsSrv::Request> req,
                      std::shared_ptr<xbot_rpc::srv::RegisterMethodsSrv::Response> res) {
    std::lock_guard<std::mutex> lk(registered_methods_mutex);
    registered_methods[req->node_id] = req->methods;
    RCLCPP_INFO(node->get_logger(), "new methods registered: %s registered %zu methods.",
                req->node_id.c_str(), req->methods.size());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    has_map = false;
    has_map_overlay = false;

    node = std::make_shared<rclcpp::Node>("xbot_monitoring");

    node->declare_parameter<std::string>("software_version", "UNKNOWN VERSION");
    node->declare_parameter<bool>("external_mqtt_enable", false);
    node->declare_parameter<std::string>("external_mqtt_topic_prefix", "");
    node->declare_parameter<std::string>("external_mqtt_hostname", "");
    node->declare_parameter<int>("external_mqtt_port", 1883);
    node->declare_parameter<std::string>("external_mqtt_username", "");
    node->declare_parameter<std::string>("external_mqtt_password", "");

    version_string = node->get_parameter("software_version").as_string();
    if(version_string.empty()) {
        version_string = "UNKNOWN VERSION";
    }

    external_mqtt_enable = node->get_parameter("external_mqtt_enable").as_bool();
    external_mqtt_topic_prefix = node->get_parameter("external_mqtt_topic_prefix").as_string();
    if(!external_mqtt_topic_prefix.empty() && external_mqtt_topic_prefix.back() != '/') {
        // append the /
        external_mqtt_topic_prefix = external_mqtt_topic_prefix+"/";
    }

    external_mqtt_hostname = node->get_parameter("external_mqtt_hostname").as_string();
    external_mqtt_port = std::to_string(node->get_parameter("external_mqtt_port").as_int());
    external_mqtt_username = node->get_parameter("external_mqtt_username").as_string();
    external_mqtt_password = node->get_parameter("external_mqtt_password").as_string();

    if(external_mqtt_enable) {
        RCLCPP_INFO(node->get_logger(), "Using external MQTT broker: %s:%s with topic prefix: %s",
                    external_mqtt_hostname.c_str(), external_mqtt_port.c_str(), external_mqtt_topic_prefix.c_str());
    }

    // First setup MQTT
    setupMqttClient();

    auto register_action_service = node->create_service<xbot_msgs::srv::RegisterActionsSrv>(
        "xbot/register_actions", registerActions);

    auto robotStateSubscriber = node->create_subscription<xbot_msgs::msg::RobotState>(
        "xbot_monitoring/robot_state", 10, robot_state_callback);
    auto mapSubscriber = node->create_subscription<std_msgs::msg::String>(
        "mower_map_service/json_map", 10, map_callback);
    auto mapOverlaySubscriber = node->create_subscription<xbot_msgs::msg::MapOverlay>(
        "xbot_monitoring/map_overlay", 10, map_overlay_callback);

    cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("xbot_monitoring/remote_cmd_vel", 1);
    action_pub = node->create_publisher<std_msgs::msg::String>("xbot/action", 1);

    rpc_request_pub = node->create_publisher<xbot_rpc::msg::RpcRequest>(xbot_rpc::TOPIC_REQUEST, 100);
    auto rpc_response_sub = node->create_subscription<xbot_rpc::msg::RpcResponse>(
        xbot_rpc::TOPIC_RESPONSE, 100, rpc_response_callback);
    auto rpc_error_sub = node->create_subscription<xbot_rpc::msg::RpcError>(
        xbot_rpc::TOPIC_ERROR, 100, rpc_error_callback);
    auto register_methods_service = node->create_service<xbot_rpc::srv::RegisterMethodsSrv>(
        xbot_rpc::SERVICE_REGISTER_METHODS, register_methods);

    // Create RPC provider (needs node to be created first)
    rpc_provider = std::make_unique<xbot_rpc::RpcProvider>(node, "xbot_monitoring", std::map<std::string, xbot_rpc::callback_t>{
        RPC_METHOD("rpc.ping", {
            return "pong";
        }),
        RPC_METHOD("rpc.methods", {
            std::lock_guard<std::mutex> lk(registered_methods_mutex);
            json methods = json::array();
            for (const auto& [_, method_ids] : registered_methods) {
                for (const auto& method_id : method_ids) {
                    methods.push_back(method_id);
                }
            }
            std::sort(methods.begin(), methods.end());
            return methods;
        }),
    });
    rpc_provider->init();

    // Use a multi-threaded executor to allow callbacks while the timer runs
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::regex topic_regex("/xbot_monitoring/sensors/.*/info");

    // Create a wall timer to periodically check for new sensor topics (replaces the ROS1 loop)
    auto sensor_check_timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [&topic_regex]() {
            // In ROS2, use the node's graph interface to discover topics
            auto topic_names_and_types = node->get_topic_names_and_types();
            for (const auto& [topic_name, types] : topic_names_and_types) {
                if (!std::regex_match(topic_name, topic_regex) || active_subscribers.count(topic_name) != 0)
                    continue;

                RCLCPP_INFO(node->get_logger(), "Found new sensor topic %s", topic_name.c_str());
                active_subscribers[topic_name] = node->create_subscription<xbot_msgs::msg::SensorInfo>(
                    topic_name, 1, [topic = topic_name](const xbot_msgs::msg::SensorInfo::SharedPtr msg) {
                        RCLCPP_INFO(node->get_logger(), "Got sensor info for sensor on topic %s on topic %s",
                                    msg->sensor_name.c_str(), topic.c_str());
                        auto exist = found_sensors.count(topic);

                        // Sensor already known and sensor-info equals?
                        if(exist != 0 && found_sensors[topic] == *msg)
                            return;

                        {
                            // Sensor is new or sensor-info differ from the buffered one
                            std::unique_lock<std::mutex> lk(mqtt_callback_mutex);
                            found_sensors[topic] = *msg;  // Save the (new|changed) sensor info
                        }

                        // Let the info subscription alive for dynamic threshold changes
                        //active_subscribers.erase(topic);  // Stop subscribing to infos

                        if (exist == 0) {
                            subscribe_to_sensor(topic);  // Subscribe for data
                        }

                        // Republish (new|changed) sensor info
                        // NOTE: If a sensor name or id changes, the related data topic wouldn't change!
                        //       But do we dynamically change a sensor name or id?
                        publish_sensor_metadata();
                    }
                );
            }
        }
    );

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
