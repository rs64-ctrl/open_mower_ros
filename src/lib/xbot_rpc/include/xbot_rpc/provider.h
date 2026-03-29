#pragma once

#include <rclcpp/rclcpp.hpp>
#include <xbot_rpc/msg/rpc_error.hpp>
#include <xbot_rpc/msg/rpc_request.hpp>
#include <xbot_rpc/msg/rpc_response.hpp>
#include <xbot_rpc/srv/register_methods_srv.hpp>
#include <xbot_rpc/constants.h>

#include <nlohmann/json.hpp>

#define RPC_METHOD(id, body) \
  { id, [](const std::string& method, const nlohmann::basic_json<>& params) body }

namespace xbot_rpc {

typedef std::function<nlohmann::basic_json<>(const std::string& method, const nlohmann::basic_json<>& params)> callback_t;

class RpcException : public std::exception {
 public:
  const int16_t code;
  const std::string message;

  RpcException(int16_t code, const std::string& message)
      : code(code), message(message) {
  }

  const char* what() const noexcept override {
    return message.c_str();
  }
};

class RpcProvider {
 private:
  std::string node_id;
  std::map<std::string, callback_t> methods;
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<xbot_rpc::msg::RpcRequest>::SharedPtr request_sub;
  rclcpp::Publisher<xbot_rpc::msg::RpcResponse>::SharedPtr response_pub;
  rclcpp::Publisher<xbot_rpc::msg::RpcError>::SharedPtr error_pub;
  rclcpp::Client<xbot_rpc::srv::RegisterMethodsSrv>::SharedPtr registration_client;

  void handleRequest(const xbot_rpc::msg::RpcRequest::SharedPtr request);
  void publishResponse(const xbot_rpc::msg::RpcRequest::SharedPtr& request, const nlohmann::basic_json<>& response);
  void publishError(const xbot_rpc::msg::RpcRequest::SharedPtr& request, int16_t code, const std::string& message);

 public:
  RpcProvider(rclcpp::Node::SharedPtr node, const std::string& node_id, const std::map<std::string, callback_t>& methods = {})
      : node_id(node_id), methods(methods), node_(node) {}

  void init();

  void addMethod(const std::string& id, callback_t callback) {
    methods.emplace(id, callback);
  }

  void addMethod(const std::pair<std::string, callback_t>& method) {
    methods.insert(method);
  }

  void publishMethods();
};

}  // namespace xbot_rpc
