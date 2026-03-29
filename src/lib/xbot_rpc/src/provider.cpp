#include "xbot_rpc/provider.h"

#include "xbot_rpc/srv/register_methods_srv.hpp"

namespace xbot_rpc {

void RpcProvider::init() {
  request_sub = node_->create_subscription<xbot_rpc::msg::RpcRequest>(
      TOPIC_REQUEST, rclcpp::QoS(100),
      std::bind(&RpcProvider::handleRequest, this, std::placeholders::_1));
  response_pub = node_->create_publisher<xbot_rpc::msg::RpcResponse>(TOPIC_RESPONSE, 100);
  error_pub = node_->create_publisher<xbot_rpc::msg::RpcError>(TOPIC_ERROR, 100);
  registration_client = node_->create_client<xbot_rpc::srv::RegisterMethodsSrv>(SERVICE_REGISTER_METHODS);
  registration_client->wait_for_service(std::chrono::seconds(10));
  publishMethods();
}

void RpcProvider::publishMethods() {
  auto request = std::make_shared<xbot_rpc::srv::RegisterMethodsSrv::Request>();
  request->node_id = node_id;
  request->methods.reserve(methods.size());
  for (const auto& [method_id, _] : methods) {
    request->methods.push_back(method_id);
  }
  auto result = registration_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Error registering methods for %s", node_id.c_str());
  }
}

void RpcProvider::handleRequest(const xbot_rpc::msg::RpcRequest::SharedPtr request) {
  // Look up the method. Ignore if not found, as it might be handled by another node.
  auto it = methods.find(request->method);
  if (it == methods.end()) {
    return;
  }

  // Parse the parameters.
  nlohmann::basic_json<> params;
  if (!request->params.empty()) {
    try {
      params = nlohmann::ordered_json::parse(request->params);
    } catch (const nlohmann::json::parse_error& e) {
      publishError(request, msg::RpcError::ERROR_INVALID_JSON, std::string("Invalid parameters JSON: ") + e.what());
      return;
    }
  }

  // Execute the method callback and publish the response.
  try {
    nlohmann::basic_json<> response = it->second(request->method, params);
    publishResponse(request, response);
  } catch (const RpcException& e) {
    publishError(request, e.code, e.message);
  } catch (const std::exception& e) {
    publishError(request, msg::RpcError::ERROR_INTERNAL, std::string("Internal error: ") + e.what());
  }
}

void RpcProvider::publishResponse(const xbot_rpc::msg::RpcRequest::SharedPtr& request,
                                  const nlohmann::basic_json<>& response) {
  if (request->id.empty()) {
    return;
  }
  xbot_rpc::msg::RpcResponse response_msg;
  response_msg.result = response.dump();
  response_msg.id = request->id;
  response_pub->publish(response_msg);
}

void RpcProvider::publishError(const xbot_rpc::msg::RpcRequest::SharedPtr& request, int16_t code, const std::string& message) {
  if (request->id.empty()) {
    return;
  }
  xbot_rpc::msg::RpcError err_msg;
  err_msg.id = request->id;
  err_msg.code = code;
  err_msg.message = message;
  error_pub->publish(err_msg);
}

}  // namespace xbot_rpc
