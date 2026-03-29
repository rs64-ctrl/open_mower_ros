//
// Created by clemens on 25.07.24.
//

#ifndef MOWERSERVICEINTERFACE_H
#define MOWERSERVICEINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <mower_msgs/msg/status.hpp>

#include <MowerServiceInterfaceBase.hpp>

class MowerServiceInterface : public MowerServiceInterfaceBase {
 public:
  MowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const rclcpp::Node::SharedPtr& node,
                        const rclcpp::Publisher<mower_msgs::msg::Status>::SharedPtr& status_publisher)
      : MowerServiceInterfaceBase(service_id, ctx), node_(node), status_publisher_(status_publisher) {
  }

  void SetMowerEnabled(bool enabled);

  void Tick();

 protected:
  void OnMowerStatusChanged(const uint8_t& new_value) override;
  void OnRainDetectedChanged(const uint8_t& new_value) override;
  void OnMowerRunningChanged(const uint8_t& new_value) override;
  void OnMowerESCTemperatureChanged(const float& new_value) override;
  void OnMowerMotorTemperatureChanged(const float& new_value) override;
  void OnMowerMotorCurrentChanged(const float& new_value) override;
  void OnMowerMotorRPMChanged(const float& new_value) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;

 private:
  rclcpp::Node::SharedPtr node_;
  mower_msgs::msg::Status status_msg_{};
  rclcpp::Publisher<mower_msgs::msg::Status>::SharedPtr status_publisher_;
};

#endif  // MOWERSERVICEINTERFACE_H
