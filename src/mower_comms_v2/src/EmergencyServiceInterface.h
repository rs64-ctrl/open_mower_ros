//
// Created by clemens on 25.07.24.
//

#ifndef EMERGENCYSERVICEINTERFACE_H
#define EMERGENCYSERVICEINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <mower_msgs/msg/emergency.hpp>

#include <EmergencyServiceInterfaceBase.hpp>

namespace sc = std::chrono;

class EmergencyServiceInterface : public EmergencyServiceInterfaceBase {
 public:
  EmergencyServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                            const rclcpp::Publisher<mower_msgs::msg::Emergency>::SharedPtr& publisher,
                            const rclcpp::Node::SharedPtr& node)
      : EmergencyServiceInterfaceBase(service_id, ctx), publisher_(publisher), node_(node) {
  }

  bool SetEmergency(bool new_value);
  void Heartbeat();

 protected:
  void OnEmergencyActiveChanged(const uint8_t& new_value) override;
  void OnEmergencyLatchChanged(const uint8_t& new_value) override;
  void OnEmergencyReasonChanged(const char* new_value, uint32_t length) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(uint16_t service_id) override;

  void PublishEmergencyState();

  std::recursive_mutex state_mutex_{};

  rclcpp::Publisher<mower_msgs::msg::Emergency>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;

  // keep track of high level emergency
  bool latched_emergency_ = true;
  bool active_low_level_emergency_ = true;
  bool active_high_level_emergency_ = true;
  std::string latest_emergency_reason_ = "NONE";
};

#endif  // EMERGENCYSERVICEINTERFACE_H
