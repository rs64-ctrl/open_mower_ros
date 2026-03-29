//
// Created by clemens on 09.09.24.
//

#ifndef POWERSERVICEINTERFACE_H
#define POWERSERVICEINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <mower_msgs/msg/power.hpp>

#include <PowerServiceInterfaceBase.hpp>

class PowerServiceInterface : public PowerServiceInterfaceBase {
 public:
  PowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const rclcpp::Publisher<mower_msgs::msg::Power>::SharedPtr& status_publisher,
                        float battery_full_voltage, float battery_empty_voltage,
                        float battery_critical_voltage, float battery_critical_high_voltage,
                        float battery_charge_current,
                        const rclcpp::Node::SharedPtr& node)
      : PowerServiceInterfaceBase(service_id, ctx),
        node_(node),
        status_publisher_(status_publisher),
        battery_full_voltage_(battery_full_voltage),
        battery_empty_voltage_(battery_empty_voltage),
        battery_critical_voltage_(battery_critical_voltage),
        battery_critical_high_voltage_(battery_critical_high_voltage),
        battery_charge_current_(battery_charge_current) {
  }

 protected:
  void OnChargeVoltageChanged(const float& new_value) override;
  void OnChargeCurrentChanged(const float& new_value) override;
  void OnBatteryVoltageChanged(const float& new_value) override;
  void OnChargingStatusChanged(const char* new_value, uint32_t length) override;
  void OnChargerEnabledChanged(const uint8_t& new_value) override;
  bool OnConfigurationRequested(uint16_t service_id) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;

  rclcpp::Node::SharedPtr node_;
  mower_msgs::msg::Power power_msg_{};
  rclcpp::Publisher<mower_msgs::msg::Power>::SharedPtr status_publisher_;

  float battery_full_voltage_;
  float battery_empty_voltage_;
  float battery_critical_voltage_;
  float battery_critical_high_voltage_;
  float battery_charge_current_;
};

#endif  // POWERSERVICEINTERFACE_H
