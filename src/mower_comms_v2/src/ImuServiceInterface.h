//
// Created by clemens on 26.07.24.
//

#ifndef IMUSERVICEINTERFACE_H
#define IMUSERVICEINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <ImuServiceInterfaceBase.hpp>

class ImuServiceInterface : public ImuServiceInterfaceBase {
 public:
  ImuServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                      const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& imu_publisher,
                      const std::string& axis_config,
                      const rclcpp::Node::SharedPtr& node)
      : ImuServiceInterfaceBase(service_id, ctx),
        node_(node),
        imu_publisher_(imu_publisher),
        axis_config_(axis_config) {
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnAxesChanged(const double* new_value, uint32_t length) override;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  std::string axis_config_;

  sensor_msgs::msg::Imu imu_msg{};
  bool validateAxisConfig();
};

#endif  // IMUSERVICEINTERFACE_H
