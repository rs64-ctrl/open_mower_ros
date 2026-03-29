//
// Created by clemens on 25.07.24.
//

#ifndef DIFFDRIVESERVICEINTERFACE_H
#define DIFFDRIVESERVICEINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mower_msgs/msg/esc_status.hpp>

#include <DiffDriveServiceInterfaceBase.hpp>

class DiffDriveServiceInterface : public DiffDriveServiceInterfaceBase {
 public:
  DiffDriveServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                            const rclcpp::Node::SharedPtr& node,
                            const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& actual_twist_publisher,
                            const rclcpp::Publisher<mower_msgs::msg::ESCStatus>::SharedPtr& left_esc_status_publisher,
                            const rclcpp::Publisher<mower_msgs::msg::ESCStatus>::SharedPtr& right_esc_status_publisher,
                            double ticks_per_meter, double wheel_distance)
      : DiffDriveServiceInterfaceBase(service_id, ctx),
        node_(node),
        actual_twist_publisher_(actual_twist_publisher),
        left_esc_status_publisher_(left_esc_status_publisher),
        right_esc_status_publisher_(right_esc_status_publisher),
        wheel_distance_(wheel_distance),
        ticks_per_meter_(ticks_per_meter) {
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

  /**
   * Convenience function to transmit the twist from a ROS2 message
   * @param msg The ROS2 message
   */
  void SendTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);

 protected:
  /**
   * Callback whenever an updated twist arrives
   * @param new_value the updated value
   * @param length length of array
   */
  void OnActualTwistChanged(const double* new_value, uint32_t length) override;
  void OnLeftESCTemperatureChanged(const float& new_value) override;
  void OnLeftESCCurrentChanged(const float& new_value) override;
  void OnRightESCTemperatureChanged(const float& new_value) override;
  void OnRightESCCurrentChanged(const float& new_value) override;
  void OnWheelTicksChanged(const uint32_t* new_value, uint32_t length) override;
  void OnLeftESCStatusChanged(const uint8_t& new_value) override;
  void OnRightESCStatusChanged(const uint8_t& new_value) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(uint16_t service_id) override;

 private:
  std::mutex state_mutex_{};
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr actual_twist_publisher_;
  rclcpp::Publisher<mower_msgs::msg::ESCStatus>::SharedPtr left_esc_status_publisher_;
  rclcpp::Publisher<mower_msgs::msg::ESCStatus>::SharedPtr right_esc_status_publisher_;

  double wheel_distance_;
  double ticks_per_meter_;

  // Store the latest ESC state
  mower_msgs::msg::ESCStatus left_esc_state_{};
  mower_msgs::msg::ESCStatus right_esc_state_{};
};

#endif  // DIFFDRIVESERVICEINTERFACE_H
