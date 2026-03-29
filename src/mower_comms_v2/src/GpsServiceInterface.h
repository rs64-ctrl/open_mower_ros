//
// Created by clemens on 30.11.24.
//

#ifndef GPSSERVICEINTERFACE_H
#define GPSSERVICEINTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <xbot_msgs/msg/absolute_pose.hpp>
#include <nmea_msgs/msg/sentence.hpp>

#include <GpsServiceInterfaceBase.hpp>

class GpsServiceInterface : public GpsServiceInterfaceBase {
 public:
  GpsServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                      const rclcpp::Publisher<xbot_msgs::msg::AbsolutePose>::SharedPtr& absolute_pose_publisher,
                      const rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr& nmea_publisher,
                      double datum_lat, double datum_long, double datum_height,
                      uint32_t baud_rate, const std::string& protocol, uint8_t port_index,
                      bool absolute_coords,
                      const rclcpp::Node::SharedPtr& node);

  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnPositionChanged(const double* new_value, uint32_t length) override;
  void OnPositionHorizontalAccuracyChanged(const double& new_value) override;
  void OnFixTypeChanged(const char* new_value, uint32_t length) override;
  void OnMotionVectorENUChanged(const double* new_value, uint32_t length) override;
  void OnMotionHeadingAndAccuracyChanged(const double* new_value, uint32_t length) override;
  void OnVehicleHeadingAndAccuracyChanged(const double* new_value, uint32_t length) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<xbot_msgs::msg::AbsolutePose>::SharedPtr absolute_pose_publisher_;
  rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_publisher_;

  std::string protocol_;
  uint32_t baud_rate_;
  uint8_t port_index_;
  bool absolute_coords_;

  xbot_msgs::msg::AbsolutePose pose_msg_{};
  double datum_e_, datum_n_, datum_u_;
  std::string datum_zone_;
  void SendNMEA(double lat_in, double lon_in);
};

#endif  // GPSSERVICEINTERFACE_H
