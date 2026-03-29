//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "rclcpp/rclcpp.hpp"
#include "devices/serial_gps_device.h"
#include "devices/tcp_gps_device.h"
#include "interfaces/ublox_gps_interface.h"
#include "interfaces/nmea_gps_interface.h"
#include "xbot_msgs/msg/wheel_tick.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "xbot_msgs/msg/absolute_pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/u_int32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include <nmeaparse/nmea.h>
#include "GeographicLib/DMS.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <algorithm>
#include <cctype>

using namespace xbot::driver::gps;
using namespace nmea;

class DriverGpsNode : public rclcpp::Node {
public:
    DriverGpsNode() : Node("xbot_driver_gps") {
        // Declare parameters
        this->declare_parameter<std::string>("protocol", "");
        this->declare_parameter<bool>("ubx_mode", false);
        this->declare_parameter<bool>("verbose", false);
        this->declare_parameter<std::string>("device_type", "serial");
        this->declare_parameter<int>("baudrate", 38400);
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<std::string>("tcp_host", "");
        this->declare_parameter<std::string>("tcp_port", "");
        this->declare_parameter<std::string>("filename", "/dev/null");
        this->declare_parameter<std::string>("mode", "absolute");
        this->declare_parameter<double>("datum_lat", 0.0);
        this->declare_parameter<double>("datum_long", 0.0);
        this->declare_parameter<double>("datum_height", 0.0);
        this->declare_parameter<bool>("publish_latency", true);
    }

    bool initialize() {
        std::string protocol;
        bool has_protocol = this->get_parameter("protocol", protocol) && !protocol.empty();
        bool ubx_mode = false;
        this->get_parameter("ubx_mode", ubx_mode);
        bool has_ubx_mode = this->has_parameter("ubx_mode");

        // Check if ubx_mode was actually set by user (not just default)
        // In ROS2, we check if the parameter was set to a non-default value
        // Since we declared ubx_mode with default false, we treat it as "has" if protocol is empty
        if (!has_protocol && !has_ubx_mode) {
            RCLCPP_ERROR(this->get_logger(), "Neither 'protocol' nor 'ubx_mode' parameter provided. Set 'protocol' (e.g., 'UBX' or 'NMEA') or legacy 'ubx_mode'. Exiting.");
            return false;
        }

        if (has_protocol && has_ubx_mode) {
            RCLCPP_WARN(this->get_logger(), "Both 'protocol' and 'ubx_mode' are set. using 'protocol' and ignoring 'ubx_mode'.");
        }

        std::string chosen_protocol;
        if (has_protocol) {
            // Normalize case
            std::transform(protocol.begin(), protocol.end(), protocol.begin(),
                           [](unsigned char c){ return static_cast<char>(std::toupper(c)); });
            chosen_protocol = protocol;
        } else {
            // Backward compatibility with legacy ubx_mode
            chosen_protocol = ubx_mode ? "UBX" : "NMEA";
            RCLCPP_INFO(this->get_logger(), "Using legacy 'ubx_mode' to select protocol: %s", chosen_protocol.c_str());
        }

        if(chosen_protocol == "UBX") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Using UBX mode for GPS");
            gpsInterface_ = new UbxGpsInterface();
            isUbxInterface_ = true;
        } else if (chosen_protocol == "NMEA") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Using NMEA mode for GPS");
            gpsInterface_ = new NmeaGpsInterface();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported protocol '%s'. Supported values: 'UBX', 'NMEA'. Exiting.", chosen_protocol.c_str());
            return false;
        }

        gpsInterface_->set_log_function(
            [this](std::string text, LogLevel level) {
                this->gps_log(text, level);
            }
        );

        this->get_parameter("verbose", allow_verbose_logging_);
        if (allow_verbose_logging_) {
            RCLCPP_WARN(this->get_logger(), "GPS node has verbose logging enabled");
        }

        std::string device_type;
        this->get_parameter("device_type", device_type);
        if (device_type == "serial") {
            SerialGpsDevice *device = new SerialGpsDevice();
            int baudrate;
            this->get_parameter("baudrate", baudrate);
            device->set_baudrate(baudrate);
            std::string serial_port;
            this->get_parameter("serial_port", serial_port);
            device->set_serial_port(serial_port);
            gpsInterface_->set_device(device);
        } else if (device_type == "tcp") {
            TcpGpsDevice *device = new TcpGpsDevice();
            std::string tcp_host, tcp_port;
            this->get_parameter("tcp_host", tcp_host);
            this->get_parameter("tcp_port", tcp_port);
            device->set_host(tcp_host);
            device->set_port(tcp_port);
            gpsInterface_->set_device(device);
        } else if (device_type == "file") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Reading GPS data from file!");
            std::string filename;
            this->get_parameter("filename", filename);
            gpsInterface_->set_file_name(filename);
        } else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid device type");
            return false;
        }

        std::string mode;
        this->get_parameter("mode", mode);
        if (mode == "absolute") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Using absolute mode for GPS");
            gpsInterface_->set_mode(xbot::driver::gps::GpsInterface::ABSOLUTE);
            double datum_lat, datum_long, datum_height;
            bool has_datum = true;
            has_datum &= this->get_parameter("datum_lat", datum_lat) && datum_lat != 0.0;
            has_datum &= this->get_parameter("datum_long", datum_long) && datum_long != 0.0;
            has_datum &= this->get_parameter("datum_height", datum_height);
            if (!has_datum) {
                RCLCPP_ERROR_STREAM(this->get_logger(),
                        "You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
                return false;
            }
            gpsInterface_->set_datum(datum_lat, datum_long, datum_height);
        } else if (mode == "relative") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Using relative mode for GPS");
            gpsInterface_->set_mode(xbot::driver::gps::GpsInterface::RELATIVE);
        }

        // Create publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovariance>("~/pose", 10);
        xbot_pose_pub_ = this->create_publisher<xbot_msgs::msg::AbsolutePose>("~/xb_pose", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
        vrs_nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("/nmea", 10);

        // Create subscribers
        wheel_tick_sub_ = this->create_subscription<xbot_msgs::msg::WheelTick>(
            "~/wheel_ticks", rclcpp::SensorDataQoS(),
            [this](const xbot_msgs::msg::WheelTick::SharedPtr msg) {
                this->wheel_tick_received(msg);
            }
        );
        rtcm_sub_ = this->create_subscription<rtcm_msgs::msg::Message>(
            "rtcm", rclcpp::SensorDataQoS(),
            [this](const rtcm_msgs::msg::Message::SharedPtr msg) {
                this->rtcm_received(msg);
            }
        );

        gpsInterface_->set_state_callback(
            [this](const GpsInterface::GpsState &state) {
                this->gps_state_received(state);
            }
        );

        bool publish_latency;
        this->get_parameter("publish_latency", publish_latency);
        if (publish_latency && isUbxInterface_) {
            latency_pub1_ = this->create_publisher<std_msgs::msg::UInt32>("~/wheel_tick_stamp_esc", 100);
            latency_pub2_ = this->create_publisher<std_msgs::msg::UInt32>("~/wheel_tick_ublox_rx", 100);
            latency_pub3_ = this->create_publisher<std_msgs::msg::UInt32>("~/wheel_tick_round_trip_host", 100);
            dynamic_cast<UbxGpsInterface*>(gpsInterface_)->set_wheel_latency_callback(
                [this](uint32_t wheel_tick_stamp, uint32_t wheel_tick_stamp_ublox, uint32_t wheel_tick_round_trip_stamp) {
                    this->wheel_latency_received(wheel_tick_stamp, wheel_tick_stamp_ublox, wheel_tick_round_trip_stamp);
                }
            );
        }
        gpsInterface_->set_imu_callback(
            [this](const GpsInterface::ImuState &state) {
                this->imu_received(state);
            }
        );

        if (!gpsInterface_->start()) {
            return false;
        }

        return true;
    }

    ~DriverGpsNode() {
        if (gpsInterface_) {
            gpsInterface_->stop();
            delete gpsInterface_;
        }
    }

private:
    void gps_log(std::string text, LogLevel level) {
        switch (level) {
            case VERBOSE:
                if (!allow_verbose_logging_) {
                    return;
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "[driver_gps] " << text);
                break;
            case INFO:
                RCLCPP_INFO_STREAM(this->get_logger(), "[driver_gps] " << text);
                break;
            case WARN:
                RCLCPP_WARN_STREAM(this->get_logger(), "[driver_gps] " << text);
                break;
            default:
                RCLCPP_ERROR_STREAM(this->get_logger(), "[driver_gps] " << text);
                break;
        }
    }

    void generate_nmea(double lat_in, double lon_in) {
        // only send every 10 seconds, this will be more than needed
        if ((this->get_clock()->now() - last_vrs_feedback_).seconds() < 10.0) {
            return;
        }
        last_vrs_feedback_ = this->get_clock()->now();
        NMEACommand cmd1;

        auto lat = GeographicLib::DMS::Encode(lat_in, GeographicLib::DMS::component::MINUTE, 4,
                                              GeographicLib::DMS::flag::LATITUDE, ';');
        auto lon = GeographicLib::DMS::Encode(lon_in, GeographicLib::DMS::component::MINUTE, 4,
                                              GeographicLib::DMS::flag::LONGITUDE, ';');

        // remove separator char
        boost::erase_all(lat, ";");
        boost::erase_all(lon, ";");

        auto lat_hemisphere = lat.substr(lat.length() - 1, 1);
        auto lon_hemisphere = lon.substr(lon.length() - 1, 1);

        std::stringstream message_ss;
        auto time_facet = new boost::posix_time::time_facet("%H%M%s");

        message_ss.imbue(std::locale(message_ss.getloc(), time_facet));
        // Use current time converted to boost ptime
        auto now = this->get_clock()->now();
        auto now_ns = now.nanoseconds();
        auto now_sec = now_ns / 1000000000LL;
        auto boost_time = boost::posix_time::from_time_t(static_cast<time_t>(now_sec));
        message_ss << boost_time << "," <<
                   lat.substr(0, lat.length() - 1) << "," <<
                   lat_hemisphere << "," <<
                   lon.substr(0, lon.length() - 1) << "," <<
                   lon_hemisphere << ",1,8,0,0,M,0,M,0000,";

        //build message
        cmd1.name = "GPGGA";
        cmd1.message = message_ss.str();

        nmea_msgs::msg::Sentence vrs_msg;
        vrs_msg.header.frame_id = "gps";
        vrs_msg.header.stamp = this->get_clock()->now();
        vrs_msg.sentence = cmd1.toString();
        boost::erase_all(vrs_msg.sentence, "\r\n");
        vrs_nmea_pub_->publish(vrs_msg);
    }

    void wheel_tick_received(const xbot_msgs::msg::WheelTick::SharedPtr msg) {
        // Limit frequency
        rclcpp::Time stamp(msg->stamp);
        if ((stamp - last_wheel_tick_time_).seconds() < 0.09)
            return;
        // drop if not ubx
        if(!isUbxInterface_)
            return;
        ((UbxGpsInterface*)gpsInterface_)->send_wheel_ticks(
            static_cast<uint32_t>(stamp.nanoseconds() / 1000000),
            msg->wheel_direction_rl,
            msg->wheel_ticks_rl / 10,
            msg->wheel_direction_rr, msg->wheel_ticks_rr / 10);
        last_wheel_tick_time_ = stamp;
    }

    void rtcm_received(const rtcm_msgs::msg::Message::SharedPtr rtcm) {
        gpsInterface_->send_rtcm(rtcm->message.data(), rtcm->message.size());
    }

    void convert_gps_result(const GpsInterface::GpsState &state, xbot_msgs::msg::AbsolutePose &result) {
        result.header.frame_id = "gps";
        result.header.stamp = this->get_clock()->now();

        result.source = xbot_msgs::msg::AbsolutePose::SOURCE_GPS;
        result.flags = 0;
        result.sensor_stamp = state.sensor_time;
        result.received_stamp = state.received_time;
        switch (state.rtk_type) {
            case GpsInterface::GpsState::RTK_FLOAT:
                result.flags = xbot_msgs::msg::AbsolutePose::FLAG_GPS_RTK | xbot_msgs::msg::AbsolutePose::FLAG_GPS_RTK_FLOAT;
                break;
            case GpsInterface::GpsState::RTK_FIX:
                result.flags = xbot_msgs::msg::AbsolutePose::FLAG_GPS_RTK | xbot_msgs::msg::AbsolutePose::FLAG_GPS_RTK_FIXED;
                break;
            default:
                result.flags = 0;
        }

        if (state.fix_type == GpsInterface::GpsState::FixType::DR_ONLY ||
            state.fix_type == GpsInterface::GpsState::FixType::GNSS_DR_COMBINED) {
            result.flags |= xbot_msgs::msg::AbsolutePose::FLAG_GPS_DEAD_RECKONING;
        }

        result.orientation_valid = state.vehicle_heading_valid;
        result.motion_vector_valid = true;
        result.position_accuracy = state.position_accuracy;
        result.orientation_accuracy = state.vehicle_heading_accuracy;

        double heading = state.vehicle_heading_valid ? state.vehicle_heading : state.motion_heading;
        double headingAcc = state.vehicle_heading_valid ? state.vehicle_heading_accuracy : state.motion_heading_accuracy;

        result.pose.pose.position.x = state.pos_e;
        result.pose.pose.position.y = state.pos_n;
        result.pose.pose.position.z = state.pos_u;

        tf2::Quaternion q_mag;
        q_mag.setRPY(0.0, 0.0, heading);
        result.pose.pose.orientation = tf2::toMsg(q_mag);

        result.pose.covariance = {
                pow(state.position_accuracy, 2), 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, pow(state.position_accuracy, 2), 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, pow(state.position_accuracy, 2), 0.0, 0.0,
                0.0, 0.0, 0.0, 10000.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, pow(headingAcc, 2)
        };

        result.motion_vector.x = state.vel_e;
        result.motion_vector.y = state.vel_n;
        result.motion_vector.z = state.vel_u;

        result.vehicle_heading = state.vehicle_heading;
        result.motion_heading = state.motion_heading;
    }

    void gps_state_received(const GpsInterface::GpsState &state) {
        // new state received, publish
        xbot_msgs::msg::AbsolutePose pose_result;
        convert_gps_result(state, pose_result);
        xbot_pose_pub_->publish(pose_result);
        pose_pub_->publish(pose_result.pose);

        // send feedback to VRS
        generate_nmea(state.pos_lat, state.pos_lon);
    }

    void wheel_latency_received(uint32_t wheel_tick_stamp, uint32_t wheel_tick_stamp_ublox,
                                uint32_t wheel_tick_round_trip_stamp) {
        std_msgs::msg::UInt32 latency_msg1, latency_msg2, latency_msg3;
        latency_msg1.data = wheel_tick_stamp;
        latency_msg2.data = wheel_tick_stamp_ublox;
        latency_msg3.data = wheel_tick_round_trip_stamp;
        latency_pub1_->publish(latency_msg1);
        latency_pub2_->publish(latency_msg2);
        latency_pub3_->publish(latency_msg3);
    }

    void imu_received(const GpsInterface::ImuState &state) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "gps";
        imu_msg.angular_velocity.x = state.gx;
        imu_msg.angular_velocity.y = state.gy;
        imu_msg.angular_velocity.z = state.gz;
        imu_msg.linear_acceleration.x = state.ax;
        imu_msg.linear_acceleration.y = state.ay;
        imu_msg.linear_acceleration.z = state.az;
        imu_pub_->publish(imu_msg);
    }

    // Members
    bool isUbxInterface_ = false;
    GpsInterface *gpsInterface_ = nullptr;
    bool allow_verbose_logging_ = false;

    rclcpp::Time last_wheel_tick_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_vrs_feedback_{0, 0, RCL_ROS_TIME};

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_pub_;
    rclcpp::Publisher<xbot_msgs::msg::AbsolutePose>::SharedPtr xbot_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr latency_pub1_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr latency_pub2_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr latency_pub3_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr vrs_nmea_pub_;

    // Subscribers
    rclcpp::Subscription<xbot_msgs::msg::WheelTick>::SharedPtr wheel_tick_sub_;
    rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DriverGpsNode>();

    if (!node->initialize()) {
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
