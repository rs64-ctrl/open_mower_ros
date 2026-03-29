#include "xesc_yfr4_driver/xesc_yfr4_driver.h"

void xesc_yfr4_driver::XescYFR4Driver::error_func(const std::string &s) {
    RCLCPP_ERROR_STREAM(logger_, s);
}

xesc_yfr4_driver::XescYFR4Driver::XescYFR4Driver(rclcpp::Node::SharedPtr node)
    : node_(node), logger_(node->get_logger()) {
    RCLCPP_INFO_STREAM(logger_, "Starting xesc YardForce R4 adapter driver");

    xesc_interface = new xesc_yfr4_driver::XescYFR4Interface(std::bind(&XescYFR4Driver::error_func, this, std::placeholders::_1));

    float motor_current_limit;
    float min_pcb_temp;
    float max_pcb_temp;
    std::string serial_port;

    serial_port = node->declare_parameter<std::string>("serial_port", "");
    if (serial_port.empty()) {
        RCLCPP_ERROR_STREAM(logger_, "You need to provide parameter serial_port.");
        throw std::runtime_error("You need to provide parameter serial_port.");
    }
    motor_current_limit = node->declare_parameter<double>("motor_current_limit", -1.0);
    if (motor_current_limit < 0.0) {
        RCLCPP_ERROR_STREAM(logger_, "You need to provide parameter motor_current_limit");
        throw std::runtime_error("You need to provide parameter motor_current_limit");
    }
    min_pcb_temp = node->declare_parameter<double>("min_pcb_temp", 0.0);
    max_pcb_temp = node->declare_parameter<double>("max_pcb_temp", 0.0);

    xesc_interface->update_settings(motor_current_limit, min_pcb_temp, max_pcb_temp);
    xesc_interface->start(serial_port);
}

void xesc_yfr4_driver::XescYFR4Driver::getStatus(xesc_msgs::msg::XescStateStamped &state_msg) {
    if (!xesc_interface)
        return;

    xesc_interface->get_status(&status);
    state_msg.header.stamp = node_->get_clock()->now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.direction = status.direction;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.rpm = status.rpm;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_yfr4_driver::XescYFR4Driver::getStatusBlocking(xesc_msgs::msg::XescStateStamped &state_msg) {
    if (!xesc_interface)
        return;
    xesc_interface->wait_for_status(&status);
    state_msg.header.stamp = node_->get_clock()->now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.direction = status.direction;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.rpm = status.rpm;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_yfr4_driver::XescYFR4Driver::stop() {
    RCLCPP_INFO_STREAM(logger_, "Stopping xesc YardForce R4 adapter driver");
    xesc_interface->stop();
    delete xesc_interface;
}

void xesc_yfr4_driver::XescYFR4Driver::setDutyCycle(float duty_cycle) {
    if (xesc_interface) {
        xesc_interface->setDutyCycle(duty_cycle);
    }
}
