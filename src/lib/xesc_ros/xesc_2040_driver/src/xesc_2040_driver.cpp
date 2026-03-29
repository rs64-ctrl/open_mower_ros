#include "xesc_2040_driver/xesc_2040_driver.h"


void xesc_2040_driver::Xesc2040Driver::error_func(const std::string &s) {
    RCLCPP_ERROR_STREAM(logger_, s);
}


xesc_2040_driver::Xesc2040Driver::Xesc2040Driver(rclcpp::Node::SharedPtr node)
    : node_(node), logger_(node->get_logger()) {
    RCLCPP_INFO_STREAM(logger_, "Starting xesc 2040 driver");

    xesc_interface = new xesc_2040_driver::Xesc2040Interface(std::bind(&Xesc2040Driver::error_func, this, std::placeholders::_1));

    uint8_t hall_table[8];
    float motor_current_limit;
    float acceleration;
    bool has_motor_temp;
    float min_motor_temp;
    float max_motor_temp;
    float min_pcb_temp;
    float max_pcb_temp;
    std::string serial_port;

    serial_port = node->declare_parameter<std::string>("serial_port", "");
    if(serial_port.empty()) {
        RCLCPP_ERROR_STREAM(logger_, "You need to provide parameter serial_port.");
        throw std::runtime_error("You need to provide parameter serial_port.");
    }

    for(int i = 0; i < 8; i++) {
        int tmp;
        std::string param_name = std::string("hall_table_") + std::to_string(i);
        tmp = node->declare_parameter<int>(param_name, -1);
        if (tmp == -1) {
            RCLCPP_ERROR_STREAM(logger_, "You need to provide parameter " << param_name);
            throw std::runtime_error("You need to provide parameter " + param_name);
        }
        hall_table[i] = tmp;
    }

    motor_current_limit = node->declare_parameter<double>("motor_current_limit", -1.0);
    if(motor_current_limit < 0.0) {
        RCLCPP_ERROR_STREAM(logger_, "You need to provide parameter motor_current_limit");
        throw std::runtime_error("You need to provide parameter motor_current_limit");
    }
    acceleration = node->declare_parameter<double>("acceleration", -1.0);
    if(acceleration < 0.0) {
        RCLCPP_ERROR_STREAM(logger_, "You need to provide parameter acceleration");
        throw std::runtime_error("You need to provide parameter acceleration");
    }
    has_motor_temp = node->declare_parameter<bool>("has_motor_temp", false);
    min_motor_temp = node->declare_parameter<double>("min_motor_temp", 0.0);
    max_motor_temp = node->declare_parameter<double>("max_motor_temp", 0.0);
    min_pcb_temp = node->declare_parameter<double>("min_pcb_temp", 0.0);
    max_pcb_temp = node->declare_parameter<double>("max_pcb_temp", 0.0);

    xesc_interface->update_settings(hall_table,
                                    motor_current_limit,
                                    acceleration,
                                    has_motor_temp,
                                    min_motor_temp,
                                    max_motor_temp,
                                    min_pcb_temp,
                                    max_pcb_temp);

    xesc_interface->start(serial_port);
}

void xesc_2040_driver::Xesc2040Driver::getStatus(xesc_msgs::msg::XescStateStamped &state_msg) {
    if(!xesc_interface)
        return;
    xesc_interface->get_status(&status);

    state_msg.header.stamp = node_->get_clock()->now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.voltage_input = status.voltage_input;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.temperature_motor = status.temperature_motor;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.direction = status.direction;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_2040_driver::Xesc2040Driver::getStatusBlocking(xesc_msgs::msg::XescStateStamped &state_msg) {
    if(!xesc_interface)
        return;
    xesc_interface->wait_for_status(&status);

    state_msg.header.stamp = node_->get_clock()->now();
    state_msg.state.connection_state = status.connection_state;
    state_msg.state.fw_major = status.fw_version_major;
    state_msg.state.fw_minor = status.fw_version_minor;
    state_msg.state.voltage_input = status.voltage_input;
    state_msg.state.temperature_pcb = status.temperature_pcb;
    state_msg.state.temperature_motor = status.temperature_motor;
    state_msg.state.current_input = status.current_input;
    state_msg.state.duty_cycle = status.duty_cycle;
    state_msg.state.tacho = status.tacho;
    state_msg.state.tacho_absolute = status.tacho_absolute;
    state_msg.state.direction = status.direction;
    state_msg.state.fault_code = status.fault_code;
}

void xesc_2040_driver::Xesc2040Driver::stop() {
    RCLCPP_INFO_STREAM(logger_, "stopping XESC2040 driver");
    xesc_interface->stop();

    delete xesc_interface;
}

void xesc_2040_driver::Xesc2040Driver::setDutyCycle(float duty_cycle) {
    if(xesc_interface) {
        xesc_interface->setDutyCycle(duty_cycle);
    }
}
