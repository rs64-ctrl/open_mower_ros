#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <xesc_msgs/msg/xesc_state_stamped.hpp>
#include "xesc_driver/xesc_driver.h"

class XescDriverNode : public rclcpp::Node
{
public:
    XescDriverNode()
        : Node("xesc_driver_node")
    {
    }

    void init()
    {
        xesc_driver_ptr_ = std::make_unique<xesc_driver::XescDriver>(shared_from_this());

        state_pub_ = this->create_publisher<xesc_msgs::msg::XescStateStamped>("sensors/core", 10);

        duty_cycle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "~/duty_cycle", rclcpp::QoS(0).best_effort(),
            std::bind(&XescDriverNode::velReceived, this, std::placeholders::_1));
    }

    void run()
    {
        xesc_msgs::msg::XescStateStamped state_msg;
        while (rclcpp::ok()) {
            xesc_driver_ptr_->getStatusBlocking(state_msg);
            state_pub_->publish(state_msg);
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "stopping XESC driver node");
        xesc_driver_ptr_->stop();
    }

private:
    void velReceived(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!xesc_driver_ptr_)
            return;
        xesc_driver_ptr_->setDutyCycle(msg->data);
    }

    std::unique_ptr<xesc_driver::XescDriver> xesc_driver_ptr_;
    rclcpp::Publisher<xesc_msgs::msg::XescStateStamped>::SharedPtr state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr duty_cycle_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<XescDriverNode>();
    node->init();

    // Use a multi-threaded executor so callbacks fire while run() blocks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Spin in a background thread
    auto spin_thread = std::thread([&executor]() {
        executor.spin();
    });

    node->run();

    rclcpp::shutdown();
    spin_thread.join();

    return 0;
}
