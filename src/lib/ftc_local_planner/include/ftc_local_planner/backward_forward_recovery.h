#ifndef BACKWARD_FORWARD_RECOVERY_H
#define BACKWARD_FORWARD_RECOVERY_H

#include "ftc_local_planner/costmap_controller_interface.h"

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace ftc_local_planner
{

class BackwardForwardRecovery : public mbf_costmap_core::CostmapRecovery
{
public:
    BackwardForwardRecovery();

    void initialize(
        std::string name,
        const rclcpp::Node::SharedPtr & node,
        tf2_ros::Buffer * tf,
        nav2_costmap_2d::Costmap2DROS * global_costmap,
        nav2_costmap_2d::Costmap2DROS * local_costmap) override;

    uint32_t runBehavior(std::string & message) override;

    bool cancel() override;

private:
    bool attemptMove(double distance, bool forward);
    bool isPositionValid(double x, double y);

    std::string name_;
    bool initialized_;
    bool cancelled_;
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer * tf_;
    nav2_costmap_2d::Costmap2DROS * global_costmap_;
    nav2_costmap_2d::Costmap2DROS * local_costmap_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    double max_distance_;
    double linear_vel_;
    double check_frequency_;
    unsigned char max_cost_threshold_;
    double timeout_seconds_;
};

}  // namespace ftc_local_planner

#endif  // BACKWARD_FORWARD_RECOVERY_H
