#include <ftc_local_planner/backward_forward_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ftc_local_planner
{

BackwardForwardRecovery::BackwardForwardRecovery()
  : initialized_(false),
    cancelled_(false),
    max_distance_(0.5),
    linear_vel_(0.3),
    check_frequency_(10.0),
    max_cost_threshold_(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 10),
    timeout_seconds_(3.0) {}

void BackwardForwardRecovery::initialize(
    std::string name,
    const rclcpp::Node::SharedPtr & node,
    tf2_ros::Buffer * tf,
    nav2_costmap_2d::Costmap2DROS * global_costmap,
    nav2_costmap_2d::Costmap2DROS * local_costmap)
{
    if (!initialized_) {
        name_ = name;
        node_ = node;
        tf_ = tf;
        global_costmap_ = global_costmap;
        local_costmap_ = local_costmap;

        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

        node_->declare_parameter(name_ + ".max_distance", max_distance_);
        node_->declare_parameter(name_ + ".linear_vel", linear_vel_);
        node_->declare_parameter(name_ + ".check_frequency", check_frequency_);
        node_->declare_parameter(name_ + ".max_cost_threshold",
            static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 10));
        node_->declare_parameter(name_ + ".timeout", timeout_seconds_);

        max_distance_ = node_->get_parameter(name_ + ".max_distance").as_double();
        linear_vel_ = node_->get_parameter(name_ + ".linear_vel").as_double();
        check_frequency_ = node_->get_parameter(name_ + ".check_frequency").as_double();
        int temp_threshold = node_->get_parameter(name_ + ".max_cost_threshold").as_int();
        max_cost_threshold_ = static_cast<unsigned char>(temp_threshold);
        timeout_seconds_ = node_->get_parameter(name_ + ".timeout").as_double();

        initialized_ = true;
    } else {
        RCLCPP_ERROR(node_->get_logger(),
            "You should not call initialize twice on this object, doing nothing");
    }
}

uint32_t BackwardForwardRecovery::runBehavior(std::string & message)
{
    if (!initialized_)
    {
        message = "This object must be initialized before runBehavior is called";
        RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
        return 100;  // FAILURE
    }

    cancelled_ = false;

    RCLCPP_WARN(node_->get_logger(), "Running Backward/Forward recovery behavior");

    if (attemptMove(max_distance_, false)) {
        RCLCPP_INFO(node_->get_logger(), "Successfully moved backwards");
        message = "Successfully moved backwards";
        return 0;  // SUCCESS
    }

    if (cancelled_) {
        message = "Recovery cancelled";
        return 101;  // CANCELED
    }

    if (attemptMove(max_distance_, true)) {
        RCLCPP_INFO(node_->get_logger(), "Successfully moved forwards");
        message = "Successfully moved forwards";
        return 0;  // SUCCESS
    }

    if (cancelled_) {
        message = "Recovery cancelled";
        return 101;  // CANCELED
    }

    RCLCPP_WARN(node_->get_logger(),
        "Backward/Forward recovery behavior failed to move in either direction");
    message = "Failed to move in either direction";
    return 100;  // FAILURE
}

bool BackwardForwardRecovery::cancel()
{
    cancelled_ = true;
    return true;
}

bool BackwardForwardRecovery::attemptMove(double distance, bool forward)
{
    geometry_msgs::msg::PoseStamped start_pose;
    local_costmap_->getRobotPose(start_pose);

    rclcpp::Rate rate(check_frequency_);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = forward ? linear_vel_ : -linear_vel_;

    double moved_distance = 0.0;
    rclcpp::Time start_time = node_->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_seconds_);

    while (moved_distance < distance && (node_->now() - start_time) < timeout && !cancelled_)
    {
        geometry_msgs::msg::PoseStamped current_pose;
        local_costmap_->getRobotPose(current_pose);

        moved_distance = std::hypot(
            current_pose.pose.position.x - start_pose.pose.position.x,
            current_pose.pose.position.y - start_pose.pose.position.y
        );

        if (!isPositionValid(current_pose.pose.position.x, current_pose.pose.position.y))
        {
            RCLCPP_WARN(node_->get_logger(),
                "Reached maximum allowed cost after moving %.2f meters", moved_distance);
            cmd_vel.linear.x = 0;
            cmd_vel_pub_->publish(cmd_vel);
            return false;
        }

        cmd_vel_pub_->publish(cmd_vel);
        rate.sleep();
    }

    cmd_vel.linear.x = 0;
    cmd_vel_pub_->publish(cmd_vel);

    if (moved_distance >= distance) {
        RCLCPP_INFO(node_->get_logger(),
            "%s movement completed successfully", forward ? "Forward" : "Backward");
        return true;
    } else {
        RCLCPP_WARN(node_->get_logger(),
            "%s movement timed out after %.2f seconds", forward ? "Forward" : "Backward", timeout_seconds_);
        return false;
    }
}

bool BackwardForwardRecovery::isPositionValid(double x, double y)
{
    unsigned int mx, my;
    if (local_costmap_->getCostmap()->worldToMap(x, y, mx, my))
    {
        unsigned char cost = local_costmap_->getCostmap()->getCost(mx, my);
        return (cost <= max_cost_threshold_);
    }
    return false;
}

}  // namespace ftc_local_planner

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::BackwardForwardRecovery, mbf_costmap_core::CostmapRecovery)
