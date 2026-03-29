/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#include "mbf_costmap_nav/costmap_controller_execution.h"

namespace mbf_costmap_nav
{

CostmapControllerExecution::CostmapControllerExecution(
    const std::string &controller_name,
    const mbf_costmap_core::CostmapController::Ptr &controller_ptr,
    const mbf_utility::RobotInformation::ConstPtr& robot_info,
    const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr &vel_pub,
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &goal_pub,
    const CostmapWrapper::Ptr &costmap_ptr,
    const rclcpp::Node::SharedPtr& node)
  : AbstractControllerExecution(controller_name, controller_ptr, robot_info, vel_pub, goal_pub, node)
  , costmap_ptr_(costmap_ptr)
{
  node->declare_parameter<bool>("controller_lock_costmap", true);
  node->get_parameter("controller_lock_costmap", lock_costmap_);
}

CostmapControllerExecution::~CostmapControllerExecution()
{
}

uint32_t CostmapControllerExecution::computeVelocityCmd(
    const geometry_msgs::msg::PoseStamped &robot_pose,
    const geometry_msgs::msg::TwistStamped &robot_velocity,
    geometry_msgs::msg::TwistStamped &vel_cmd,
    std::string &message)
{
  if (lock_costmap_)
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
    return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
  }
  return controller_->computeVelocityCommands(robot_pose, robot_velocity, vel_cmd, message);
}

bool CostmapControllerExecution::safetyCheck()
{
  if (!costmap_ptr_->isCurrent())
  {
    RCLCPP_WARN(node_handle_->get_logger(),
                "Sensor data is out of date, we're not going to allow commanding of the base for safety");
    return false;
  }
  return true;
}

} /* namespace mbf_costmap_nav */
