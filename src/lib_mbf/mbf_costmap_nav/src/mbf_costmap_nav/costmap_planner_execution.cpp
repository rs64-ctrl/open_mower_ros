/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#include "mbf_costmap_nav/costmap_planner_execution.h"
#include <mbf_msgs/action/get_path.hpp>
#include <mbf_utility/navigation_utility.h>

namespace mbf_costmap_nav
{

CostmapPlannerExecution::CostmapPlannerExecution(
    const std::string& planner_name,
    const mbf_costmap_core::CostmapPlanner::Ptr& planner_ptr,
    const mbf_utility::RobotInformation::ConstPtr& robot_info,
    const CostmapWrapper::Ptr& costmap_ptr,
    const rclcpp::Node::SharedPtr& node)
  : AbstractPlannerExecution(planner_name, planner_ptr, robot_info, node)
  , costmap_ptr_(costmap_ptr)
{
  node->declare_parameter<bool>("planner_lock_costmap", true);
  node->get_parameter("planner_lock_costmap", lock_costmap_);
}

CostmapPlannerExecution::~CostmapPlannerExecution()
{
}

uint32_t CostmapPlannerExecution::makePlan(const geometry_msgs::msg::PoseStamped &start,
                                           const geometry_msgs::msg::PoseStamped &goal,
                                           double tolerance,
                                           std::vector<geometry_msgs::msg::PoseStamped> &plan,
                                           double &cost,
                                           std::string &message)
{
  const std::string frame = costmap_ptr_->getGlobalFrameID();
  geometry_msgs::msg::PoseStamped g_start, g_goal;

  if (!mbf_utility::transformPose(node_, robot_info_->getTransformListener(), frame,
                                  rclcpp::Duration::from_seconds(0.5), start, g_start))
    return mbf_msgs::action::GetPath::Result::TF_ERROR;

  if (!mbf_utility::transformPose(node_, robot_info_->getTransformListener(), frame,
                                  rclcpp::Duration::from_seconds(0.5), goal, g_goal))
    return mbf_msgs::action::GetPath::Result::TF_ERROR;

  if (lock_costmap_)
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_ptr_->getCostmap()->getMutex()));
    return planner_->makePlan(g_start, g_goal, tolerance, plan, cost, message);
  }
  return planner_->makePlan(g_start, g_goal, tolerance, plan, cost, message);
}

} /* namespace mbf_costmap_nav */
