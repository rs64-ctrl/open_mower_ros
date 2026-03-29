/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_CONTROLLER_EXECUTION_H_
#define MBF_COSTMAP_NAV__COSTMAP_CONTROLLER_EXECUTION_H_

#include <mbf_abstract_nav/abstract_controller_execution.h>
#include <mbf_costmap_core/costmap_controller.h>

#include "mbf_costmap_nav/costmap_wrapper.h"

namespace mbf_costmap_nav
{

class CostmapControllerExecution : public mbf_abstract_nav::AbstractControllerExecution
{
public:
  CostmapControllerExecution(
      const std::string &controller_name,
      const mbf_costmap_core::CostmapController::Ptr &controller_ptr,
      const mbf_utility::RobotInformation::ConstPtr& robot_info,
      const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr &vel_pub,
      const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &goal_pub,
      const CostmapWrapper::Ptr &costmap_ptr,
      const rclcpp::Node::SharedPtr& node);

  virtual ~CostmapControllerExecution();

private:
  void preRun()
  {
    costmap_ptr_->checkActivate();
  };

  void postRun()
  {
    costmap_ptr_->checkDeactivate();
  };

  bool safetyCheck();

  virtual uint32_t computeVelocityCmd(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::TwistStamped &robot_velocity,
      geometry_msgs::msg::TwistStamped &vel_cmd,
      std::string &message);

  const CostmapWrapper::Ptr costmap_ptr_;

  bool lock_costmap_;

  std::string controller_name_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_CONTROLLER_EXECUTION_H_ */
