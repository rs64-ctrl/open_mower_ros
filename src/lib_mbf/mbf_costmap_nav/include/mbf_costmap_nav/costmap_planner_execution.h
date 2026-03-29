/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_PLANNER_EXECUTION_H_
#define MBF_COSTMAP_NAV__COSTMAP_PLANNER_EXECUTION_H_

#include <mbf_abstract_nav/abstract_planner_execution.h>
#include <mbf_costmap_core/costmap_planner.h>

#include "mbf_costmap_nav/costmap_wrapper.h"

namespace mbf_costmap_nav
{

class CostmapPlannerExecution : public mbf_abstract_nav::AbstractPlannerExecution
{
public:
  CostmapPlannerExecution(const std::string& planner_name,
                          const mbf_costmap_core::CostmapPlanner::Ptr& planner_ptr,
                          const mbf_utility::RobotInformation::ConstPtr& robot_info,
                          const CostmapWrapper::Ptr& costmap_ptr,
                          const rclcpp::Node::SharedPtr& node);

  virtual ~CostmapPlannerExecution();

private:
  void preRun()
  {
    costmap_ptr_->checkActivate();
  };

  void postRun()
  {
    costmap_ptr_->checkDeactivate();
  };

  virtual uint32_t makePlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      double tolerance,
      std::vector<geometry_msgs::msg::PoseStamped> &plan,
      double &cost,
      std::string &message);

  const CostmapWrapper::Ptr costmap_ptr_;

  bool lock_costmap_;

  std::string planner_name_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_PLANNER_EXECUTION_H_ */
