/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_RECOVERY_EXECUTION_H_
#define MBF_COSTMAP_NAV__COSTMAP_RECOVERY_EXECUTION_H_

#include <mbf_abstract_nav/abstract_recovery_execution.h>
#include <mbf_costmap_core/costmap_recovery.h>

#include "mbf_costmap_nav/costmap_wrapper.h"

namespace mbf_costmap_nav
{

class CostmapRecoveryExecution : public mbf_abstract_nav::AbstractRecoveryExecution
{
public:
  typedef std::shared_ptr<CostmapRecoveryExecution> Ptr;

  CostmapRecoveryExecution(
      const std::string &recovery_name,
      const mbf_costmap_core::CostmapRecovery::Ptr &recovery_ptr,
      const mbf_utility::RobotInformation::ConstPtr& robot_info,
      const CostmapWrapper::Ptr &global_costmap,
      const CostmapWrapper::Ptr &local_costmap,
      const rclcpp::Node::SharedPtr& node);

  virtual ~CostmapRecoveryExecution();

private:
  void preRun()
  {
    local_costmap_->checkActivate();
    global_costmap_->checkActivate();
  };

  void postRun()
  {
    local_costmap_->checkDeactivate();
    global_costmap_->checkDeactivate();
  };

  const CostmapWrapper::Ptr global_costmap_;
  const CostmapWrapper::Ptr local_costmap_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_RECOVERY_EXECUTION_H_ */
