/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#include "mbf_costmap_nav/costmap_recovery_execution.h"

namespace mbf_costmap_nav
{

CostmapRecoveryExecution::CostmapRecoveryExecution(
    const std::string &recovery_name,
    const mbf_costmap_core::CostmapRecovery::Ptr &recovery_ptr,
    const mbf_utility::RobotInformation::ConstPtr& robot_info,
    const CostmapWrapper::Ptr &global_costmap,
    const CostmapWrapper::Ptr &local_costmap,
    const rclcpp::Node::SharedPtr& node)
  : AbstractRecoveryExecution(recovery_name, recovery_ptr, robot_info, node)
  , global_costmap_(global_costmap)
  , local_costmap_(local_costmap)
{
}

CostmapRecoveryExecution::~CostmapRecoveryExecution()
{
}

} /* namespace mbf_costmap_nav */
