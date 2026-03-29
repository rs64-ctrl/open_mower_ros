/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_NAVIGATION_SERVER_H_
#define MBF_COSTMAP_NAV__COSTMAP_NAVIGATION_SERVER_H_

#include <mbf_abstract_nav/abstract_navigation_server.h>

#include <mbf_msgs/srv/check_path.hpp>
#include <mbf_msgs/srv/check_pose.hpp>
#include <mbf_msgs/srv/check_point.hpp>
#include <std_srvs/srv/empty.hpp>

#include "mbf_costmap_nav/costmap_planner_execution.h"
#include "mbf_costmap_nav/costmap_controller_execution.h"
#include "mbf_costmap_nav/costmap_recovery_execution.h"
#include "mbf_costmap_nav/costmap_wrapper.h"

#include <pluginlib/class_loader.hpp>
#include <unordered_map>
#include <string>

namespace mbf_costmap_nav
{

typedef std::unordered_map<std::string, CostmapWrapper::Ptr> StringToMap;

class CostmapNavigationServer : public mbf_abstract_nav::AbstractNavigationServer
{
public:
  typedef std::shared_ptr<CostmapNavigationServer> Ptr;

  CostmapNavigationServer(const TFPtr &tf_listener_ptr, const rclcpp::Node::SharedPtr& node);

  virtual ~CostmapNavigationServer();

  virtual void stop();

private:
  virtual mbf_abstract_nav::AbstractPlannerExecution::Ptr newPlannerExecution(
      const std::string &plugin_name,
      const mbf_abstract_core::AbstractPlanner::Ptr &plugin_ptr);

  virtual mbf_abstract_nav::AbstractControllerExecution::Ptr newControllerExecution(
      const std::string &plugin_name,
      const mbf_abstract_core::AbstractController::Ptr &plugin_ptr);

  virtual mbf_abstract_nav::AbstractRecoveryExecution::Ptr newRecoveryExecution(
      const std::string &plugin_name,
      const mbf_abstract_core::AbstractRecovery::Ptr &plugin_ptr);

  virtual mbf_abstract_core::AbstractPlanner::Ptr loadPlannerPlugin(const std::string &planner_type);

  virtual bool initializePlannerPlugin(
      const std::string &name,
      const mbf_abstract_core::AbstractPlanner::Ptr &planner_ptr);

  virtual mbf_abstract_core::AbstractController::Ptr loadControllerPlugin(const std::string &controller_type);

  virtual bool initializeControllerPlugin(
      const std::string &name,
      const mbf_abstract_core::AbstractController::Ptr &controller_ptr);

  virtual mbf_abstract_core::AbstractRecovery::Ptr loadRecoveryPlugin(const std::string &recovery_type);

  virtual bool initializeRecoveryPlugin(
      const std::string &name,
      const mbf_abstract_core::AbstractRecovery::Ptr &behavior_ptr);

  void callServiceCheckPointCost(
      const std::shared_ptr<mbf_msgs::srv::CheckPoint::Request> request,
      std::shared_ptr<mbf_msgs::srv::CheckPoint::Response> response);

  void callServiceCheckPoseCost(
      const std::shared_ptr<mbf_msgs::srv::CheckPose::Request> request,
      std::shared_ptr<mbf_msgs::srv::CheckPose::Response> response);

  void callServiceCheckPathCost(
      const std::shared_ptr<mbf_msgs::srv::CheckPath::Request> request,
      std::shared_ptr<mbf_msgs::srv::CheckPath::Response> response);

  void callServiceClearCostmaps(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);

  pluginlib::ClassLoader<mbf_costmap_core::CostmapRecovery> recovery_plugin_loader_;
  pluginlib::ClassLoader<mbf_costmap_core::CostmapController> controller_plugin_loader_;
  pluginlib::ClassLoader<mbf_costmap_core::CostmapPlanner> planner_plugin_loader_;

  const CostmapWrapper::Ptr local_costmap_ptr_;
  const CostmapWrapper::Ptr global_costmap_ptr_;

  StringToMap planner_name_to_costmap_ptr_;
  StringToMap controller_name_to_costmap_ptr_;

  rclcpp::Service<mbf_msgs::srv::CheckPoint>::SharedPtr check_point_cost_srv_;
  rclcpp::Service<mbf_msgs::srv::CheckPose>::SharedPtr check_pose_cost_srv_;
  rclcpp::Service<mbf_msgs::srv::CheckPath>::SharedPtr check_path_cost_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_costmaps_srv_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_NAVIGATION_SERVER_H_ */
