/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

#include "mbf_costmap_nav/footprint_helper.h"
#include "mbf_costmap_nav/costmap_navigation_server.h"

namespace mbf_costmap_nav
{

template <typename Key, typename Value>
const Value& findWithDefault(const std::unordered_map<Key, Value>& map, const Key& key, const Value& default_value)
{
  auto iter = map.find(key);
  if (iter == map.end())
    return default_value;
  return iter->second;
}

CostmapNavigationServer::CostmapNavigationServer(const TFPtr &tf_listener_ptr, const rclcpp::Node::SharedPtr& node) :
  AbstractNavigationServer(tf_listener_ptr, node),
  recovery_plugin_loader_("mbf_costmap_core", "mbf_costmap_core::CostmapRecovery"),
  controller_plugin_loader_("mbf_costmap_core", "mbf_costmap_core::CostmapController"),
  planner_plugin_loader_("mbf_costmap_core", "mbf_costmap_core::CostmapPlanner"),
  global_costmap_ptr_(std::make_shared<CostmapWrapper>("global_costmap", tf_listener_ptr_, node)),
  local_costmap_ptr_(std::make_shared<CostmapWrapper>("local_costmap", tf_listener_ptr_, node))
{
  // Advertise services
  check_point_cost_srv_ = node_->create_service<mbf_msgs::srv::CheckPoint>(
      "~/check_point_cost",
      std::bind(&CostmapNavigationServer::callServiceCheckPointCost, this,
                std::placeholders::_1, std::placeholders::_2));
  check_pose_cost_srv_ = node_->create_service<mbf_msgs::srv::CheckPose>(
      "~/check_pose_cost",
      std::bind(&CostmapNavigationServer::callServiceCheckPoseCost, this,
                std::placeholders::_1, std::placeholders::_2));
  check_path_cost_srv_ = node_->create_service<mbf_msgs::srv::CheckPath>(
      "~/check_path_cost",
      std::bind(&CostmapNavigationServer::callServiceCheckPathCost, this,
                std::placeholders::_1, std::placeholders::_2));
  clear_costmaps_srv_ = node_->create_service<std_srvs::srv::Empty>(
      "~/clear_costmaps",
      std::bind(&CostmapNavigationServer::callServiceClearCostmaps, this,
                std::placeholders::_1, std::placeholders::_2));

  // Declare costmap-specific parameters
  node_->declare_parameter<bool>("shutdown_costmaps", false);
  node_->declare_parameter<double>("shutdown_costmaps_delay", 2.0);

  // Activate costmaps (lifecycle: configure + activate)
  // Must happen after make_shared so shared_from_this() works inside Costmap2DROS
  local_costmap_ptr_->activate();
  global_costmap_ptr_->activate();

  // Initialize all plugins
  initializeServerComponents();
}

CostmapNavigationServer::~CostmapNavigationServer()
{
  controller_plugin_manager_.clearPlugins();
  planner_plugin_manager_.clearPlugins();
  recovery_plugin_manager_.clearPlugins();

  action_server_recovery_ptr_.reset();
  action_server_exe_path_ptr_.reset();
  action_server_get_path_ptr_.reset();
  action_server_move_base_ptr_.reset();
}

mbf_abstract_nav::AbstractPlannerExecution::Ptr CostmapNavigationServer::newPlannerExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractPlanner::Ptr &plugin_ptr)
{
  const CostmapWrapper::Ptr& costmap_ptr =
      findWithDefault(planner_name_to_costmap_ptr_, plugin_name, global_costmap_ptr_);
  return std::make_shared<mbf_costmap_nav::CostmapPlannerExecution>(
      plugin_name, std::static_pointer_cast<mbf_costmap_core::CostmapPlanner>(plugin_ptr),
      robot_info_, costmap_ptr, node_);
}

mbf_abstract_nav::AbstractControllerExecution::Ptr CostmapNavigationServer::newControllerExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractController::Ptr &plugin_ptr)
{
  const CostmapWrapper::Ptr& costmap_ptr =
      findWithDefault(controller_name_to_costmap_ptr_, plugin_name, local_costmap_ptr_);
  return std::make_shared<mbf_costmap_nav::CostmapControllerExecution>(
      plugin_name, std::static_pointer_cast<mbf_costmap_core::CostmapController>(plugin_ptr),
      robot_info_, vel_pub_, goal_pub_, costmap_ptr, node_);
}

mbf_abstract_nav::AbstractRecoveryExecution::Ptr CostmapNavigationServer::newRecoveryExecution(
    const std::string &plugin_name,
    const mbf_abstract_core::AbstractRecovery::Ptr &plugin_ptr)
{
  return std::make_shared<mbf_costmap_nav::CostmapRecoveryExecution>(
      plugin_name,
      std::static_pointer_cast<mbf_costmap_core::CostmapRecovery>(plugin_ptr),
      robot_info_,
      global_costmap_ptr_,
      local_costmap_ptr_,
      node_);
}

mbf_abstract_core::AbstractPlanner::Ptr CostmapNavigationServer::loadPlannerPlugin(const std::string &planner_type)
{
  mbf_abstract_core::AbstractPlanner::Ptr planner_ptr;
  try
  {
    planner_ptr = planner_plugin_loader_.createSharedInstance(planner_type);
    RCLCPP_DEBUG(node_->get_logger(), "mbf_costmap_core-based planner plugin '%s' loaded.", planner_type.c_str());
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "Failed to load the '%s' planner, are you sure it's properly registered"
                 " and that the containing library is built? Exception: %s",
                 planner_type.c_str(), ex.what());
  }
  return planner_ptr;
}

bool CostmapNavigationServer::initializePlannerPlugin(
    const std::string &name,
    const mbf_abstract_core::AbstractPlanner::Ptr &planner_ptr)
{
  mbf_costmap_core::CostmapPlanner::Ptr costmap_planner_ptr =
      std::static_pointer_cast<mbf_costmap_core::CostmapPlanner>(planner_ptr);
  RCLCPP_DEBUG(node_->get_logger(), "Initialize planner \"%s\".", name.c_str());

  const CostmapWrapper::Ptr& costmap_ptr = findWithDefault(planner_name_to_costmap_ptr_, name, global_costmap_ptr_);

  if (!costmap_ptr)
  {
    RCLCPP_FATAL(node_->get_logger(), "The costmap pointer has not been initialized!");
    return false;
  }

  costmap_planner_ptr->initialize(name, node_, tf_listener_ptr_.get(), costmap_ptr.get());
  return true;
}

mbf_abstract_core::AbstractController::Ptr CostmapNavigationServer::loadControllerPlugin(const std::string &controller_type)
{
  mbf_abstract_core::AbstractController::Ptr controller_ptr;
  try
  {
    controller_ptr = controller_plugin_loader_.createSharedInstance(controller_type);
    RCLCPP_DEBUG(node_->get_logger(), "mbf_costmap_core-based controller plugin '%s' loaded.", controller_type.c_str());
  }
  catch (const pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "Failed to load the '%s' controller, are you sure it's properly registered"
                 " and that the containing library is built? Exception: %s",
                 controller_type.c_str(), ex.what());
  }
  return controller_ptr;
}

bool CostmapNavigationServer::initializeControllerPlugin(
    const std::string &name,
    const mbf_abstract_core::AbstractController::Ptr &controller_ptr)
{
  RCLCPP_DEBUG(node_->get_logger(), "Initialize controller \"%s\".", name.c_str());

  if (!tf_listener_ptr_)
  {
    RCLCPP_FATAL(node_->get_logger(), "The tf listener pointer has not been initialized!");
    return false;
  }

  const CostmapWrapper::Ptr& costmap_ptr = findWithDefault(controller_name_to_costmap_ptr_, name, local_costmap_ptr_);

  if (!costmap_ptr)
  {
    RCLCPP_FATAL(node_->get_logger(), "The costmap pointer has not been initialized!");
    return false;
  }

  mbf_costmap_core::CostmapController::Ptr costmap_controller_ptr =
      std::static_pointer_cast<mbf_costmap_core::CostmapController>(controller_ptr);
  costmap_controller_ptr->initialize(name, node_, tf_listener_ptr_.get(), costmap_ptr.get());
  RCLCPP_DEBUG(node_->get_logger(), "Controller plugin \"%s\" initialized.", name.c_str());
  return true;
}

mbf_abstract_core::AbstractRecovery::Ptr CostmapNavigationServer::loadRecoveryPlugin(
    const std::string &recovery_type)
{
  mbf_abstract_core::AbstractRecovery::Ptr recovery_ptr;
  try
  {
    recovery_ptr = recovery_plugin_loader_.createSharedInstance(recovery_type);
    RCLCPP_DEBUG(node_->get_logger(), "mbf_costmap_core-based recovery behavior plugin '%s' loaded.",
                 recovery_type.c_str());
  }
  catch (pluginlib::PluginlibException &ex)
  {
    RCLCPP_FATAL(node_->get_logger(),
                 "Failed to load the '%s' recovery behavior, are you sure it's properly registered"
                 " and that the containing library is built? Exception: %s",
                 recovery_type.c_str(), ex.what());
  }
  return recovery_ptr;
}

bool CostmapNavigationServer::initializeRecoveryPlugin(
    const std::string &name,
    const mbf_abstract_core::AbstractRecovery::Ptr &behavior_ptr)
{
  RCLCPP_DEBUG(node_->get_logger(), "Initialize recovery behavior \"%s\".", name.c_str());

  if (!tf_listener_ptr_)
  {
    RCLCPP_FATAL(node_->get_logger(), "The tf listener pointer has not been initialized!");
    return false;
  }

  if (!local_costmap_ptr_)
  {
    RCLCPP_FATAL(node_->get_logger(), "The local costmap pointer has not been initialized!");
    return false;
  }

  if (!global_costmap_ptr_)
  {
    RCLCPP_FATAL(node_->get_logger(), "The global costmap pointer has not been initialized!");
    return false;
  }

  mbf_costmap_core::CostmapRecovery::Ptr behavior =
      std::static_pointer_cast<mbf_costmap_core::CostmapRecovery>(behavior_ptr);
  behavior->initialize(name, node_, tf_listener_ptr_.get(), global_costmap_ptr_.get(), local_costmap_ptr_.get());
  RCLCPP_DEBUG(node_->get_logger(), "Recovery behavior plugin \"%s\" initialized.", name.c_str());
  return true;
}

void CostmapNavigationServer::stop()
{
  AbstractNavigationServer::stop();
  RCLCPP_INFO(node_->get_logger(), "Stopping local and global costmap for shutdown");
  local_costmap_ptr_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  global_costmap_ptr_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

void CostmapNavigationServer::callServiceCheckPointCost(
    const std::shared_ptr<mbf_msgs::srv::CheckPoint::Request> request,
    std::shared_ptr<mbf_msgs::srv::CheckPoint::Response> response)
{
  CostmapWrapper::Ptr costmap;
  std::string costmap_name;
  switch (request->costmap)
  {
    case mbf_msgs::srv::CheckPoint::Request::LOCAL_COSTMAP:
      costmap = local_costmap_ptr_;
      costmap_name = "local costmap";
      break;
    case mbf_msgs::srv::CheckPoint::Request::GLOBAL_COSTMAP:
      costmap = global_costmap_ptr_;
      costmap_name = "global costmap";
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "No valid costmap provided; options are LOCAL_COSTMAP or GLOBAL_COSTMAP");
      return;
  }

  // Get costmap value at the requested point
  unsigned int mx, my;
  if (!costmap->getCostmap()->worldToMap(request->point.point.x, request->point.point.y, mx, my))
  {
    response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPoint::Response::OUTSIDE);
    return;
  }

  response->cost = costmap->getCostmap()->getCost(mx, my);
  switch (response->cost)
  {
    case nav2_costmap_2d::FREE_SPACE:
      response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPoint::Response::FREE);
      break;
    case nav2_costmap_2d::LETHAL_OBSTACLE:
      response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPoint::Response::LETHAL);
      break;
    case nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
      response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPoint::Response::INSCRIBED);
      break;
    default:
      response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPoint::Response::FREE);
      break;
  }
}

void CostmapNavigationServer::callServiceCheckPoseCost(
    const std::shared_ptr<mbf_msgs::srv::CheckPose::Request> request,
    std::shared_ptr<mbf_msgs::srv::CheckPose::Response> response)
{
  CostmapWrapper::Ptr costmap;
  switch (request->costmap)
  {
    case mbf_msgs::srv::CheckPose::Request::LOCAL_COSTMAP:
      costmap = local_costmap_ptr_;
      break;
    case mbf_msgs::srv::CheckPose::Request::GLOBAL_COSTMAP:
      costmap = global_costmap_ptr_;
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "No valid costmap provided; options are LOCAL_COSTMAP or GLOBAL_COSTMAP");
      return;
  }

  // Get robot footprint cells at the requested pose and check costs
  const geometry_msgs::msg::Point &position = request->pose.pose.position;
  double yaw = tf2::getYaw(request->pose.pose.orientation);

  std::vector<Cell> footprint_cells =
      FootprintHelper::getFootprintCells(position.x, position.y, yaw,
                                         costmap->getRobotFootprint(),
                                         *costmap->getCostmap(), true);

  response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPose::Response::FREE);
  if (footprint_cells.empty())
  {
    response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPose::Response::OUTSIDE);
    return;
  }

  unsigned char max_cost = 0;
  for (const auto &cell : footprint_cells)
  {
    unsigned char cost = costmap->getCostmap()->getCost(cell.x, cell.y);
    if (cost > max_cost)
      max_cost = cost;
  }

  response->cost = max_cost;
  if (max_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
    response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPose::Response::LETHAL);
  else if (max_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPose::Response::INSCRIBED);
}

void CostmapNavigationServer::callServiceCheckPathCost(
    const std::shared_ptr<mbf_msgs::srv::CheckPath::Request> request,
    std::shared_ptr<mbf_msgs::srv::CheckPath::Response> response)
{
  CostmapWrapper::Ptr costmap;
  switch (request->costmap)
  {
    case mbf_msgs::srv::CheckPath::Request::LOCAL_COSTMAP:
      costmap = local_costmap_ptr_;
      break;
    case mbf_msgs::srv::CheckPath::Request::GLOBAL_COSTMAP:
      costmap = global_costmap_ptr_;
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "No valid costmap provided; options are LOCAL_COSTMAP or GLOBAL_COSTMAP");
      return;
  }

  unsigned char max_cost = 0;
  for (const auto &pose : request->path.poses)
  {
    const geometry_msgs::msg::Point &position = pose.pose.position;
    double yaw = tf2::getYaw(pose.pose.orientation);

    std::vector<Cell> footprint_cells =
        FootprintHelper::getFootprintCells(position.x, position.y, yaw,
                                           costmap->getRobotFootprint(),
                                           *costmap->getCostmap(), true);

    if (footprint_cells.empty())
    {
      response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPath::Response::OUTSIDE);
      return;
    }

    for (const auto &cell : footprint_cells)
    {
      unsigned char cost = costmap->getCostmap()->getCost(cell.x, cell.y);
      if (cost > max_cost)
        max_cost = cost;
    }
  }

  response->cost = max_cost;
  if (max_cost == nav2_costmap_2d::LETHAL_OBSTACLE)
    response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPath::Response::LETHAL);
  else if (max_cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPath::Response::INSCRIBED);
  else
    response->state = static_cast<uint8_t>(mbf_msgs::srv::CheckPath::Response::FREE);
}

void CostmapNavigationServer::callServiceClearCostmaps(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
  local_costmap_ptr_->clear();
  global_costmap_ptr_->clear();
}

} /* namespace mbf_costmap_nav */
