/*
 *  Copyright 2019, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#include "mbf_costmap_nav/costmap_wrapper.h"
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

namespace mbf_costmap_nav
{

CostmapWrapper::CostmapWrapper(const std::string &name,
                               const TFPtr &tf_listener_ptr,
                               const rclcpp::Node::SharedPtr &node)
  : nav2_costmap_2d::Costmap2DROS(name, std::string(""), std::string(""), false)
  , node_(node)
  , shutdown_costmap_(false)
  , clear_on_shutdown_(false)
  , costmap_users_(0)
  , shutdown_costmap_delay_(0.0)
{
  // Read parameters from the parent node (not the costmap lifecycle node)
  node_->declare_parameter<bool>(name + ".shutdown_costmaps", false);
  node_->declare_parameter<bool>(name + ".clear_on_shutdown", false);
  node_->get_parameter(name + ".shutdown_costmaps", shutdown_costmap_);
  node_->get_parameter(name + ".clear_on_shutdown", clear_on_shutdown_);
}

void CostmapWrapper::activate()
{
  // Nav2 Costmap2DROS is a lifecycle node — drive through configure and activate
  // Use trigger_transition which properly manages the state machine
  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  if (shutdown_costmap_)
  {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }
  else
  {
    ++costmap_users_;
  }
}

CostmapWrapper::~CostmapWrapper()
{
  if (shutdown_costmap_timer_)
    shutdown_costmap_timer_->cancel();
}

void CostmapWrapper::reconfigure(double shutdown_costmap, double shutdown_costmap_delay)
{
  shutdown_costmap_delay_ = shutdown_costmap_delay;
  if (shutdown_costmap_delay_ <= 0.0)
    RCLCPP_WARN(node_->get_logger(),
                "Zero shutdown costmaps delay is not recommended, as it forces us to enable costmaps on each action");

  if (shutdown_costmap_ && !shutdown_costmap)
  {
    checkActivate();
    shutdown_costmap_ = shutdown_costmap;
  }
  if (!shutdown_costmap_ && shutdown_costmap)
  {
    shutdown_costmap_ = shutdown_costmap;
    checkDeactivate();
  }
}

void CostmapWrapper::clear()
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getCostmap()->getMutex());
  resetLayers();
}

void CostmapWrapper::checkActivate()
{
  std::lock_guard<std::mutex> sl(check_costmap_mutex_);

  if (shutdown_costmap_timer_)
  {
    shutdown_costmap_timer_->cancel();
    shutdown_costmap_timer_.reset();
  }

  if (shutdown_costmap_ && !costmap_users_)
  {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    RCLCPP_DEBUG(node_->get_logger(), "%s activated", getName().c_str());
  }
  ++costmap_users_;
}

void CostmapWrapper::checkDeactivate()
{
  std::lock_guard<std::mutex> sl(check_costmap_mutex_);

  --costmap_users_;
  if (costmap_users_ < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Negative number (%d) of active costmap users!", costmap_users_);
    costmap_users_ = 0;
  }

  if (shutdown_costmap_ && !costmap_users_)
  {
    shutdown_costmap_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(shutdown_costmap_delay_),
        std::bind(&CostmapWrapper::deactivate, this));
  }
}

void CostmapWrapper::deactivate()
{
  std::lock_guard<std::mutex> sl(check_costmap_mutex_);

  // Cancel the timer so it doesn't fire again
  if (shutdown_costmap_timer_)
  {
    shutdown_costmap_timer_->cancel();
    shutdown_costmap_timer_.reset();
  }

  if (costmap_users_)
  {
    RCLCPP_WARN(node_->get_logger(), "Deactivating costmap with %d active users!", costmap_users_);
    return;
  }

  if (clear_on_shutdown_)
    clear();
  trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  RCLCPP_DEBUG(node_->get_logger(), "%s deactivated", getName().c_str());
}

} /* namespace mbf_costmap_nav */
