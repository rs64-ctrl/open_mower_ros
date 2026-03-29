/*
 *  Copyright 2019, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#ifndef MBF_COSTMAP_NAV__COSTMAP_WRAPPER_H_
#define MBF_COSTMAP_NAV__COSTMAP_WRAPPER_H_

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <mbf_utility/types.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

namespace mbf_costmap_nav
{

class CostmapWrapper : public nav2_costmap_2d::Costmap2DROS
{
public:
  typedef std::shared_ptr<CostmapWrapper> Ptr;

  CostmapWrapper(const std::string &name,
                 const TFPtr &tf_listener_ptr,
                 const rclcpp::Node::SharedPtr &node);

  /// Must be called after make_shared to drive the lifecycle (shared_from_this requirement)
  void activate();

  virtual ~CostmapWrapper();

  void reconfigure(double shutdown_costmap, double shutdown_costmap_delay);

  void clear();

  void checkActivate();

  void checkDeactivate();

private:
  void deactivate();

  rclcpp::Node::SharedPtr node_;

  std::mutex check_costmap_mutex_;
  bool shutdown_costmap_;
  bool clear_on_shutdown_;
  int16_t costmap_users_;
  rclcpp::TimerBase::SharedPtr shutdown_costmap_timer_;
  double shutdown_costmap_delay_;
};

} /* namespace mbf_costmap_nav */

#endif /* MBF_COSTMAP_NAV__COSTMAP_WRAPPER_H_ */
