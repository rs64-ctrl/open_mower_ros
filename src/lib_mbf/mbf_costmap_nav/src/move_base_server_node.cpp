/*
 *  Copyright 2018, Magazino GmbH, Sebastian Puetz, Jorge Santos Simon
 *  BSD-3-Clause license, see LICENSE file.
 */

#include "mbf_costmap_nav/costmap_navigation_server.h"

#include <mbf_utility/types.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("mbf_costmap_nav");
  node->declare_parameter<double>("tf_cache_time", 10.0);

  double cache_time;
  node->get_parameter("tf_cache_time", cache_time);
  TFPtr tf_buffer_ptr(new TF(node->get_clock(), tf2::durationFromSec(cache_time)));
  tf2_ros::TransformListener tf_listener(*tf_buffer_ptr);

  mbf_costmap_nav::CostmapNavigationServer costmap_nav_server(tf_buffer_ptr, node);

  rclcpp::spin(node);
  return EXIT_SUCCESS;
}
