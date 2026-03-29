/*
 *  Software License Agreement (BSD License)
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  BSD-3-Clause license, see LICENSE file.
 *  Author: TKruse
 */

#ifndef FOOTPRINT_HELPER_H_
#define FOOTPRINT_HELPER_H_

#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace mbf_costmap_nav
{

struct Cell
{
  unsigned int x, y;
};

class FootprintHelper
{
public:
  static std::vector<Cell> getFootprintCells(double x, double y, double theta,
                                             const std::vector<geometry_msgs::msg::Point>& footprint_spec,
                                             const nav2_costmap_2d::Costmap2D&, bool fill);

  static void getLineCells(int x0, int x1, int y0, int y1, std::vector<Cell>& pts);

  static void getFillCells(std::vector<Cell>& footprint);
};

} /* namespace mbf_costmap_nav */
#endif /* FOOTPRINT_HELPER_H_ */
