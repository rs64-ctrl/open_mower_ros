/*
 *  Software License Agreement (BSD License)
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  BSD-3-Clause license, see LICENSE file.
 *  Author: TKruse
 */

#include "mbf_costmap_nav/footprint_helper.h"
#include <cmath>

namespace mbf_costmap_nav
{

void FootprintHelper::getLineCells(int x0, int x1, int y0, int y1, std::vector<Cell>& pts) {
  int deltax = abs(x1 - x0);
  int deltay = abs(y1 - y0);
  int x = x0;
  int y = y0;

  int xinc1, xinc2, yinc1, yinc2;
  int den, num, numadd, numpixels;

  Cell pt;

  if (x1 >= x0) { xinc1 = 1; xinc2 = 1; }
  else           { xinc1 = -1; xinc2 = -1; }

  if (y1 >= y0) { yinc1 = 1; yinc2 = 1; }
  else           { yinc1 = -1; yinc2 = -1; }

  if (deltax >= deltay) {
    xinc1 = 0; yinc2 = 0;
    den = deltax; num = deltax / 2; numadd = deltay; numpixels = deltax;
  } else {
    xinc2 = 0; yinc1 = 0;
    den = deltay; num = deltay / 2; numadd = deltax; numpixels = deltay;
  }

  for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
    pt.x = x; pt.y = y;
    pts.push_back(pt);
    num += numadd;
    if (num >= den) { num -= den; x += xinc1; y += yinc1; }
    x += xinc2; y += yinc2;
  }
}

void FootprintHelper::getFillCells(std::vector<Cell>& footprint) {
  Cell swap, pt;
  unsigned int i = 0;
  while (i < footprint.size() - 1) {
    if (footprint[i].x > footprint[i + 1].x) {
      swap = footprint[i];
      footprint[i] = footprint[i + 1];
      footprint[i + 1] = swap;
      if(i > 0) --i;
    } else {
      ++i;
    }
  }

  i = 0;
  Cell min_pt, max_pt;
  unsigned int min_x = footprint[0].x;
  unsigned int max_x = footprint[footprint.size() - 1].x;
  for (unsigned int x = min_x; x <= max_x; ++x) {
    if (i >= footprint.size() - 1) break;
    if (footprint[i].y < footprint[i + 1].y) {
      min_pt = footprint[i]; max_pt = footprint[i + 1];
    } else {
      min_pt = footprint[i + 1]; max_pt = footprint[i];
    }
    i += 2;
    while (i < footprint.size() && footprint[i].x == x) {
      if(footprint[i].y < min_pt.y) min_pt = footprint[i];
      else if(footprint[i].y > max_pt.y) max_pt = footprint[i];
      ++i;
    }
    for (unsigned int y = min_pt.y; y < max_pt.y; ++y) {
      pt.x = x; pt.y = y;
      footprint.push_back(pt);
    }
  }
}

static const std::vector<Cell>& clearAndReturn(std::vector<Cell>& _cells)
{
  _cells.clear();
  return _cells;
}

std::vector<Cell> FootprintHelper::getFootprintCells(double x, double y, double theta,
                                                     const std::vector<geometry_msgs::msg::Point>& footprint_spec,
                                                     const nav2_costmap_2d::Costmap2D& costmap, bool fill)
{
  std::vector<Cell> footprint_cells;

  if (footprint_spec.size() <= 1) {
    unsigned int mx, my;
    if (costmap.worldToMap(x, y, mx, my)) {
      Cell center;
      center.x = mx; center.y = my;
      footprint_cells.push_back(center);
    }
    return footprint_cells;
  }

  double cos_th = cos(theta);
  double sin_th = sin(theta);
  double new_x, new_y;
  unsigned int x0, y0, x1, y1;
  unsigned int last_index = footprint_spec.size() - 1;

  for (unsigned int i = 0; i < last_index; ++i) {
    new_x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    if(!costmap.worldToMap(new_x, new_y, x0, y0)) return clearAndReturn(footprint_cells);

    new_x = x + (footprint_spec[i + 1].x * cos_th - footprint_spec[i + 1].y * sin_th);
    new_y = y + (footprint_spec[i + 1].x * sin_th + footprint_spec[i + 1].y * cos_th);
    if (!costmap.worldToMap(new_x, new_y, x1, y1)) return clearAndReturn(footprint_cells);

    getLineCells(x0, x1, y0, y1, footprint_cells);
  }

  new_x = x + (footprint_spec[last_index].x * cos_th - footprint_spec[last_index].y * sin_th);
  new_y = y + (footprint_spec[last_index].x * sin_th + footprint_spec[last_index].y * cos_th);
  if (!costmap.worldToMap(new_x, new_y, x0, y0)) return clearAndReturn(footprint_cells);
  new_x = x + (footprint_spec[0].x * cos_th - footprint_spec[0].y * sin_th);
  new_y = y + (footprint_spec[0].x * sin_th + footprint_spec[0].y * cos_th);
  if(!costmap.worldToMap(new_x, new_y, x1, y1)) return clearAndReturn(footprint_cells);

  getLineCells(x0, x1, y0, y1, footprint_cells);

  if(fill) getFillCells(footprint_cells);

  return footprint_cells;
}

} /* namespace mbf_costmap_nav */
