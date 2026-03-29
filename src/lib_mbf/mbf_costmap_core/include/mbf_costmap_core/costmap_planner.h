/*
 *  Copyright 2018, Sebastian Puetz
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  costmap_planner.h
 *
 *  author: Sebastian Puetz <spuetz@uni-osnabrueck.de>
 *
 */

#ifndef MBF_COSTMAP_CORE__COSTMAP_PLANNER_H_
#define MBF_COSTMAP_CORE__COSTMAP_PLANNER_H_

#include <mbf_abstract_core/abstract_planner.h>
#include <mbf_utility/types.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mbf_costmap_core {

class CostmapPlanner : public mbf_abstract_core::AbstractPlanner{
  public:

    typedef std::shared_ptr< ::mbf_costmap_core::CostmapPlanner > Ptr;

    virtual uint32_t makePlan(const geometry_msgs::msg::PoseStamped &start,
                              const geometry_msgs::msg::PoseStamped &goal,
                              double tolerance,
                              std::vector<geometry_msgs::msg::PoseStamped> &plan,
                              double &cost,
                              std::string &message) = 0;

    virtual bool cancel() = 0;

    /**
     * @brief Initialization function for the CostmapPlanner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    virtual void initialize(std::string name,
                            const rclcpp::Node::SharedPtr &node,
                            TF *tf,
                            nav2_costmap_2d::Costmap2DROS *costmap_ros) = 0;

    virtual ~CostmapPlanner(){}

  protected:
    CostmapPlanner(){}
};

}  /* namespace mbf_costmap_core */

#endif  /* MBF_COSTMAP_CORE__COSTMAP_PLANNER_H_ */
