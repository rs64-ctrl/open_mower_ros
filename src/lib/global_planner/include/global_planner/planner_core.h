#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#define POT_HIGH 1.0e10        // unassigned cell potential

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <vector>
#include <mutex>
#include <atomic>
#include <global_planner/costmap_planner_interface.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/expander.h>
#include <global_planner/traceback.h>
#include <global_planner/orientation_filter.h>

namespace global_planner {

class Expander;
class GridPath;

/**
 * @class PlannerCore
 * @brief Provides a ROS2 wrapper for the global_planner planner which runs a fast,
 *        interpolated navigation function on a costmap.
 *        Implements the MBF CostmapPlanner interface for use with mbf_costmap_nav.
 */

class GlobalPlanner : public mbf_costmap_core::CostmapPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        GlobalPlanner();

        /**
         * @brief  Constructor for the PlannerCore object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        GlobalPlanner(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the PlannerCore object
         */
        ~GlobalPlanner();

        /**
         * @brief  MBF CostmapPlanner initialize interface
         * @param  name The name of this planner
         * @param  node Shared pointer to the parent ROS2 node
         * @param  tf Pointer to a tf2 buffer
         * @param  costmap_ros A pointer to the Costmap2DROS to use for planning
         */
        void initialize(
            std::string name,
            const rclcpp::Node::SharedPtr & node,
            tf2_ros::Buffer * tf,
            nav2_costmap_2d::Costmap2DROS * costmap_ros) override;

        /**
         * @brief  Legacy initialize without node (for standalone use)
         */
        void initialize(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief MBF makePlan interface
         */
        uint32_t makePlan(
            const geometry_msgs::msg::PoseStamped & start,
            const geometry_msgs::msg::PoseStamped & goal,
            double tolerance,
            std::vector<geometry_msgs::msg::PoseStamped> & plan,
            double & cost,
            std::string & message) override;

        /**
         * @brief MBF cancel interface
         */
        bool cancel() override;

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                      std::vector<geometry_msgs::msg::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::msg::PoseStamped>& plan);

        /**
         * @brief  Computes the full navigation function for the map given a point in the world to start from
         * @param world_point The point to use for seeding the navigation function
         * @return True if the navigation function was computed successfully, false otherwise
         */
        bool computePotential(const geometry_msgs::msg::Point& world_point);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed
         */
        bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                                  const geometry_msgs::msg::PoseStamped& goal,
                                  std::vector<geometry_msgs::msg::PoseStamped>& plan);

        /**
         * @brief Get the potential, or navigation cost, at a given point in the world
         */
        double getPointPotential(const geometry_msgs::msg::Point& world_point);

        /**
         * @brief Check for a valid potential value at a given point in the world
         */
        bool validPointPotential(const geometry_msgs::msg::Point& world_point);
        bool validPointPotential(const geometry_msgs::msg::Point& world_point, double tolerance);

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path);

        bool makePlanService(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
                             std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp);

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        nav2_costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
        bool initialized_, allow_unknown_;

        // ROS2 node handle for parameter access and publishers (protected for subclass access)
        rclcpp::Node::SharedPtr node_;

    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const geometry_msgs::msg::PoseStamped& global_pose, unsigned int mx, unsigned int my);
        void publishPotential(float* potential);

        double planner_window_x_, planner_window_y_, default_tolerance_;
        std::mutex mutex_;
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr make_plan_srv_;

        PotentialCalculator* p_calc_;
        Expander* planner_;
        Traceback* path_maker_;
        OrientationFilter* orientation_filter_;

        bool publish_potential_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub_;
        int publish_scale_;

        void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

        float* potential_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;

        bool old_navfn_behavior_;
        float convert_offset_;

        bool outline_map_;

        // ROS2 parameter callback handle (replaces dynamic_reconfigure)
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        rcl_interfaces::msg::SetParametersResult parameterCallback(
            const std::vector<rclcpp::Parameter> & parameters);

        // Cancel flag for MBF
        std::atomic<bool> canceled_{false};
};

} //end namespace global_planner

#endif
