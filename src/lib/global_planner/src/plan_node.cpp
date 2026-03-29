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
 * Author: Bhaskara Marthi
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

namespace global_planner {

class PlannerWithCostmap : public GlobalPlanner {
    public:
        PlannerWithCostmap(
            const rclcpp::Node::SharedPtr & node,
            const std::string & name,
            nav2_costmap_2d::Costmap2DROS * cmap);

        bool makePlanService(
            const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
            std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp);

    private:
        void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
        nav2_costmap_2d::Costmap2DROS* cmap_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr make_plan_service_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

bool PlannerWithCostmap::makePlanService(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp)
{
    std::vector<geometry_msgs::msg::PoseStamped> path;

    auto start = req->start;
    auto goal = req->goal;
    start.header.frame_id = "map";
    goal.header.frame_id = "map";
    bool success = makePlan(start, goal, path);

    if (success) {
        resp->plan.poses = path;
    }
    resp->plan.header.stamp = node_->now();
    resp->plan.header.frame_id = "map";

    return true;
}

void PlannerWithCostmap::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
    geometry_msgs::msg::PoseStamped global_pose;
    cmap_->getRobotPose(global_pose);
    std::vector<geometry_msgs::msg::PoseStamped> path;
    makePlan(global_pose, *goal, path);
}

PlannerWithCostmap::PlannerWithCostmap(
    const rclcpp::Node::SharedPtr & node,
    const std::string & name,
    nav2_costmap_2d::Costmap2DROS * cmap)
    : GlobalPlanner(name, cmap->getCostmap(), cmap->getGlobalFrameID()),
      node_(node)
{
    cmap_ = cmap;
    // Also set the node_ on the base class so publishers work
    GlobalPlanner::node_ = node;

    make_plan_service_ = node_->create_service<nav_msgs::srv::GetPlan>(
        "make_plan",
        std::bind(&PlannerWithCostmap::makePlanService, this,
            std::placeholders::_1, std::placeholders::_2));

    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", 1,
        std::bind(&PlannerWithCostmap::poseCallback, this, std::placeholders::_1));
}

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("global_planner");

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "costmap", std::string(""), std::string("costmap"), false);

    auto planner = std::make_shared<global_planner::PlannerWithCostmap>(
        node, "planner", costmap_ros.get());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
