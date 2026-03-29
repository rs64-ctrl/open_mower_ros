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
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

// Register this planner as a CostmapPlanner plugin for MBF
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, mbf_costmap_core::CostmapPlanner)

namespace global_planner {

void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalPlanner::GlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true),
        p_calc_(NULL), planner_(NULL), path_maker_(NULL), orientation_filter_(NULL),
        potential_array_(NULL) {
}

GlobalPlanner::GlobalPlanner(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string frame_id) :
        GlobalPlanner() {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

GlobalPlanner::~GlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (orientation_filter_)
        delete orientation_filter_;
}

void GlobalPlanner::initialize(
    std::string name,
    const rclcpp::Node::SharedPtr & node,
    tf2_ros::Buffer * /*tf*/,
    nav2_costmap_2d::Costmap2DROS * costmap_ros)
{
    node_ = node;
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void GlobalPlanner::initialize(std::string name, nav2_costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        costmap_ = costmap;
        frame_id_ = frame_id;

        // If node_ was not set via MBF initialize, we won't have publishers/services.
        // This path supports standalone usage.
        if (!node_) {
            RCLCPP_WARN(rclcpp::get_logger("global_planner"),
                "GlobalPlanner::initialize called without a ROS2 node. "
                "Publishers and services will not be available.");
        }

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        // Declare and get parameters
        auto declare_param = [&](const std::string & param_name, auto default_val) -> decltype(default_val) {
            if (node_) {
                std::string full_name = name + "." + param_name;
                if (!node_->has_parameter(full_name)) {
                    node_->declare_parameter(full_name, rclcpp::ParameterValue(default_val));
                }
                return static_cast<decltype(default_val)>(
                    node_->get_parameter(full_name).get_value<decltype(default_val)>());
            }
            return default_val;
        };

        old_navfn_behavior_ = declare_param("old_navfn_behavior", false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic = declare_param("use_quadratic", true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra = declare_param("use_dijkstra", true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path = declare_param("use_grid_path", false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        allow_unknown_ = declare_param("allow_unknown", true);
        planner_->setHasUnknown(allow_unknown_);
        planner_window_x_ = declare_param("planner_window_x", 0.0);
        planner_window_y_ = declare_param("planner_window_y", 0.0);
        default_tolerance_ = declare_param("default_tolerance", 0.0);
        publish_scale_ = declare_param("publish_scale", 100);
        outline_map_ = declare_param("outline_map", true);

        // Dynamic reconfigure replacement: initial values
        int lethal_cost = declare_param("lethal_cost", 253);
        int neutral_cost = declare_param("neutral_cost", 50);
        double cost_factor = declare_param("cost_factor", 3.0);
        publish_potential_ = declare_param("publish_potential", true);
        int orientation_mode = declare_param("orientation_mode", 1);
        int orientation_window_size = declare_param("orientation_window_size", 1);

        planner_->setLethalCost(static_cast<unsigned char>(lethal_cost));
        path_maker_->setLethalCost(static_cast<unsigned char>(lethal_cost));
        planner_->setNeutralCost(static_cast<unsigned char>(neutral_cost));
        planner_->setFactor(static_cast<float>(cost_factor));
        orientation_filter_->setMode(orientation_mode);
        orientation_filter_->setWindowSize(orientation_window_size);

        if (node_) {
            plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
                name + "/plan", 1);
            potential_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
                name + "/potential", 1);

            make_plan_srv_ = node_->create_service<nav_msgs::srv::GetPlan>(
                name + "/make_plan",
                std::bind(&GlobalPlanner::makePlanService, this,
                    std::placeholders::_1, std::placeholders::_2));

            // Register parameter change callback (replaces dynamic_reconfigure)
            param_callback_handle_ = node_->add_on_set_parameters_callback(
                std::bind(&GlobalPlanner::parameterCallback, this, std::placeholders::_1));
        }

        initialized_ = true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("global_planner"),
            "This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

rcl_interfaces::msg::SetParametersResult GlobalPlanner::parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // We need the planner name prefix; we just check each known suffix
    for (const auto & param : parameters) {
        const std::string & pname = param.get_name();

        if (pname.find("lethal_cost") != std::string::npos) {
            int val = static_cast<int>(param.as_int());
            planner_->setLethalCost(static_cast<unsigned char>(val));
            path_maker_->setLethalCost(static_cast<unsigned char>(val));
        } else if (pname.find("neutral_cost") != std::string::npos) {
            planner_->setNeutralCost(static_cast<unsigned char>(param.as_int()));
        } else if (pname.find("cost_factor") != std::string::npos) {
            planner_->setFactor(static_cast<float>(param.as_double()));
        } else if (pname.find("publish_potential") != std::string::npos) {
            publish_potential_ = param.as_bool();
        } else if (pname.find("orientation_mode") != std::string::npos) {
            orientation_filter_->setMode(static_cast<int>(param.as_int()));
        } else if (pname.find("orientation_window_size") != std::string::npos) {
            orientation_filter_->setWindowSize(static_cast<int>(param.as_int()));
        }
    }
    return result;
}

void GlobalPlanner::clearRobotCell(const geometry_msgs::msg::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"),
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

bool GlobalPlanner::makePlanService(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> req,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> resp)
{
    makePlan(req->start, req->goal, resp->plan.poses);

    resp->plan.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();
    resp->plan.header.frame_id = frame_id_;

    return true;
}

uint32_t GlobalPlanner::makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double tolerance,
    std::vector<geometry_msgs::msg::PoseStamped> & plan,
    double & cost,
    std::string & message)
{
    canceled_.store(false);
    bool success = makePlan(start, goal, tolerance, plan);
    if (success) {
        cost = 0.0;
        // Estimate cost as path length
        for (size_t i = 1; i < plan.size(); ++i) {
            double dx = plan[i].pose.position.x - plan[i-1].pose.position.x;
            double dy = plan[i].pose.position.y - plan[i-1].pose.position.y;
            cost += std::sqrt(dx*dx + dy*dy);
        }
        message = "Plan found";
        return 0;  // SUCCESS
    } else {
        cost = -1.0;
        message = "Failed to find a plan";
        return 50;  // PLANNING failure code
    }
}

bool GlobalPlanner::cancel() {
    canceled_.store(true);
    return true;
}

void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                           std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalPlanner::makePlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"),
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"),
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                global_frame.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    if (start.header.frame_id != global_frame) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"),
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
                global_frame.c_str(), start.header.frame_id.c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        RCLCPP_WARN(rclcpp::get_logger("global_planner"),
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        RCLCPP_WARN(rclcpp::get_logger("global_planner"),
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    if(outline_map_)
        outlineMap(costmap_->getCharMap(), nx, ny, nav2_costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);

    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::msg::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();
            plan.push_back(goal_copy);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("global_planner"),
                "Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"), "Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);

    //publish the plan for visualization purposes
    publishPlan(plan);
    delete[] potential_array_;
    return !plan.empty();
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
    if (!initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"),
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    if (!plan_pub_) return;

    //create a message for the plan
    nav_msgs::msg::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_->publish(gui_path);
}

bool GlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::msg::PoseStamped& goal,
                                       std::vector<geometry_msgs::msg::PoseStamped>& plan) {
    if (!initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"),
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        RCLCPP_ERROR(rclcpp::get_logger("global_planner"), "NO PATH!");
        return false;
    }

    rclcpp::Time plan_time = node_ ? node_->now() : rclcpp::Clock().now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void GlobalPlanner::publishPotential(float* potential)
{
    if (!potential_pub_) return;

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::msg::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else {
            if (fabs(max) < DBL_EPSILON) {
                grid.data[i] = -1;
            } else {
                grid.data[i] = potential_array_[i] * publish_scale_ / max;
            }
        }
    }
    potential_pub_->publish(grid);
}

} //end namespace global_planner
