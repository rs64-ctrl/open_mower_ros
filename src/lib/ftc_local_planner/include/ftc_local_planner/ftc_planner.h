
#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "ftc_local_planner/costmap_controller_interface.h"
#include "ftc_local_planner/oscillation_detector.h"
#include "ftc_local_planner/ftc_planner_config.h"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Forward declare generated service type
#include "ftc_local_planner/srv/planner_get_progress.hpp"
#include "ftc_local_planner/msg/pid.hpp"

namespace ftc_local_planner
{

    class FTCPlanner : public mbf_costmap_core::CostmapController
    {

        enum PlannerState
        {
            PRE_ROTATE,
            FOLLOWING,
            WAITING_FOR_GOAL_APPROACH,
            POST_ROTATE,
            FINISHED
        };

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Service<ftc_local_planner::srv::PlannerGetProgress>::SharedPtr progress_server_;

        // State tracking
        PlannerState current_state;
        rclcpp::Time state_entered_time;

        bool is_crashed;

        // Parameter callback handle
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

        tf2_ros::Buffer *tf_buffer;
        nav2_costmap_2d::Costmap2DROS *costmap;
        nav2_costmap_2d::Costmap2D *costmap_map_;

        std::vector<geometry_msgs::msg::PoseStamped> global_plan;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_point_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub;

        FTCPlannerConfig config;
        FTCPlannerConfig default_config;  // stores defaults for restore

        Eigen::Affine3d current_control_point;

        /**
         * PID State
         */
        double lat_error, lon_error, angle_error = 0.0;
        double last_lon_error = 0.0;
        double last_lat_error = 0.0;
        double last_angle_error = 0.0;
        double i_lon_error = 0.0;
        double i_lat_error = 0.0;
        double i_angle_error = 0.0;
        rclcpp::Time last_time;

        /**
         * Speed ramp for acceleration and deceleration
         */
        double current_movement_speed;

        /**
         * State for point interpolation
         */
        uint32_t current_index;
        double current_progress;
        Eigen::Affine3d local_control_point;

        /**
         * Private members
         */
        rclcpp::Publisher<ftc_local_planner::msg::PID>::SharedPtr pubPid;
        FailureDetector failure_detector_;
        rclcpp::Time time_last_oscillation_;
        bool oscillation_detected_ = false;
        bool oscillation_warning_ = false;

        double distanceLookahead();
        PlannerState update_planner_state();
        void update_control_point(double dt);
        void calculate_velocity_commands(double dt, geometry_msgs::msg::TwistStamped &cmd_vel);

        bool checkCollision(int max_points);
        bool checkOscillation(geometry_msgs::msg::TwistStamped &cmd_vel);
        void debugObstacle(visualization_msgs::msg::Marker &obstacle_points, double x, double y, unsigned char cost, int maxIDs);

        double time_in_current_state()
        {
            return (node_->now() - state_entered_time).seconds();
        }

        std::string plugin_name_;
        std::string p(const std::string& param) const;
        void declareParameters();
        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);

    public:
        FTCPlanner();

        void getProgress(
            const std::shared_ptr<ftc_local_planner::srv::PlannerGetProgress::Request> request,
            std::shared_ptr<ftc_local_planner::srv::PlannerGetProgress::Response> response);

        bool setPlan(const std::vector<geometry_msgs::msg::PoseStamped> &plan) override;

        void initialize(
            std::string name,
            const rclcpp::Node::SharedPtr & node,
            tf2_ros::Buffer *tf,
            nav2_costmap_2d::Costmap2DROS *costmap_ros) override;

        ~FTCPlanner() override;

        uint32_t
        computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                                const geometry_msgs::msg::TwistStamped &velocity,
                                geometry_msgs::msg::TwistStamped &cmd_vel,
                                std::string &message) override;

        bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

        bool cancel() override;
    };
};
#endif
