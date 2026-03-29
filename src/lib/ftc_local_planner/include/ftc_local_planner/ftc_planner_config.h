#ifndef FTC_LOCAL_PLANNER__FTC_PLANNER_CONFIG_H_
#define FTC_LOCAL_PLANNER__FTC_PLANNER_CONFIG_H_

namespace ftc_local_planner
{

/**
 * @brief Configuration struct replacing ROS1 dynamic_reconfigure FTCPlannerConfig.
 *        All parameters are declared and updated via ROS2 parameter callbacks.
 */
struct FTCPlannerConfig
{
    // ControlPoint group
    double speed_fast = 0.5;
    double speed_fast_threshold = 1.5;
    double speed_fast_threshold_angle = 5.0;
    double speed_slow = 0.2;
    double speed_angular = 20.0;
    double acceleration = 1.0;

    // PID group
    double kp_lon = 1.0;
    double ki_lon = 0.0;
    double ki_lon_max = 10.0;
    double kd_lon = 0.0;
    double ki_lat = 0.0;
    double ki_lat_max = 10.0;
    double kp_lat = 1.0;
    double kd_lat = 0.0;
    double kp_ang = 1.0;
    double ki_ang = 0.0;
    double ki_ang_max = 10.0;
    double kd_ang = 0.0;

    // Robot group
    double max_cmd_vel_speed = 2.0;
    double max_cmd_vel_ang = 2.0;
    double max_goal_distance_error = 1.0;
    double max_goal_angle_error = 10.0;
    double goal_timeout = 5.0;
    double max_follow_distance = 1.0;

    // Top-level
    bool forward_only = true;
    bool restore_defaults = false;
    bool debug_pid = false;

    // Recovery group
    bool oscillation_recovery = true;
    double oscillation_v_eps = 5.0;
    double oscillation_omega_eps = 5.0;
    double oscillation_recovery_min_duration = 5.0;

    // Obstacles group
    bool check_obstacles = true;
    int obstacle_lookahead = 5;
    bool obstacle_footprint = true;
    bool debug_obstacle = true;
};

}  // namespace ftc_local_planner

#endif  // FTC_LOCAL_PLANNER__FTC_PLANNER_CONFIG_H_
