#pragma once

#include <string>
#include <vector>

namespace param
{

struct Param
{
    struct Navigation
    {
        double xy_goal_tolerance;
        double yaw_goal_tolerance;
    };
    Navigation navigation;

    struct TargetSystem
    {
        double l_f;
        double l_r;
        double d_l;
        double d_r;
        double tire_radius;
    };
    TargetSystem target_system;

    struct Controller
    {
        std::string name;
        double control_interval;
        int num_samples;
        int prediction_horizon;
        double step_len_sec;
        double param_exploration;
        double param_lambda;
        double param_alpha;
        std::vector<double> sigma;
        bool reduce_computation;
        double vx_min;
        double vx_max;
        double vy_min;
        double vy_max;
        double omega_min;
        double omega_max;
        std::vector<double> weight_cmd_change;
        std::vector<double> weight_vehicle_cmd_change;
        double ref_velocity;
        double weight_velocity_error;
        double weight_angular_error;
        double weight_collision_penalty;
        double weight_distance_error_penalty;
        double weight_terminal_state_penalty;
        double weight_lateral_velocity_penalty;
        double best_sample_blend;
        bool use_sg_filter;
        int sg_filter_half_window_size;
        int sg_filter_poly_order;
    };
    Controller controller;
};

} // namespace param
