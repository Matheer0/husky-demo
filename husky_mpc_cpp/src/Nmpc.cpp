#include "husky_mpc_cpp/Nmpc.hpp"
#include <iostream>

NMPC::NMPC(const casadi::SX& prediction_states, const casadi::SX& prediction_controls, const casadi::DM& Q, const casadi::DM& R, int n_states, int n_controls,
        int N, double max_linear_acc, double max_angular_acc) : prediction_states_(prediction_states), prediction_controls_(prediction_controls),
        Q_(Q), R_(R), n_states_(n_states), n_controls_(n_controls), N_(N), max_linear_acc_(max_linear_acc), max_angular_acc_(max_angular_acc)
{
    lbx_ = casadi::DM::zeros((n_states_*(N_+1) + n_controls_*N_), 1);
    ubx_ = casadi::DM::zeros((n_states_*(N_+1) + n_controls_*N_), 1);
}

// Cost based on states and controls
std::tuple<casadi::SX, casadi::SX> NMPC::compute_cost(const casadi::SX& P, double time_interval, const casadi::Function& dynamics_function)
{
    casadi::SX cost_fn = 0;
    casadi::SX g = prediction_states_(casadi::Slice(), 0) - P(casadi::Slice(0, n_states_)); // Difference between actual and predicted states

    // Runge Kutta-4 Estimation
    for (int k = 0; k < N_; ++k)
    {
        casadi::SX current_state = prediction_states_(casadi::Slice(), k);
        casadi::SX control = prediction_controls_(casadi::Slice(), k);
        casadi::SX next_state = prediction_states_(casadi::Slice(), k+1); // Predicted next state
        casadi::SX distance_difference = next_state - P(casadi::Slice((k+1)*n_states_, (k+2)*n_states_), casadi::Slice());
        
        casadi::SX state_cost = mtimes(mtimes(distance_difference.T(), casadi::SX(Q_)), distance_difference);
        casadi::SX control_cost = mtimes(mtimes(control.T(), casadi::SX(R_)), control);
        cost_fn = cost_fn + state_cost +  control_cost;
        casadi::SX k1 = mtimes(dynamics_function(current_state)[0], control);
        casadi::SX k2 = mtimes(dynamics_function(current_state + time_interval/2*k1)[0], control);
        casadi::SX k3 = mtimes(dynamics_function(current_state + time_interval/2*k2)[0], control);
        casadi::SX k4 = mtimes(dynamics_function(current_state + time_interval * k3)[0], control);
        casadi::SX next_state_RK4 = current_state + (time_interval / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
        g = vertcat(g, next_state - next_state_RK4); // Difference between next state of optimal trajectory and next state approximation
    }
    return std::tuple<casadi::SX, casadi::SX>(cost_fn, g); 
}

// Obstacle avoidance constraints
casadi::SX NMPC::generate_obstacle_avoidance_constraints(const std::vector<Obstacle>& obstacle_list)
{
    casadi::SX distance;
    auto mask = casadi::DM({{1, 0, 0}, {0, 1, 0}}); // Mask to extract x and y coordinates from state vector
    for (auto obstacle : obstacle_list)
    {
        casadi::DM state_obstacle({obstacle.x_, obstacle.y_});
        for (int k = 0; k < N_; ++k)
        {
            auto current_state = prediction_states_(casadi::Slice(), k);
            distance = vertcat(casadi::SX(distance), norm_2(mtimes(casadi::SX(mask), current_state) - casadi::SX(state_obstacle))); // Append current obstacle distance to distance list
        }
    }
    return distance;
}

// Acceleration constraints
casadi::SX NMPC::generate_acceleration_constraints(double time_interval)
{
    casadi::SX linear_acc_contraints;
    casadi::SX angular_acc_contraints;
    for (int k = 0; k < N_-1; ++k)
    {
        casadi::SX linear_acceleration = (prediction_controls_(0, k+1) - prediction_controls_(0, k)) / time_interval;
        casadi::SX angular_acceleration = (prediction_controls_(1, k+1) - prediction_controls_(1, k)) / time_interval;
        linear_acc_contraints = vertcat(linear_acc_contraints, linear_acceleration);
        angular_acc_contraints = vertcat(angular_acc_contraints, angular_acceleration);
    }
    return vertcat(linear_acc_contraints, angular_acc_contraints);
}


std::map<std::string, casadi::DM> NMPC::generate_state_constraints(const casadi::SX& current_state, double safety_radius, double lower_x_constraint, 
            double upper_x_constraint, double lower_y_constraint, double upper_y_constraint, 
            double min_linear_speed, double max_linear_speed, double min_angular_speed, double max_angular_speed, const std::vector<Obstacle>& obstacle_list)
{
    //state lower bounds
    lbx_(casadi::Slice(0, n_states_*(N_+1), n_states_)) = lower_x_constraint;
    //look up lower y constraints
    for (int i = 0; i < N_+1; ++i)
    {
        lbx_(1 + n_states_*i) = lower_y_constraint;
    }
    //constraint for theta
    lbx_(casadi::Slice(2, n_states_*(N_+1), n_states_)) = 0;

    // state upper bounds
    ubx_(casadi::Slice(0, n_states_*(N_+1), n_states_)) = upper_x_constraint;
    //look up upper y constraints
    for (int i = 0; i < N_+1; ++i)
    {
        ubx_(1 + n_states_*i) = upper_y_constraint;
    }
    ubx_(casadi::Slice(2, n_states_*(N_+1), n_states_)) = 2 * casadi::pi;

    // lower and upper bounds for u
    for (int k = 0; k < N_; ++k)
    {
        lbx_(n_states_*(N_+1) + n_controls_*k) = min_linear_speed;
        lbx_(n_states_*(N_+1) + n_controls_*k + 1) = min_angular_speed;
        ubx_(n_states_*(N_+1) + n_controls_*k) = max_linear_speed;
        ubx_(n_states_*(N_+1) + n_controls_*k + 1) = max_angular_speed;
    }
    std::map<std::string, casadi::DM> args_nmpc = {
        // TODO: update constraints
        {"lbg", vertcat(1*casadi::DM::ones(obstacle_list.size() * N_, 1), 
                          casadi::DM::zeros(n_states_*(N_+1), 1),
                          -1*max_linear_acc_*casadi::DM::ones(N_-1, 1),
                          -1*max_angular_acc_*casadi::DM::ones(N_-1, 1)
                        )},  // constraints lower bound
        {"ubg", vertcat(400*casadi::DM::ones(obstacle_list.size() * N_, 1), 
                          casadi::DM::zeros(n_states_*(N_+1), 1),
                          max_linear_acc_*casadi::DM::ones(N_-1, 1),
                          max_angular_acc_*casadi::DM::ones(N_-1, 1)
                        )},  // constraints upper bound
        {"lbx", lbx_},
        {"ubx", ubx_}
    };
    args_nmpc["p"] = current_state;
    return args_nmpc;
}

// Define problem
std::map<std::string, casadi::SX> NMPC::define_problem(const casadi::SX& P, double dt, const casadi::Function& dynamics_function, const std::vector<Obstacle>& obstacle_list)
{
    // Call compute_cost
    auto cost_tuple = compute_cost(P, dt, dynamics_function); // Error here
    casadi::SX total_cost_nmpc = std::get<0>(cost_tuple);
    casadi::SX g_nmpc = std::get<1>(cost_tuple);


    // Generate obstacle avoidance constraints
    casadi::SX distance_to_obstacle = generate_obstacle_avoidance_constraints(obstacle_list);

    // Generate acceleration constraints
    casadi::SX acc_nmpc = generate_acceleration_constraints(dt);
    // Concatenate prediction_states and prediction_controls
    casadi::SX OPT_variables_nmpc = vertcat(reshape(prediction_states_, -1, 1), reshape(prediction_controls_, -1, 1));
    std::map<std::string, casadi::SX> nlp_prob_nmpc = {
        {"f", total_cost_nmpc},
        {"x", OPT_variables_nmpc},
        {"g", vertcat(distance_to_obstacle, g_nmpc, acc_nmpc)},
        {"p", P}
    };

    return nlp_prob_nmpc;
}
