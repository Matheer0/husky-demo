#ifndef NMPC_HPP
#define NMPC_HPP

#include <casadi/casadi.hpp>
#include <vector>
#include <map>
#include <tuple>

#include "husky_mpc_cpp/Map.hpp"

class NMPC
{
public:
    NMPC() = default; // default constructor (no arguments)
    NMPC(const casadi::SX& prediction_states, const casadi::SX& prediction_controls, const casadi::DM& Q, const casadi::DM& R, int n_states, int n_controls,
        int N, double max_linear_acc, double max_angular_acc);
    std::tuple<casadi::SX, casadi::SX> compute_cost(const casadi::SX& P, double time_interval, const casadi::Function& dynamics_function);
    casadi::SX generate_obstacle_avoidance_constraints(const std::vector<Obstacle>& obstacle_list);
    casadi::SX generate_acceleration_constraints(double time_interval);
    std::map<std::string, casadi::DM> generate_state_constraints(const casadi::SX& current_state, double safety_radius, double lower_x_constraint, double upper_x_constraint, double lower_y_constraint, double upper_y_constraint, 
                                   double min_linear_speed, double max_linear_speed, double min_angular_speed, double max_angular_speed, const std::vector<Obstacle>& obstacle_list);
    std::map<std::string, casadi::SX> define_problem(const casadi::SX& P, double dt, const casadi::Function& dynamics_function, const std::vector<Obstacle>& obstacle_list);


    
    casadi::SX prediction_states_;
    casadi::SX prediction_controls_;
    casadi::DM Q_;
    casadi::DM R_;
    casadi::DM lbx_;
    casadi::DM ubx_;

    int n_states_;
    int n_controls_;
    int N_;
    double max_linear_acc_;
    double max_angular_acc_;
};

#endif // NMPC_HPP