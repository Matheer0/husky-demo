#ifndef SMPC_HPP
#define SMPC_HPP

#include <casadi/casadi.hpp>
#include <vector>
#include <map>
#include <tuple>
#include <Eigen/Dense>

#include "husky_mpc_cpp/map.hpp"

class SMPC
{
public:
    SMPC() = default; // default constructor (no arguments)

    SMPC(const casadi::SX& prediction_states, const casadi::SX& prediction_controls, 
            const casadi::DM& Q, const casadi::DM& R, int n_states, int n_controls,
            int N, double state_safety_probability, double obstacle_avoidance_safety_probability,
            double max_linear_acc, double max_angular_acc);

    std::tuple<casadi::SX, casadi::SX> compute_cost(const casadi::SX& P, double time_interval, const casadi::Function& dynamics_function);
    casadi::SX generate_obstacle_avoidance_constraints(const std::vector<Obstacle>& obstacle_list);
    casadi::SX generate_acceleration_constraints(double time_interval);


    std::map<std::string, casadi::DM> generate_state_constraints(double lower_x_constraint, double upper_x_constraint, 
                                                                 double lower_y_constraint, double upper_y_constraint,
                                                                 double max_linear_speed, double max_angular_speed, 
                                                                 double extra_distance, double max_safety_distance, 
                                                                 const std::vector<std::vector<double>>& gamma_list, 
                                                                 const std::vector<Obstacle>& obstacle_list);

    std::vector<std::vector<double>> compute_gamma(Eigen::MatrixXd& A_matrix, Eigen::MatrixXd& dynamics_covariance);

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
    double state_safety_probability_;
    double obstacle_avoidance_safety_probability_;
    double max_linear_acc_;
    double max_angular_acc_;
};

#endif // SMPC_HPP