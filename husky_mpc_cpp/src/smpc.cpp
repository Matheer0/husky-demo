#include "husky_mpc_cpp/smpc.hpp"
#include <iostream>
#include <algorithm>    // std::min
#include <boost/math/special_functions/erf.hpp> // error fuinction erf

SMPC::SMPC(const casadi::SX& prediction_states, const casadi::SX& prediction_controls, const casadi::DM& Q, const casadi::DM& R, 
            int n_states, int n_controls, int N, double risk_parameter, double max_linear_acc, double max_angular_acc) : 
        prediction_states_(prediction_states), 
        prediction_controls_(prediction_controls),
        Q_(Q), 
        R_(R), 
        n_states_(n_states), 
        n_controls_(n_controls), 
        N_(N), 
        risk_parameter_(risk_parameter),
        max_linear_acc_(max_linear_acc), 
        max_angular_acc_(max_angular_acc)
{
    lbx_ = casadi::DM::zeros((n_states_*(N_+1) + n_controls_*N_), 1);
    ubx_ = casadi::DM::zeros((n_states_*(N_+1) + n_controls_*N_), 1);
}



// Cost based on states and controls
std::tuple<casadi::SX, casadi::SX> SMPC::compute_cost(const casadi::SX& P, double time_interval, const casadi::Function& dynamics_function)
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
casadi::SX SMPC::generate_obstacle_avoidance_constraints(const std::vector<Obstacle>& obstacle_list)
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
casadi::SX SMPC::generate_acceleration_constraints(double time_interval)
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


std::map<std::string, casadi::DM> SMPC::generate_state_constraints(
            double lower_x_constraint, double upper_x_constraint, 
            double lower_y_constraint, double upper_y_constraint, const std::vector<std::vector<double>>& gamma_list,
            double max_linear_speed, double max_angular_speed, 
            double max_safety_distance, const std::vector<Obstacle>& obstacle_list)
{
    //constraint for theta
    lbx_(casadi::Slice(2, n_states_*(N_+1), n_states_)) = -casadi::pi;
    ubx_(casadi::Slice(2, n_states_*(N_+1), n_states_)) = casadi::pi;

    // lower and upper bounds for x and y
    for (int i = 0; i < N_; ++i)
    {   
        lbx_(0 + n_states_*i) = lower_x_constraint + gamma_list[0][i];
        lbx_(1 + n_states_*i) = lower_y_constraint + gamma_list[1][i];


        ubx_(0 + n_states_*i) = upper_x_constraint - gamma_list[0][i];
        ubx_(1 + n_states_*i) = upper_y_constraint - gamma_list[1][i];
    }

    lbx_(0 + n_states_*N_) = lower_x_constraint + gamma_list[0][N_-1];
    lbx_(1 + n_states_*N_) = lower_y_constraint + gamma_list[1][N_-1];

    ubx_(0 + n_states_*N_) = upper_x_constraint - gamma_list[0][N_-1];
    ubx_(1 + n_states_*N_) = upper_y_constraint - gamma_list[1][N_-1];
    

    // LOWER and UPPER BOUNDS for CONTROL u
    for (int k = 0; k < N_; ++k)
    {
        lbx_(n_states_*(N_+1) + n_controls_*k) = -1 * max_linear_speed;
        lbx_(n_states_*(N_+1) + n_controls_*k + 1) = -1 * max_angular_speed;
        ubx_(n_states_*(N_+1) + n_controls_*k) = max_linear_speed;
        ubx_(n_states_*(N_+1) + n_controls_*k + 1) = max_angular_speed;
    }

    casadi::DM min_distance;
    for (auto obstacle : obstacle_list)
    {
        min_distance = vertcat(casadi::DM(min_distance), 
                            obstacle.radius_*casadi::DM::ones(N_, 1)); // append minimum safety distance to current obstacle
    }


    ///*
    for (size_t i = 0; i < obstacle_list.size(); ++i)
    {
        for (int j = 0; j < N_; ++j)
        {
            min_distance(i*N_ +j)+= sqrt(pow(gamma_list[0][j],2) + pow(gamma_list[1][j],2));
        }

    }
    //*/


    std::map<std::string, casadi::DM> args_smpc = {
        {"lbg", vertcat(min_distance, 
                          casadi::DM::zeros(n_states_*(N_+1), 1),
                          -1*max_linear_acc_*casadi::DM::ones(N_-1, 1),
                          -1*max_angular_acc_*casadi::DM::ones(N_-1, 1)
                        )},  // constraints lower bound
        {"ubg", vertcat(max_safety_distance*casadi::DM::ones(obstacle_list.size() * N_, 1), 
                          casadi::DM::zeros(n_states_*(N_+1), 1),
                          max_linear_acc_*casadi::DM::ones(N_-1, 1),
                          max_angular_acc_*casadi::DM::ones(N_-1, 1)
                        )},  // constraints upper bound
        {"lbx", lbx_},
        {"ubx", ubx_}
    };

    return args_smpc;
}


std::vector<std::vector<double>> SMPC::compute_gamma(Eigen::MatrixXd& A_matrix, Eigen::MatrixXd& dynamics_covariance)
{
    std::vector<std::vector<double>> gamma_prediction(3, std::vector<double>(N_, 0));

    // filter matrix
    Eigen::MatrixXd g_x = Eigen::MatrixXd::Zero(3,1);
    g_x(0,0) = 1;
    Eigen::MatrixXd g_y = Eigen::MatrixXd::Zero(3,1);
    g_y(1) = 1;
    Eigen::MatrixXd g_theta = Eigen::MatrixXd::Zero(3,1);
    g_theta(2) = 1;

    //sigma_k initialization
    Eigen::MatrixXd prediction_covariance = Eigen::MatrixXd::Zero(3,3);

    //G_k, 3x3 identity matrix
    Eigen::MatrixXd G_k = Eigen::Matrix3d::Identity();

    for (int k=0; k<N_; ++k)
    {   
        // update covariance in prediction horizon: step k+1
        prediction_covariance = A_matrix*prediction_covariance*A_matrix.transpose() + G_k*dynamics_covariance*G_k.transpose();

        // Compute the inverse error function using Boost
        // https://www.boost.org/doc/libs/1_83_0/libs/math/doc/html/math_toolkit/sf_erf/error_inv.html
        double risk_parameter = boost::math::erf_inv(2*risk_parameter_-1);

        double gamma_x = (g_x.transpose() * prediction_covariance * g_x).value();
        gamma_x = sqrt(2*gamma_x);

        double gamma_y = (g_y.transpose() * prediction_covariance * g_y).value();
        gamma_y = sqrt(2*gamma_y);

        double gamma_theta = (g_theta.transpose() * prediction_covariance * g_theta).value();
        gamma_theta = sqrt(2*gamma_theta);

        
        gamma_prediction[0][k] = gamma_x*risk_parameter; 
        gamma_prediction[1][k] = gamma_y*risk_parameter; 
        gamma_prediction[2][k] = gamma_theta*risk_parameter; 

        // N=100: gamma_prediction[N] ~= 5.13
        
    }
    // std::cout<< gamma_prediction<< std::endl;
	return gamma_prediction; 

}


// Define problem
std::map<std::string, casadi::SX> SMPC::define_problem(const casadi::SX& P, double dt, const casadi::Function& dynamics_function, const std::vector<Obstacle>& obstacle_list)
{
    // Call compute_cost
    auto cost_tuple = compute_cost(P, dt, dynamics_function); // Error here
    casadi::SX total_cost_smpc = std::get<0>(cost_tuple);
    casadi::SX g_smpc = std::get<1>(cost_tuple);


    // Generate obstacle avoidance constraints
    casadi::SX distance_to_obstacle = generate_obstacle_avoidance_constraints(obstacle_list);

    // Generate acceleration constraints
    casadi::SX acc_smpc = generate_acceleration_constraints(dt);
    // Concatenate prediction_states and prediction_controls
    casadi::SX OPT_variables_smpc = vertcat(reshape(prediction_states_, -1, 1), reshape(prediction_controls_, -1, 1));
    std::map<std::string, casadi::SX> nlp_prob_smpc = {
        {"f", total_cost_smpc},
        {"x", OPT_variables_smpc},
        {"g", vertcat(distance_to_obstacle, g_smpc, acc_smpc)},
        {"p", P}
    };

    return nlp_prob_smpc;
}
