#include "husky_mpc_cpp/map.hpp"
#include "husky_mpc_cpp/robot.hpp"
#include "husky_mpc_cpp/smpc.hpp"
#include "husky_mpc_cpp/mpc_util.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <casadi/casadi.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tuple>
#include <map>
#include <iostream>
#include <memory>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <Eigen/Dense>
#include <random>
#define M_PI 3.14159265358979323846


#include <algorithm>
#include <iterator>
#include <vector>

class SMPCNode : public rclcpp::Node
{
public:
    SMPCNode();

    casadi::DM update_dynamics(const casadi::DM& current_state, const casadi::Function& f);
    casadi::DM shift_timestep_ground_truth(const casadi::DM& current_state, const casadi::DM& u, const casadi::Function& f);
    void timer_callback();
    void extract_ground_truth_state(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscriber_;

    casadi::DM state_target_;
    casadi::Function dynamics_function_;
    casadi::DM state_init_smpc_;
    Eigen::VectorXd state_init_smpc_vector_ = Eigen::VectorXd(3);
    casadi::DM horizon_controls_smpc_;
    casadi::DM horizon_states_smpc_;
    casadi::Function solver_smpc_;

    Eigen::MatrixXd A_matrix_ = Eigen::MatrixXd::Zero(3, 3);
    std::vector<Eigen::VectorXd> state_list_;

    Robot robot_smpc_;
    Map map_;
    SMPC smpc_problem_;

    int n_states_;
    int N_; // number of look ahead steps
    int n_controls_; // Number of control inputs
    int mpc_iter_;
    int timing_violation_ = 0;

    double stop_distance_;
    double max_distance_to_obstacle_;
    double obstacle_extra_distance_;
    double off_set_;
    double dt_;
    double x_init_;
    double x_target_;

    double ave_compute_time_ = 0;
    double min_compute_time_ = 100;
    double max_compute_time_ = 0;
    int result_unsaved = 1;

    Eigen::MatrixXd dynamics_covariance_ = Eigen::MatrixXd({{ 0.2, 0.1, 0.1},
                                                            {0.1, 0.2, 0.1},
                                                            {0.1, 0.1, 0.3}});
    

    // Eigen::MatrixXd dynamics_covariance_ = Eigen::MatrixXd::Zero(3, 3);
};


SMPCNode::SMPCNode() : rclcpp::Node("smpc_node")
{
    // Set up state symbolic variables
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::SX theta = casadi::SX::sym("theta");
    casadi::SX states = vertcat(x, y, theta);   // [x, y, theta]
    n_states_ = states.numel(); // 3 states

    // Control symbolic variables
    casadi::SX V_a = casadi::SX::sym("V_a");
    casadi::SX V_b = casadi::SX::sym("V_b");
    casadi::SX controls = vertcat(V_a, V_b); // [V_a, V_b]
    n_controls_ = controls.numel(); // get the number of control elements, = 2

    // safety paramets for SMPC
    double state_safety_probability = 0.8;
    double orientation_safety_probability = 0.7;    // working value: 0.7
    double obstacle_avoidance_safety_probability = 0.95;

    obstacle_extra_distance_ = 1.0;
    max_distance_to_obstacle_ = 1000.0; 

    dt_ = 0.1;  // time between steps in seconds
    N_ = 100; // number of look ahead steps

    dt_ = 0.05;  
    N_ = 50; 

    //dt_ = 0.1;  
    //N_ = 100;


    // Husky Physical Properties
    // https://github.com/husky/husky/blob/677120693643ca4b6ed3c14078dedbb8ced3b781/husky_control/config/control.yaml#L29-L42
    double linear_v_max = 1.0; // m/s, default 1.0
    double linear_acceleration_max = 3.0; // m/s^2, default 3.0

    double angular_v_max = 2.0; // rad/s, default 2.0
    double angular_acceleration_max = 6.0; // rad/s^2, default 6.0

    // coloumn vector for storing initial state and target state
    casadi::SX P = casadi::SX::sym("P", n_states_ + n_states_*N_); // robot state + N reference states, self.n_states*(N+1) * 1 list
    
    double Q_x = 1;
    double Q_y = 1;
    double Q_theta = 2;
    casadi::DM Q = diagcat(casadi::DM(Q_x), Q_y, Q_theta);
    double R1 = 2;
    double R2 = 3;
    casadi::DM R = diagcat(casadi::DM(R1), R2);

    A_matrix_(0,0) = 1;
    A_matrix_(1,1) = 1;
    A_matrix_(2,2) = 1;


    ///*
    // Map Parameters for Long Rooms
    double x_center = 40;
    double y_center = 0;
    off_set_ = 5;

    x_target_ = 30;
    double y_target = 0;
    double theta_target = 0;

    // Construct reference trajectory
    x_init_ = 0;
    double y_init = 0;
    double theta_init = 0;
    //*/



    /*
    // Map Parameters for Square Rooms
    double x_center = 10;
    double y_center = 0;
    off_set_ = 10;

    x_target_ = 17;
    double y_target = 0;
    double theta_target = 0;
    
    // Construct reference trajectory
    x_init_ = 2.;
    double y_init = -8;
    double theta_init = 1.6;
    */


    // initial and target states of robot
    state_init_smpc_ = casadi::DM({{x_init_}, {y_init}, {theta_init}});
    state_target_ = casadi::DM({{x_target_}, {y_target}, {theta_target}});
    state_init_smpc_vector_ << x_init_, y_init, theta_init;

    // set up robot
    robot_smpc_ = Robot(0., 0., linear_v_max, angular_v_max);

    map_ = Map(x_init_, y_init, x_target_, y_target, theta_target, 
        x_center, y_center, off_set_);
    map_.generate_circular_obstacles();


    // Matrix containing all states over all time steps +1 (each column is a state vector)
    casadi::SX X_smpc = casadi::SX::sym("X", n_states_, N_+1); 

    // Matrix containing all control actions over all time steps (each column is an action vector)
    casadi::SX U_smpc = casadi::SX::sym("U", n_controls_, N_);

    // Maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T, i.e. compute matrix B
    dynamics_function_ = dynamics(states);

    //
    // Set up SMPC problem
    //
    smpc_problem_ = SMPC(X_smpc, U_smpc, Q, R, 
                         n_states_, n_controls_, N_, 
                         state_safety_probability, orientation_safety_probability, obstacle_avoidance_safety_probability, 
                         linear_acceleration_max, angular_acceleration_max);
    std::map<std::string, casadi::SX> nlp_prob_smpc = smpc_problem_.define_problem(P, dt_, dynamics_function_, map_.obstacle_list); // Dynamics func here

    horizon_controls_smpc_ = casadi::DM::zeros(n_controls_, N_); // initial control
    horizon_states_smpc_ = repmat(state_init_smpc_, 1, N_+1); // initial state full

    casadi::Dict opts;
    casadi::Dict ipopt_opts;
    ipopt_opts["max_iter"] = 2000;
    ipopt_opts["print_level"] = 0;
    ipopt_opts["acceptable_tol"] = 1e-8;
    ipopt_opts["acceptable_obj_change_tol"] = 1e-6;
    opts["ipopt"] = ipopt_opts;
    opts["print_time"] = 0;
    solver_smpc_ = casadi::nlpsol("solver", "ipopt", nlp_prob_smpc, opts);

    mpc_iter_ = 0;
    stop_distance_ = 0.5;
    RCLCPP_INFO(this->get_logger(), "Finishing initialization");

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky_velocity_controller/cmd_vel_unstamped", 1);
    ground_truth_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ground_truth", 1, std::bind(&SMPCNode::extract_ground_truth_state, this, std::placeholders::_1));
    auto duration = std::chrono::milliseconds((int)(dt_*1000));
    control_timer_ = this->create_wall_timer(duration, std::bind(&SMPCNode::timer_callback, this));
}


// Unused
casadi::DM SMPCNode::update_dynamics(const casadi::DM& current_state, const casadi::Function& f)
{
    casadi::DM rotation_matrix = f(current_state)[0];
    casadi::DM matrix_B = dt_*rotation_matrix; // update speed, type: casadi.DM
    return matrix_B;
}


// Unused
casadi::DM SMPCNode::shift_timestep_ground_truth(const casadi::DM &current_state, const casadi::DM &u, const casadi::Function &f)
{
    casadi::DM rotation_matrix = f(current_state)[0]; // update speed, type: casadi.DM
    casadi::DM matrix_B = dt_*rotation_matrix;
    casadi::DM next_state = current_state + mtimes(matrix_B, u); // new [x, y, theta]
    return next_state;
}




void SMPCNode::timer_callback()
{   
    //RCLCPP_INFO_STREAM(this->get_logger(), state_init_smpc_ << "\n-------------");

    auto mask = casadi::DM({{1, 0, 0}, {0, 1, 0}}); // Mask to extract x and y coordinates from state vector
    double distance_to_target = (double)norm_2(mtimes(mask, state_init_smpc_) - mtimes(mask, state_target_));
    if (distance_to_target > stop_distance_)
    {   
        // timing computation time
        std::clock_t start = std::clock();


        // NOISE in DYNAMICS
        // use a random seed based on the current time
        unsigned seed = static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator(seed);

        // create a standard normal distribution
        std::normal_distribution<double> distribution(0.0, 1.0);
        Eigen::VectorXd noised_state(n_states_);
        for (int j = 0; j < n_states_; j++) {
            // generate random number from normal distribution
            noised_state(j) = distribution(generator);
        }
        // transform using the Cholesky decomposition of the covariance matrix
        noised_state = dynamics_covariance_.llt().matrixL() * noised_state + state_init_smpc_vector_;
        // make noised oirentation into proper state space: [-pi, pi]
        if (noised_state(2) < -M_PI)
        {   
            std::cout << "=========" << std::endl;
            noised_state(2) += 2*M_PI;
        }
        if (noised_state(2) > M_PI)
        {   
            std::cout << "=======" << std::endl;
            noised_state(2) -= 2*M_PI;
        }


        // store state for plot
        if (mpc_iter_ > 0){
            state_list_.push_back(state_init_smpc_vector_);
        }

        // CONSTRUCT SMPC CONSTRAINTS
        std::vector<std::vector<double>> gamma_prediction = smpc_problem_.compute_gamma(A_matrix_, dynamics_covariance_);
        // std::cout << "gamma_prediction:" << std::endl << gamma_prediction << std::endl;
        // std::cout << "============================" << std::endl;

        auto args_smpc = smpc_problem_.generate_state_constraints(
                map_.x_lower_limit_, map_.x_upper_limit_, 
                map_.y_lower_limit_, map_.y_upper_limit_,
                robot_smpc_.linear_v_max_, robot_smpc_.angular_v_max_, 
                obstacle_extra_distance_, max_distance_to_obstacle_, 
                gamma_prediction, map_.obstacle_list);

        // convert the Eigen::VectorXd to a CasADi DM
        casadi::DM noised_state_dm{std::vector<double>(noised_state.data(), noised_state.size() + noised_state.data())};
        
        args_smpc["p"] = noised_state_dm;
        // target states
        for (int j = 0; j < N_; ++j)
        {
            args_smpc["p"] = vertcat(casadi::SX(args_smpc["p"]), casadi::SX(state_target_));
        }

        // initial guess for optimization variables
        args_smpc["x0"] = vertcat(reshape(horizon_states_smpc_, n_states_*(N_+1), 1), reshape(horizon_controls_smpc_, n_controls_*N_, 1));

        // SOLVE OPTIMIZATION PROBLEM
        auto result_smpc = solver_smpc_(args_smpc);
        // STORE HORIZON INFO
        auto control_results_smpc = reshape(result_smpc["x"](casadi::Slice(n_states_*(N_+1), n_states_*(N_+1) + n_controls_*N_)), n_controls_, N_);
        horizon_states_smpc_ = reshape(result_smpc["x"](casadi::Slice(0, n_states_*(N_+1))), n_states_, N_+1);


        // PUBLISH VELOCITY TO /husky_velocity_controller/cmd_vel_unstamped
        geometry_msgs::msg::Twist vel_msg;
        double linear_v = (double)control_results_smpc(casadi::Slice(), 0)(0);
        double angular_v = (double)control_results_smpc(casadi::Slice(), 0)(1);

        if (std::abs(linear_v) > robot_smpc_.linear_v_max_){
            double sign = 1.0;
            if (std::signbit(linear_v) == 1){
                // negative velocity
                sign = -1.0;
            }
            linear_v = sign * robot_smpc_.linear_v_max_;
        }

        if (std::abs(angular_v) > robot_smpc_.angular_v_max_){
            double sign = 1.0;
            if (std::signbit(angular_v) == 1){
                // negative velocity
                sign = -1.0;
            }
            angular_v = sign * robot_smpc_.angular_v_max_;
        }

        // update robot speed
        robot_smpc_.current_linear_v_ = linear_v;
        robot_smpc_.current_angular_v_ = angular_v;

        // send speed to ROS
        vel_msg.linear.x = linear_v;
        vel_msg.angular.z = angular_v;
        velocity_publisher_->publish(vel_msg);


        //
        // USE HORIZON INFO AS INITIAL GUESS FOR NEXT ITERATION
        //
        // auto B_smpc = update_dynamics(state_init_smpc_, dynamics_function_);
        horizon_controls_smpc_ = horzcat(control_results_smpc(casadi::Slice(), casadi::Slice(1, N_)), reshape(control_results_smpc(casadi::Slice(), N_-1), -1, 1));
        horizon_states_smpc_ = horzcat(horizon_states_smpc_(casadi::Slice(), casadi::Slice(1, N_+1)), reshape(horizon_states_smpc_(casadi::Slice(), N_-1), -1, 1));

        ++mpc_iter_;


        double duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        if (duration > max_compute_time_){
            max_compute_time_ = duration;
        }
        if (duration < min_compute_time_){
            min_compute_time_ = duration;
        }
        if (duration > dt_){
            timing_violation_ += 1;
        }
        ave_compute_time_ += duration;
        // std::cout<<"smpc duration: "<< duration <<"\n-------------\n";
    } else {
        // PUBLISH VELOCITY TO /husky_velocity_controller/cmd_vel_unstamped
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        velocity_publisher_->publish(vel_msg);

        if (result_unsaved){
            std::cout<<"smpc max_compute_time: "<< max_compute_time_ <<'\n';
            std::cout<<"smpc min_compute_time: "<< min_compute_time_ <<'\n';
            std::cout<<"violations: "<< timing_violation_ <<'\n';
            std::cout<<"smpc ave_compute_time: "<< ave_compute_time_/mpc_iter_ <<'\n';
            std::cout<<"iterations: "<< mpc_iter_ <<'\n';


            // Specify the file path
            std::string filePath = "/home/alex/a_thesis/smpc_cpp.csv";

            // Open the CSV file for writing
            std::ofstream outputFile(filePath);
            if (!outputFile.is_open()) {
                std::cerr << "Unable to open the output file." << std::endl;
            }

            // Iterate through each Eigen::VectorXd and write to the CSV file
            for (const auto& vector : state_list_) {
                for (int i = 0; i < vector.size(); ++i) {
                    outputFile << vector(i);
                    // Add a comma if it's not the last element in the row
                    if (i < vector.size() - 1) {
                        outputFile << ",";
                    }
                }
                outputFile << "\n";  // New line for the next row
            }

            outputFile.close();
            std::cout << "CSV file written successfully." << std::endl;

            result_unsaved = 0;
        }
        
    }
}

void SMPCNode::extract_ground_truth_state(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double x= msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    state_init_smpc_ = casadi::DM({{x}, {y}, {yaw}});
    state_init_smpc_vector_ << x, y, yaw;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SMPCNode>());
    //std::cout<<"ave_compute_time: "<< SMPCNode.ave_compute_time_/SSMPCNode.mpc_iter_ <<'\n';
    rclcpp::shutdown();
}

