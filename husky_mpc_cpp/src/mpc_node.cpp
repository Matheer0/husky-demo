#include "husky_mpc_cpp/Map.hpp"
#include "husky_mpc_cpp/Robot.hpp"
#include "husky_mpc_cpp/Nmpc.hpp"
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


class MPCNode : public rclcpp::Node
{
public:
    MPCNode();
    // def update_dynamics(self, current_state, f):
    casadi::DM update_dynamics(const casadi::DM& current_state, const casadi::Function& f);
    casadi::DM shift_timestep_ground_truth(const casadi::DM& current_state, const casadi::DM& u, const casadi::Function& f);
    void timer_callback();
    void extract_ground_truth_state(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscriber_;

    casadi::DM state_init_;
    casadi::DM state_target_;
    casadi::Function dynamics_function_;
    casadi::DM state_init_nmpc_;
    casadi::DM horizon_controls_nmpc_;
    casadi::DM horizon_states_nmpc_;
    casadi::Function solver_nmpc_;

    casadi::DM Q_array_;
    casadi::DM R_array_;
    casadi::DM trajectory_nmpc_;

    Robot robot_nmpc_;
    Map map_;
    NMPC nmpc_problem_;

    int n_states_;
    int N_; // number of look ahead steps
    int n_controls_; // Number of control inputs
    int mpc_iter_;
    double off_set_;
    double dt_;
    double x_init_;
    double x_target_;
};


MPCNode::MPCNode() : rclcpp::Node("mpc_node")
{
    // Set up state symbolic variables
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::SX theta = casadi::SX::sym("theta");
    casadi::SX states = vertcat(x, y, theta);   // [x, y, theta]
    n_states_ = states.numel(); // 3 states


    // New parameters for straight trajectory
    double safety_radius = 4;
    double y_reference = 0;
    off_set_ = 10;

    dt_ = 0.2;  // time between steps in seconds
    N_ = 35; // number of look ahead steps

    // Husky Physical Properties
    // https://github.com/husky/husky/blob/677120693643ca4b6ed3c14078dedbb8ced3b781/husky_control/config/control.yaml#L29-L42
    double linear_v_max = 1.0; // m/s, default 1.0
    double linear_acceleration_max = 2.0; // m/s^2, default 3.0

    double angular_v_max = 2.0; // rad/s, default 2.0
    double angular_acceleration_max = 4.0; // rad/s^2, default 6.0

    // coloumn vector for storing initial state and target state
    casadi::SX P = casadi::SX::sym("P", n_states_ + n_states_*N_); // robot state + N reference states, self.n_states*(N+1) * 1 list
    
    double Q_x = 1;
    double Q_y = 1;
    double Q_theta = 2;
    casadi::DM Q = diagcat(casadi::DM(Q_x), Q_y, Q_theta);
    double R1 = 2;
    double R2 = 3;
    casadi::DM R = diagcat(casadi::DM(R1), R2);
    Q_array_ = Q;
    R_array_ = R;
    
    // Construct reference trajectory
    x_init_ = 0;
    double y_init = y_reference;
    double theta_init = 0;
    x_target_ = 30;
    double y_target = y_reference;
    double theta_target = 0;
        

    // initial and target states of robot
    state_init_ = casadi::DM({{x_init_}, {y_init}, {theta_init}});
    state_target_ = casadi::DM({{x_target_}, {y_target}, {theta_target}});

    // set up robot
    robot_nmpc_ = Robot(safety_radius, -1*linear_v_max, linear_v_max, -1*angular_v_max,  angular_v_max);
    map_ = Map(x_init_, y_init, x_target_, y_target, theta_target, y_reference, off_set_);
    map_.generate_circular_obstacles();

    // Control symbolic variables
    casadi::SX V_a = casadi::SX::sym("V_a");
    casadi::SX V_b = casadi::SX::sym("V_b");
    casadi::SX controls = vertcat(V_a, V_b); // [V_a, V_b]
    n_controls_ = controls.numel(); // get the number of control elements, = 2

    // Matrix containing all states over all time steps +1 (each column is a state vector)
    casadi::SX X_nmpc = casadi::SX::sym("X", n_states_, N_+1); // nominal mpc

    // Matrix containing all control actions over all time steps (each column is an action vector)
    casadi::SX U_nmpc = casadi::SX::sym("U", n_controls_, N_);

    // Maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T, i.e. compute matrix B
    dynamics_function_ = dynamics(states);


    // Set up NMPC problem
    nmpc_problem_ = NMPC(X_nmpc, U_nmpc, Q, R, n_states_, n_controls_, N_, linear_acceleration_max, angular_acceleration_max);
    //CONFIRMED
    std::map<std::string, casadi::SX> nlp_prob_nmpc = nmpc_problem_.define_problem(P, dt_, dynamics_function_, map_.obstacle_list); // Dynamics func here
    state_init_nmpc_ = state_init_;

    horizon_controls_nmpc_ = casadi::DM::zeros(n_controls_, N_); // initial control
    horizon_states_nmpc_ = repmat(state_init_nmpc_, 1, N_+1); // initial state full
    trajectory_nmpc_ = state_init_nmpc_; // stores robot's real states

    casadi::Dict opts;
    casadi::Dict ipopt_opts;
    ipopt_opts["max_iter"] = 2000;
    ipopt_opts["print_level"] = 0;
    ipopt_opts["acceptable_tol"] = 1e-8;
    ipopt_opts["acceptable_obj_change_tol"] = 1e-6;

    opts["ipopt"] = ipopt_opts;
    opts["print_time"] = 0;

    solver_nmpc_ = casadi::nlpsol("solver", "ipopt", nlp_prob_nmpc, opts);
    mpc_iter_ = 0;
    RCLCPP_INFO(this->get_logger(), "Finishing initialization");

    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/husky_velocity_controller/cmd_vel_unstamped", 1);
    ground_truth_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ground_truth", 1, std::bind(&MPCNode::extract_ground_truth_state, this, std::placeholders::_1));
    auto duration = std::chrono::milliseconds((int)(dt_*1000));
    control_timer_ = this->create_wall_timer(duration, std::bind(&MPCNode::timer_callback, this));
}

casadi::DM MPCNode::update_dynamics(const casadi::DM& current_state, const casadi::Function& f)
{
    casadi::DM rotation_matrix = f(current_state)[0];
    casadi::DM matrix_B = dt_*rotation_matrix; // update speed, type: casadi.DM
    return matrix_B;
}

casadi::DM MPCNode::shift_timestep_ground_truth(const casadi::DM &current_state, const casadi::DM &u, const casadi::Function &f)
{
    casadi::DM rotation_matrix = f(current_state)[0]; // update speed, type: casadi.DM
    casadi::DM matrix_B = dt_*rotation_matrix;
    casadi::DM next_state = current_state + mtimes(matrix_B, u); // new [x, y, theta]
    return next_state;
}


void MPCNode::timer_callback()
{   
    RCLCPP_INFO_STREAM(this->get_logger(), state_init_nmpc_ << "\n-------------");
    if ((double)norm_2(state_init_nmpc_ - state_target_) > 1)
    {
        auto args_nmpc = nmpc_problem_.generate_state_constraints(state_init_nmpc_, robot_nmpc_.safety_radius_, 
                map_.x_init_, map_.x_target_, map_.y_lower_limit_, map_.y_upper_limit_, 
                robot_nmpc_.linear_v_min_, robot_nmpc_.linear_v_max_, robot_nmpc_.angular_v_min_, 
                robot_nmpc_.angular_v_max_, map_.obstacle_list);
        for (int j = 0; j < N_; ++j)
        {
            args_nmpc["p"] = vertcat(casadi::SX(args_nmpc["p"]), casadi::SX(state_target_));
        }

        args_nmpc["x0"] = vertcat(reshape(horizon_states_nmpc_, n_states_*(N_+1), 1), reshape(horizon_controls_nmpc_, n_controls_*N_, 1));

        auto result_nmpc = solver_nmpc_(args_nmpc);
        auto control_results_nmpc = reshape(result_nmpc["x"](casadi::Slice(n_states_*(N_+1), n_states_*(N_+1) + n_controls_*N_)), n_controls_, N_);
        horizon_states_nmpc_ = reshape(result_nmpc["x"](casadi::Slice(0, n_states_*(N_+1))), n_states_, N_+1);

        auto B_nmpc = update_dynamics(state_init_nmpc_, dynamics_function_);
        //state_init_nmpc_ = state_init_nmpc_ + mtimes(B_nmpc, casadi::DM(control_results_nmpc(casadi::Slice(), 0)));
        horizon_controls_nmpc_ = horzcat(control_results_nmpc(casadi::Slice(), casadi::Slice(1, N_)), reshape(control_results_nmpc(casadi::Slice(), N_-1), -1, 1)); // ERROR HERE
        horizon_states_nmpc_ = horzcat(horizon_states_nmpc_(casadi::Slice(), casadi::Slice(1, N_+1)), reshape(horizon_states_nmpc_(casadi::Slice(), N_-1), -1, 1));

        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = (double)control_results_nmpc(casadi::Slice(), 0)(0);
        vel_msg.angular.z = (double)control_results_nmpc(casadi::Slice(), 0)(1);
        velocity_publisher_->publish(vel_msg);

        // Update robot position using control from NMPC
        trajectory_nmpc_ = horzcat(trajectory_nmpc_, state_init_nmpc_); // TODO: Double check this
        ++mpc_iter_;
    }
}

void MPCNode::extract_ground_truth_state(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double x= msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    state_init_nmpc_ = casadi::DM({{x}, {y}, {yaw}});
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCNode>());
    rclcpp::shutdown();
}

