#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <iostream>
#include <Eigen/Dense>




class Robot
{
public:
    Robot() = default; // default constructor (no arguments)
    Robot(double linear_v,
        double angular_v, 
        double linear_v_max, 
        double angular_v_max);
    // Eigen::VectorXd inverse_kinematics(void);
    // void set_robot_velocity(double linear_velocity, double angular_velocity);
    // void update_state(double dt);
    // void update();
    // Eigen::VectorXd forward_kinematics(void);

    double current_linear_v_;
    double current_angular_v_;

    double linear_v_max_;
    double angular_v_max_;

    // Eigen::VectorXd location_;
    // Eigen::MatrixXd B_matrix_;

    // Eigen::VectorXd location_;
    // Eigen::VectorXd x_dot_;
    // Eigen::VectorXd wheel_speed_;
    // Eigen::MatrixXd car_dims_;
    // Eigen::MatrixXd ikine_mat_;
    // Eigen::MatrixXd kine_mat_;
};

#endif // ROBOT_HPP