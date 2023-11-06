#include "husky_mpc_cpp/Robot.hpp"
#include <iostream>
#include <Eigen/Dense>

Robot::Robot(double linear_v, double angular_v, double x, double y, double omega,
    double linear_v_max, double angular_v_max, double dt): 
        current_linear_v_(linear_v), current_angular_v_(angular_v), 
        linear_v_max_(linear_v_max), angular_v_max_(angular_v_max)
{
    //Eigen::VectorXd location_ = Eigen::VectorXd({x, y, omega});

    // x_dot = Eigen::VectorXd({0, 0, 0});
    // wheel_speed_ = Eigen::VectorXd({0, 0});

    // b_ = 0.25;
    // r_ = 0.05;

    // car_dims = Eigen::MatrixXd({{-b_, -b_, 1},
    //                             {0, -b_, 1},
    //                             {b_, 0, 1},
    //                             {0, b_, 1},
    //                             {-b_, b_, 1}});
    

    /*
    Eigen::MatrixXd B_matrix_(3,2);
    B_matrix_(0,0) = cos(location_(2, 0))*dt;
    B_matrix_(0,1) = 0;
    B_matrix_(1,0) = sin(location_(2, 0))*dt;
    B_matrix_(1,1) = 0;
    B_matrix_(2,0) = 0;
    B_matrix_(2,1) = dt;
    */

    // ikine_mat_ = Eigen::MatrixXd({{1/r_, 0, b_/r_},
    //                             {1/r_, 0, -b_/r_}});

    // kine_mat_ = Eigen::MatrixXd({{r_/2, r_/2},
    //                             {0, 0},
    //                             {r_/(2*b_), -r_/(2*b_)}});
}

// Eigen::VectorXd Robot::inverse_kinematics()
// {
//     return ikine_mat_ * x_dot_;
// }

// void Robot::set_robot_velocity(double linear_velocity, double angular_velocity)
// {
//     x_dot_(0) = linear_velocity;
//     x_dot_(1) = 0;
//     x_dot_(2) = angular_velocity;
// }

// void Robot::update_state(double dt)
// {
//     Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3);
//     Eigen::MatrixXd B = Eigen::MatrixXd({{cos(location_(2, 0))*dt, 0},
//                                         {sin(location_(2, 0))*dt, 0},
//                                         {0, dt}});

//     Eigen::VectorXd vel = Eigen::VectorXd({x_dot_(0), x_dot_(2)});
//     location_ = A*location_ + B*vel;
// }

// Eigen::VectorXd Robot::forward_kinematics()
// {
//     return kine_mat_ * wheel_speed_;
// }