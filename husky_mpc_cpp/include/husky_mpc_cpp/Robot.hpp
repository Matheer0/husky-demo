#ifndef ROBOT_HPP
#define ROBOT_HPP




class Robot
{
public:
    Robot() = default; // default constructor (no arguments)
    Robot(double safety_radius, double linear_v_min, double linear_v_max, double angular_v_min, double angular_v_max);
    // Eigen::VectorXd inverse_kinematics(void);
    // void set_robot_velocity(double linear_velocity, double angular_velocity);
    // void update_state(double dt);
    // void update();
    // Eigen::VectorXd forward_kinematics(void);

    double safety_radius_;
    double linear_v_min_;
    double linear_v_max_;
    double angular_v_min_;
    double angular_v_max_;
    double b_;
    double r_;

    // Eigen::VectorXd location_;
    // Eigen::VectorXd x_dot_;
    // Eigen::VectorXd wheel_speed_;
    // Eigen::MatrixXd car_dims_;
    // Eigen::Matrix B_matrix_;
    // Eigen::MatrixXd ikine_mat_;
    // Eigen::MatrixXd kine_mat_;
};

#endif // ROBOT_HPP