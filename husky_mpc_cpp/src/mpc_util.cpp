#include "husky_mpc_cpp/mpc_util.hpp"

casadi::Function dynamics(const casadi::SX& states)
{
    casadi::SX theta = states(2);
    casadi::SX rot_3d_z = vertcat(horzcat(cos(theta), 0), horzcat(sin(theta), 0), horzcat(casadi::SX(0), 1));
    casadi::SX RHS = rot_3d_z;
    return casadi::Function("f", {states}, {RHS});
}



