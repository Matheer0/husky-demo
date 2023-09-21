import casadi as ca
import numpy as np
from casadi import sin, cos, pi

# ROBOT PROPERTY PARAMETER 
wheel_radius = 1    # wheel radius
Lx = 0.3            # L in J Matrix (half robot x-axis length)
Ly = 0.3            # l in J Matrix (half robot y-axis length)

# CONTROL PARAMETERS
# setting matrix_weights' variables
Q_x = 100
Q_y = 100
Q_theta = 100
R1 = 1
R2 = 1
R3 = 1
R4 = 1
# coloumn vector for storing initial state and target state
Q = ca.diagcat(Q_x, Q_y, Q_theta)   # state weights matrix (Q_X, Q_Y, Q_THETA)
R = ca.diagcat(R1, R2, R3, R4)  # controls weights matrix
Q_array = [[100, 0, 0], [0, 100, 0], [0, 0, 100]]
R_array = [[R1, 0, 0, 0], [0, R2, 0, 0], [0, 0, R3, 0], [0, 0, 0, R4]]


def dynamics(states):
    theta = states[2]
    # DISCRETIZATION MODEL (e.g. x2 = f(x1, v, t) = x1 + v * time_interval)
    rot_3d_z = ca.vertcat(
        ca.horzcat(cos(theta), 0),
        ca.horzcat(sin(theta), 0),
        ca.horzcat(         0, 1)
    )
    # Mecanum wheel transfer function which can be found here: 
    # https://www.researchgate.net/publication/334319114_Model_Predictive_Control_for_a_Mecanum-wheeled_robot_in_Dynamical_Environments
    J = (wheel_radius/4) * ca.DM([
        [         1,         1,          1,         1],
        [        -1,         1,          1,        -1],
        [-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)]
    ])

    # RHS = states + J @ controls * time_interval  # Euler discretization
    RHS = rot_3d_z
    # ratation matrix that maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T, i.e. compute new velocity
    return ca.Function('f', [states], [RHS])

# Turn DM matrix to ARRAY
def DM2Arr(dm):
    return np.array(dm.full())
