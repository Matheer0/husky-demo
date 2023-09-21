#!/usr/bin/env python3
import threading
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from gazebo_msgs.msg import ModelStates, LinkStates
from geometry_msgs.msg  import Twist, Point
from nav_msgs.msg  import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
import math
import control as ctrl
from scipy.special import erfinv
from husky_mpc.util import dynamics, DM2Arr
from husky_mpc.map import Map
from husky_mpc.nmpc import NMPC
from husky_mpc.robot import Robot

import pinocchio as pin


class MPCNode(Node):

    def __init__(self):
        super().__init__('mpc_node')
        self.velocity_publisher =  self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel_unstamped', 10)
        
        # Set up state symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(
            x,
            y,
            theta
        )   # [x, y, theta]
        self.n_states = states.numel() # 3 states

        self.dynamics_covariance = np.array([[0.2, 0.1, 0.1], [0.1, 0.2, 0.1], [0.1, 0.1, 0.2]])

        # new parameters for straight trajectory
        y_reference = 0
        self.off_set = 10

        self.dt = 0.2  # time between steps in seconds
        self.N = 35              # number of look ahead steps

        # Husky Physical Properties
        # https://github.com/husky/husky/blob/677120693643ca4b6ed3c14078dedbb8ced3b781/husky_control/config/control.yaml#L29-L42
        linear_v_max = 1.0               # m/s, default 1.0
        linear_acceleration_max = 2.0    # m/s^2, default 3.0

        angular_v_max = 2.0              # rad/s, default 2.0
        angular_acceleration_max = 4.0   # rad/s^2, default 6.0


        # coloumn vector for storing initial state and target state
        P = ca.SX.sym('P', self.n_states + self.n_states*self.N) # robot state + N reference states, self.n_states*(N+1) * 1 list

        # setting matrix_weights' variables
        Q_x = 1
        Q_y = 1
        Q_theta = 2
        R1 = 2
        R2 = 3
        Q = ca.diagcat(Q_x, Q_y, Q_theta)   # state weights matrix (Q_X, Q_Y, Q_THETA)
        R = ca.diagcat(R1, R2)  # controls weights matrix
        self.Q_array = [[Q_x, 0, 0], [0, Q_y, 0], [0, 0, Q_theta]]
        self.R_array = [[R1, 0], [0, R2]]

        
        # CONSTRUCT REFERECE TRAJECTORY
        self.x_init = 0
        y_init = y_reference
        theta_init = 0
        self.x_target = 30
        y_target = y_reference
        theta_target = 0
        obstacles = 3

        # initial and target states of robot
        self.state_init = ca.DM([[self.x_init], 
                            [y_init], 
                            [theta_init]
                            ])        
        self.state_target = ca.DM([[self.x_target], 
                                   [y_target], 
                                   [theta_target]
                                   ]) 
        
        # set up robot
        safety_radius = 4
        self.robot_nmpc = Robot(self.x_init, y_init, theta_init, safety_radius, -1*linear_v_max, linear_v_max, -1*angular_v_max,  angular_v_max, self.dt)

        self.map = Map(self.x_init, y_init, self.x_target, y_target, theta_target, y_reference, self.off_set, obstacles)
        self.map.generate_circular_obstacles(self.x_init+5, self.x_target-5, 3)


        # CONTROL SYMBOLIC VARIABLES
        V_a = ca.SX.sym('V_a')
        V_b = ca.SX.sym('V_b')
        controls = ca.vertcat(
            V_a,
            V_b
        ) # [V_a, V_b]
        self.n_controls = controls.numel() # get the number of control elements, = 2


        # matrix containing all states over all time steps +1 (each column is a state vector)
        # self.n_states row by (N + 1) col symbolic primitive
        X_nmpc = ca.SX.sym('X', self.n_states, self.N + 1) # nominal mpc 

        # matrix containing all control actions over all time steps (each column is an action vector)
        U_nmpc = ca.SX.sym('U', self.n_controls, self.N)

        # maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T, i.e. compute matrix B
        self.dynamics_function = dynamics(states)


        #
        #  SET UP NMPC PROBLEM
        #
        self.nmpc_problem = NMPC(X_nmpc, U_nmpc, Q, R, self.n_states, self.n_controls, 
                                 self.N, linear_acceleration_max, angular_acceleration_max)
        nlp_prob_nmpc = self.nmpc_problem.define_problem(P, self.dt, self.dynamics_function, self.map.obstacle_list)

        self.state_init_nmpc = self.state_init

        self.horizon_controls_nmpc = ca.DM.zeros((self.n_controls, self.N))  # initial control
        self.horizon_states_nmpc = ca.repmat(self.state_init_nmpc, 1, self.N+1)       # initial state full
        self.trajectory_nmpc = DM2Arr(self.state_init_nmpc) #                stores robot's real states


        # OPTIMIZATION PARAMETERS
        opts = {
            'ipopt': {
                'max_iter': 2000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }

        self.solver_nmpc = ca.nlpsol('solver', 'ipopt', nlp_prob_nmpc, opts)

        self.mpc_iter = 0


        # for Gazebo simulation only
        self.control_timer = self.create_timer(self.dt, self.timer_callback)
        self.ground_truth_subscriber_ = self.create_subscription(Odometry, "/ground_truth", self.extract_ground_truth_state, 10) 

            
    def update_dynamics(self, current_state, f):
        rotation_matrix = f(current_state)
        # linear_velocity = rotation_matrix@u

        matrix_B = self.dt*rotation_matrix # update speed, type: casadi.DM
        return DM2Arr(matrix_B)


    def shift_timestep_ground_truth(self, current_state, u, f):
        rotation_matrix = f(current_state) # update speed, type: casadi.DM
        matrix_B = self.dt*rotation_matrix
        next_state = ca.DM.full(current_state + (matrix_B @ u)) # new [x, y, theta]
        return next_state



    def timer_callback(self):
        print(self.state_init_nmpc)
        print("-----------")

        if ca.norm_2(self.state_init_nmpc - self.state_target) > 1:

            noise = np.zeros(self.n_states)
            #if mpc_iter > 0:
                # noise in dynamics
            noise = np.random.multivariate_normal(np.zeros(self.n_states), self.dynamics_covariance)
            
            # CONSTRUCT NMPC CONSTRAINTS
            args_nmpc = self.nmpc_problem.generate_state_constraints(self.state_init_nmpc, noise, self.robot_nmpc.safety_radius, 
                                                                     self.x_init, self.x_target, 
                                                                     self.map.y_lower_limit, self.map.y_upper_limit, 
                                                                     self.robot_nmpc.linear_v_min, self.robot_nmpc.linear_v_max, 
                                                                     self.robot_nmpc.angular_v_min, self.robot_nmpc.angular_v_max, self.map.obstacle_list)

            # target states
            for j in range(self.N):

                args_nmpc['p'] = ca.vertcat(
                    args_nmpc['p'],
                    self.state_target 
                )

            # initial guess for optimization variables
            args_nmpc['x0'] = ca.vertcat(
                ca.reshape(self.horizon_states_nmpc, self.n_states*(self.N+1), 1),
                ca.reshape(self.horizon_controls_nmpc, self.n_controls*self.N, 1)   # turn horizoself.n_states t0 1*2N dimension
            )

            # NMPC
            result_nmpc = self.solver_nmpc(
                x0=args_nmpc['x0'],
                lbx=args_nmpc['lbx'],
                ubx=args_nmpc['ubx'],
                lbg=args_nmpc['lbg'],
                ubg=args_nmpc['ubg'],
                p=args_nmpc['p']
            )
            # STORE HORIZON INFO
            control_results_nmpc = ca.reshape(result_nmpc['x'][self.n_states * (self.N + 1):], self.n_controls, self.N) # 2*N
            self.horizon_states_nmpc = ca.reshape(result_nmpc['x'][: self.n_states * (self.N+1)], self.n_states, self.N+1)


            # UPDATE ROBOT STATE
            # PUBLISH VELOCITY TO /husky_velocity_controller/cmd_vel_unstamped
            vel_msg = Twist()
            vel_msg.linear.x = float(control_results_nmpc[:, 0][0])
            vel_msg.angular.z = float(control_results_nmpc[:, 0][1])
            self.velocity_publisher.publish(vel_msg)
            
            


            # USE HORIZON INFO AS INITIAL GUESS FOR NEXT ITERATION
            self.horizon_controls_nmpc = ca.horzcat(
                control_results_nmpc[:, 1:],
                ca.reshape(control_results_nmpc[:, -1], -1, 1)
            )
            self.horizon_states_nmpc = ca.horzcat(
                self.horizon_states_nmpc[:, 1:],
                ca.reshape(self.horizon_states_nmpc[:, -1], -1, 1) 
            )



            # Customized for non-ROS simulations below
            # update robot position using control from NMPC
            self.trajectory_nmpc = np.dstack((
                self.trajectory_nmpc, 
                self.state_init_nmpc
            ))

            self.mpc_iter += 1

        else:
            print("done")
 
            self.destroy_node()



    def extract_ground_truth_state(self, msg: Odometry): 
        
        # GROUND TRUTH ORIENTATION
        orientation_4d_quaternion = np.array([msg.pose.pose.orientation.x,
                                    msg.pose.pose.orientation.y,
                                    msg.pose.pose.orientation.z,
                                    msg.pose.pose.orientation.w])    
        ori = pin.Quaternion(orientation_4d_quaternion)
        # transform into 3D
        orientation_ground_truth = pin.utils.matrixToRpy(ori.matrix())

        # GROUND TRUTH STATE
        self.state_init_nmpc = ca.DM([[msg.pose.pose.position.x], 
                            [msg.pose.pose.position.y], 
                            [orientation_ground_truth[2]]
                            ])  


def main(args=None):    
    rclpy.init(args=args)
    node = MPCNode()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    
    rate = node.create_rate(500)


    
    try:
        while rclpy.ok():
            rate.sleep()

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()
    thread.join()

    


    
