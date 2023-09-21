from time import time
import time as tm
import casadi as ca
import numpy as np
from numpy.linalg import norm



class NMPC:
    def __init__(self, prediction_states, prediction_controls, Q, R, n_states, n_controls, N, max_linear_acc, max_angular_acc):
        self.prediction_states = prediction_states
        self.prediction_controls = prediction_controls
        self.Q = Q
        self.R = R
		
        self.n_states = n_states
        self.n_controls = n_controls
        self.N = N  # prediction horizons

        self.max_linear_acc = max_linear_acc
        self.max_angular_acc = max_angular_acc

        self.lbx = ca.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        self.ubx = ca.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))


    def compute_cost(self, P, time_interval, dynamics_function):
        cost_fn = 0  # cost function

        g = self.prediction_states[:, 0] - P[: self.n_states]
        # runge kutta
        for k in range(self.N):
            current_state = self.prediction_states[:, k]
            # control = U[:, k] - K_matrix@current_state
            control = self.prediction_controls[:, k]
    
            next_state = self.prediction_states[:, k+1]
            distance_difference = next_state - P[(k+1)*self.n_states:(k+2)*self.n_states, :]
            cost_fn = cost_fn \
                + distance_difference.T @ self.Q @ distance_difference \
                + control.T @ self.R @ control
    
            k1 = dynamics_function(current_state) @ control
            k2 = dynamics_function(current_state + time_interval/2*k1) @ control
            k3 = dynamics_function(current_state + time_interval/2*k2) @ control
            k4 = dynamics_function(current_state + time_interval * k3) @ control
            next_state_RK4 = current_state + (time_interval / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g = ca.vertcat(g, next_state - next_state_RK4)

        return cost_fn, g 


    # obstacle avoidance
    def generate_obstacle_avoidance_constraints(self, obstacle_list):
        distance = []
        mask = ca.DM([[1, 0, 0], 
                  [0, 1, 0]]) 
        
        for i in range(len(obstacle_list)):
            obstacle = obstacle_list[i]
            #state_obstacle = ca.DM([obstacle.x, obstacle.y, 0]) 
            state_obstacle = ca.DM([obstacle.x, obstacle.y]) 

            for k in range(self.N):
                current_state = self.prediction_states[:, k]
                # we only consider x and y, not including theta
                distance = ca.vertcat(distance, ca.norm_2(mask@current_state - state_obstacle))
        return distance
    

    # control constraints
    def generate_acceleration_contraints(self, time_interval):
        linear_acc_contraints = []
        angular_acc_contraints = []

        for k in range(self.N-1):
            # a = dv/dt
            linear_acceleration = (self.prediction_controls[0, k+1] - self.prediction_controls[0, k])/time_interval
            angular_acceleration = (self.prediction_controls[1, k+1] - self.prediction_controls[1, k])/time_interval

            linear_acc_contraints = ca.vertcat(linear_acc_contraints, linear_acceleration)
            angular_acc_contraints = ca.vertcat(angular_acc_contraints, angular_acceleration)
        
        acceleration_contraints = ca.vertcat(linear_acc_contraints, angular_acc_contraints)
        return acceleration_contraints


    def define_problem(self, P, dt, dynamics_function, obstacle_list):
        total_cost_nmpc, g_nmpc = self.compute_cost(P, dt, dynamics_function)
        distance_to_obstacle = self.generate_obstacle_avoidance_constraints(obstacle_list)

        # linear and angular acceleration contraints
        #acc_nmpc = self.generate_test_contraints()
        acc_nmpc = self.generate_acceleration_contraints(dt)

        OPT_variables_nmpc = ca.vertcat(
            self.prediction_states.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
            self.prediction_controls.reshape((-1, 1))
        )

        nlp_prob_nmpc = {
            'f': total_cost_nmpc,
            'x': OPT_variables_nmpc,
            'g': ca.vertcat(distance_to_obstacle, g_nmpc, acc_nmpc),
            'p': P
        }

        return nlp_prob_nmpc
    

	# GENERATE x, y and theta CONSTRAINTS FOR NMPC
    # TODO: add map parameters as input arguments 
    def generate_state_constraints(self, current_state, noise, safety_radius, lower_x_constraint, upper_x_constraint, lower_y_constraint, upper_y_constraint, 
                                   min_linear_speed, max_linear_speed, min_angular_speed, max_angular_speed, obstacle_list):
        noised_state = current_state + noise

        # state lower bounds
        self.lbx[0: self.n_states*(self.N+1): self.n_states] = lower_x_constraint

        # look up lower y constraints
        for i in range(self.N+1):
            self.lbx[1+ self.n_states*i] = lower_y_constraint
        self.lbx[2: self.n_states*(self.N+1): self.n_states] = -ca.pi


        # state upper bounds
        self.ubx[0: self.n_states*(self.N+1): self.n_states] = upper_x_constraint
        
        # look up upper y constraints
        for i in range(self.N+1):
            self.ubx[1+ self.n_states*i] = upper_y_constraint
        self.ubx[2: self.n_states*(self.N+1): self.n_states] = ca.pi

        # lower and upper bounds for u
        for k in range(self.N):
            self.lbx[self.n_states*(self.N+1) + self.n_controls*k: self.n_states*(self.N+1) + self.n_controls*(k+1)] = ca.DM([min_linear_speed, min_angular_speed])
            self.ubx[self.n_states*(self.N+1) + self.n_controls*k: self.n_states*(self.N+1) + self.n_controls*(k+1)] = ca.DM([max_linear_speed, max_angular_speed])
        
        args_nmpc = {
            'lbg': ca.vertcat(safety_radius*ca.DM.ones((len(obstacle_list) * self.N, 1)), 
                              ca.DM.zeros((self.n_states*(self.N+1), 1)),
                              -1*self.max_linear_acc*ca.DM.ones(self.N-1, 1),
                              -1*self.max_angular_acc*ca.DM.ones(self.N-1, 1)
                            ),  # constraints lower bound
            'ubg': ca.vertcat(400*ca.DM.ones((len(obstacle_list) * self.N, 1)), 
                              ca.DM.zeros((self.n_states*(self.N+1), 1)),
                              self.max_linear_acc*ca.DM.ones(self.N-1, 1),
                              self.max_angular_acc*ca.DM.ones(self.N-1, 1)
                            ),  # constraints upper bound
            'lbx': self.lbx,
            'ubx': self.ubx
        }
        args_nmpc['p'] = noised_state

        return args_nmpc
