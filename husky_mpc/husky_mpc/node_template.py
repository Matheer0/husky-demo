#!/usr/bin/env python3
import threading
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from gazebo_msgs.msg import ModelStates, LinkStates
from geometry_msgs.msg  import Twist
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
from husky_mpc.smpc import SMPC
from husky_mpc.robot import Robot
from husky_mpc.simulation_code import simulate




class MPCNode(Node):

    def __init__(self):
        super().__init__('mpc_node')
        self.velocity_publisher =  self.create_publisher(Twist, '/husky_velocity_controller/cmd_vel_unstamped', 10)



        '''
        add 2D code below
        '''



   
        '''
        add code above
        '''

        

        self.control_timer = self.create_timer(self.dt, self.timer_callback)

            
    # def update_dynamics
        

    # def shift_timestep_ground_truth
   
    


    def lqr(self, actual_state_x, desired_state_xf, Q, R, A, B):
        x_error = actual_state_x - desired_state_xf
        N = 100
        P = [None] * (N + 1)
        Qf = Q
        P[N] = Qf
        for i in range(N, 0, -1):
            P[i-1] = Q + A.T @ P[i] @ A - (A.T @ P[i] @ B) @ np.linalg.pinv(
                R + B.T @ P[i] @ B) @ (B.T @ P[i] @ A)      
        K = [None] * N
        u = [None] * N
        for i in range(N):
            K[i] = -np.linalg.pinv(R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ A
            u[i] = K[i] @ x_error
        K_star = K[N-1]
        return K_star


    def timer_callback(self):
        #self.get_logger().info("test")
        if ca.norm_2(self.state_init_smpc - self.state_target) > 0.1:
           
            # CONSTRUCT SMPC CONSTRAINTS
        

            '''
            add NMPC and SMPC setup code below
            '''





            '''
            add SMPC solving code below
            '''



    
            '''
            add code above
            '''

            


            # UPDATE ROBOT STATE
            # SMPC
            self.robot_smpc.B_matrix = self.update_dynamics(self.state_init_smpc, self.dynamics_function)
            #print(control_results_smpc[:, 0])
            # PUBLISH VELOCITY TO /husky_velocity_controller/cmd_vel_unstamped
            vel_msg = Twist()
            vel_msg.linear.x = float(control_results_smpc[:, 0][0])
            vel_msg.angular.z = float(control_results_smpc[:, 0][1])
            self.velocity_publisher.publish(vel_msg)

            
            self.state_init_smpc = self.state_init_smpc + self.robot_smpc.B_matrix@control_results_smpc[:, 0] # 3x1 ca.DM array for next_state

            # check if location constraints are satisfied or not

            '''
            mask = ca.DM([[1, 0, 0], 
                  [0, 1, 0]]) 
            if ca.norm_2(mask@self.state_init_smpc - ca.DM([10, 0])) < self.robot_smpc.safety_radius:
                print("smpc")
                print(self.mpc_iter)
                print(ca.norm_2(mask@self.state_init_smpc - ca.DM([10, 0])))
                print("  ")
            '''





            '''
            add NMPC solving code below, and update initial guess for next iteration
            '''
            


            
            

            self.mpc_iter += 1

        else:
            print("done")




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
