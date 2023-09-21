import numpy as np
import casadi as ca


class Robot:
	def __init__(self, x, y, omega, safety_radius, linear_v_min, linear_v_max, angular_v_min,  angular_v_max, dt):
		
		self.safety_radius = safety_radius

		self.linear_v_min = linear_v_min 
		self.linear_v_max = linear_v_max
		self.angular_v_min = angular_v_min
		self.angular_v_max = angular_v_max

		self.location = np.array([
							[x],
							[y],
							[omega]
						  ])

		self.x_dot = np.array([
							[0],
							[0],
							[0]
						  ])

		self.wheel_speed = np.array([
										[0],
										[0]
									])

		self.b = 0.25
		self.r = 0.05

		self.car_dims = np.array([
										[-self.b, -self.b, 1],
										[0 		, -self.b, 1],
										[ self.b,  		0, 1],
										[ 0, 	   self.b, 1],
										[ -self.b, self.b, 1]
									])
		
		self.B_matrix = np.array([
						[np.cos(self.location[2, 0])*dt,  0],
						[np.sin(self.location[2, 0])*dt,  0],
						[0					 , dt]
					])
		



	# Computes wheel velocities with linear and angular velocities
	def inverse_kinematics(self):
		ikine_mat = np.array([
							[1/self.r, 0, self.b/self.r],
							[1/self.r, 0, -self.b/self.r]
							])

		return ikine_mat@self.x_dot
	

	# Step 1 (after MPC): Update Robot Volocity
	def set_robot_velocity(self, linear_velocity, angular_velocity):
		self.x_dot = np.array([
										[linear_velocity],
										[0],
										[angular_velocity]
									])
		# # self.wheel_speed = self.inverse_kinematics()






	# Step 2 (after MPC): Move Robot
	def update(self, dt):
		# # self.wheel_speed[self.wheel_speed>2] = 2;
		# # self.wheel_speed[self.wheel_speed<-2] = -2;

		# recompute linear and angular velocities using adjusted wheel speeds
		# # self.x_dot = self.forward_kinematics() 
		
		self.update_state(dt)
		# self.wheel_speed = self.inverse_kinematics()


	def forward_kinematics(self):
		kine_mat = np.array([
							[self.r/2  		  , self.r/2],
							[0 		 		  ,	0],
							[self.r/(2*self.b), -self.r/(2*self.b)]
							])

		return kine_mat@self.wheel_speed


	def update_state(self, dt):
		A = np.array([
						[1, 0, 0],
						[0, 1, 0],
						[0, 0, 1]
					])
		B = np.array([
						[np.cos(self.location[2, 0])*dt,  0],
						[np.sin(self.location[2, 0])*dt,  0],
						[0					 , dt]
					])
		

		vel = np.array([
							[self.x_dot[0, 0]],
							[self.x_dot[2, 0]]
						])

		self.location = A@self.location + B@vel


	def get_state(self):
		return self.location, self.x_dot

	

