from time import time
import time as tm
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
import math
import random


class Obstacle:
	def __init__(self, x, y, radius):
		self.x = x
		self.y = y
		self.radius = radius

class Map:
	def __init__(self, x_init, y_init, x_target, y_target, theta_target, y_center, y_offset, number_obstacles):
		self.x_init = x_init
		self.y_init = y_init
		self.x_target = x_target
		self.y_target = y_target
		self.theta_target = theta_target

		self.y_center = y_center
		self.y_offset = y_offset

		self.y_lower_limit = -ca.inf
		self.y_upper_limit = ca.inf

		self.y_lower_limit = self.y_center-self.y_offset
		self.y_upper_limit = self.y_center+self.y_offset

		self.number_obstacles = number_obstacles
		self.obstacle_locations = np.zeros((3, number_obstacles))

		self.obstacle_list = []
		
	

	def trajectory_y(self, x):
		# return int(700/2 + 200*np.sin(0.5*np.pi*(x + 200)/100))
		return self.y_target


	# straight reference trajectory from initial state to target state
	def construct_reference(self, simulations, increment, horizons):
		reference_states = []
		# 3D point
		for i in range(simulations):
			x = self.x_init + increment*(i)
			y = self.trajectory_y(x)
			reference_states.append([x, y, 0])
		for i in range(horizons):
			reference_states.append([self.x_target, self.y_target, self.theta_target])
				
		for i in range(simulations-1):
			dy = reference_states[i+1][1] - reference_states[i][1]
			dx = reference_states[i+1][0] - reference_states[i][0]
			reference_states[i][2] = math.atan(dy/dx)

		return reference_states
	

	# single target without predefined trajectory
	def construct_single_target_reference(self, simulations, horizons):
		reference_states = []
		# 3D point
		for i in range(simulations + horizons):
			reference_states.append([self.x_target, self.y_target, self.theta_target])

		return reference_states
	

	# GENERATE RECTANGULAR OBSTACLES BASED ON x LOCATION
	def generate_boundary_obstacles(self, x_reference_init, x_reference_end, length_limit, width_limit):
		for i in range(self.number_obstacles):
			x_obs_center = random.randint(x_reference_init, x_reference_end)
			obs_half_length = random.randint(1, length_limit)
			obs_width = random.randint(1, width_limit)
			x_obs_init = x_obs_center - obs_half_length
			x_obs_end = x_obs_center + obs_half_length
			y_obs = self.y_upper_limit - obs_width
			self.obstacle_locations[:,i] = [x_obs_init, x_obs_end, y_obs]



	# GENERATE y STATE SPACE
	def generate_upper_state_space(self, simulations, increment, obstacles):
		for i in range(simulations):
			x = self.x_init + increment*i
			
			for j in range(obstacles):
				if self.obstacle_locations[0][j] <= x <= self.obstacle_locations[1][j] and self.upper_y_constraint[i] > self.obstacle_locations[2][j]:
					self.upper_y_constraint[i] = self.obstacle_locations[2][j]


	# GENERATE CIRCULAR OBSTACLES
	def generate_circular_obstacles(self, x_reference_init, x_reference_end, radius_limit):
		
		obs = Obstacle(15, 0, 2)
		self.obstacle_list.append(obs)
		
		'''
		obs = Obstacle(24, 0, 2)
		self.obstacle_list.append(obs)
		obs = Obstacle(32, 0, 2)
		self.obstacle_list.append(obs)
		'''

		'''
		for i in range(self.number_obstacles):
			obs_x = random.randint(x_reference_init, x_reference_end)
			obs_y = random.randint(self.y_lower_limit, self.y_upper_limit)
			radius = random.randint(1, radius_limit)
			obs = Obstacle(obs_x, obs_y, radius)
			#obs = Obstacle(4, 0, 2)
			
			self.obstacle_list.append(obs)
		'''

		
			
		

	


