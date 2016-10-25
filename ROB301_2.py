import math
import numpy as np
import matplotlib.pyplot as pyplot

# Global Variables
L = 0.1
h = 0.01
k_alpha = 3
k_beta = -1
k_rho = 1
tolerance = 0.01

def pose_to_pose(init_pose, desired_pose):
	''' Takes the initial and desired pose, and calculates the path taken.
	Returns an array of the positions at some given time step
	'''

	# Array of the points at any given time
	x = [init_pose[0]]
	y = [init_pose[1]]
	theta = [init_pose[2]]

	# First iteration
	curr_pose = init_pose
	polar = get_polar_coordinates(curr_pose, desired_pose)
	direction = get_direction(polar)
	# Loop until destination is reached
	while polar[0] > tolerance:
		linear_control = polar_to_linear(polar, direction)
		curr_pose = bicycle_EOM(linear_control, curr_pose)
		x.append(curr_pose[0])
		y.append(curr_pose[1])
		theta.append(curr_pose[2])
		polar = get_polar_coordinates(curr_pose, desired_pose)

	return [x, y, theta]

def bicycle_EOM(linear_control, curr_pose):
	'''
	Takes some input linear control (of v, gamma), and applies the kinematic model
	for 1 time step and returns the new pose
	'''
	v = linear_control[0]
	gamma = linear_control[1]

	theta = curr_pose[2]
	x_dot = v * math.cos(theta)
	y_dot = v * math.sin(theta)
	theta_dot = (v/L) * math.tan(gamma)

	curr_pose_array = np.array(curr_pose)
	delta = np.array([x_dot * h, y_dot * h, theta_dot * h])
	new_pose = curr_pose_array + delta
	return new_pose.tolist()

def get_direction(polar):
	# Called at the beginning to determine if the vehicle is moving forward/back
	if abs(polar[1]) < math.pi/2:
		return 1 # Forward
	else:
		return -1 # Backwards

def get_polar_coordinates(curr_pose, desired_pose):
	'''
	Converts the current_pose to polar coordinates using the desired_pose
	'''
	del_x = curr_pose[0] - desired_pose[0]
	del_y = curr_pose[1] - desired_pose[1]

	rho = math.sqrt(del_x**2 + del_y**2)
	alpha = math.atan(del_y/del_x) - curr_pose[2]
	beta = -curr_pose[2] - alpha
	
	if alpha > 0:
		alpha = min(alpha, math.pi/2)
	else:
		alpha = max(alpha, math.pi/2)
	return [rho, alpha, beta]

def polar_to_linear(polar, direction):
	'''
	Converts the polar coordinates to the linear control policy, using
	the defined proportionality constants
	'''
	v = k_rho * polar[0]
	gamma = k_alpha * polar[1] + k_beta * polar[2]
	if direction > 0: # Forward
		return [v, gamma]
	return [-v, -gamma] # Backwards


if __name__ == "__main__":
	# Constants
	start1 = [-1.5, -1.25, 0]
	start2 = [1.3, -1.4, 0.5]
	start3 = [0.5, 1.23, math.pi/2]
	start4 = [-1.4, 0.5, -0.5]
	start5 = [-1, 1, -0.1]

	desired_x = 0
	desired_y = 0
	desired_theta = math.pi/2
	desired_pose = [desired_x, desired_y, desired_theta]

	# Find the solutions
	coord1 = pose_to_pose(start1, desired_pose)
	print(1)
	print(coord1[2][-1])
	coord2 = pose_to_pose(start2, desired_pose)
	print(2)
	print(coord2[2][-1])
	coord3 = pose_to_pose(start3, desired_pose)
	print(3)
	print(coord3[2][-1])
	coord4 = pose_to_pose(start4, desired_pose)
	print(4)
	print(coord4[2][-1])
	coord5 = pose_to_pose(start5, desired_pose)
	print(5)
	print(coord5[2][-1])

	# Plot the graphs
	pyplot.plot(coord1[0], coord1[1], 'k')
	pyplot.plot(coord2[0], coord2[1], 'b')
	pyplot.plot(coord3[0], coord3[1], 'y')
	pyplot.plot(coord4[0], coord4[1], 'c')
	pyplot.plot(coord5[0], coord5[1], 'r')
	pyplot.axis([-2, 2, -2, 2])
	pyplot.xlabel("x")
	pyplot.ylabel("y")
	pyplot.title("2a) pose-to-pose for bicycle model")
	pyplot.show()
