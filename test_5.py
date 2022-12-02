# Test 5 is the real life simulation of the UT Campus
# It also makes extensive use our own plotting and animation functions to visualize the results.

# Python standard libraries:

import matplotlib.pyplot as plt
import numpy as np
import random as rd
import time
import cv2
import os
import glob

# Our implementations of common path-planning algorithms:

from rrt import rrt
from prm import prm
from djikstra import djikstra

# Plotting functions:

from plot_obstacles import plot_obstacles
from plot_graph import plot_graph
from plot_path import plot_path

# Auxiliary functions:

from pick_endpoints import pick_endpoints
from remove_nodes import remove_nodes
from restore_nodes import restore_nodes
from make_video import make_video

if __name__ == "__main__":

	# Define obstacles:

	static_obstacles = [[0, 4, 4.5, 1],
					   [3.5, 0, 1, 1.5],
					   [3.5, 2.5, 1, 1.5],
					   [5.5, 0, 2, 1],
					   [5.5, 2, 2, 3],
					   [8.5, 0, 1.5, 5],
					   [0, 8, 4.5, 2],
					   [5.5, 9, 2, 1],
					   [9, 7, 1, 3]]

	# Cars going right: (space them out by 2)

	dynamic_obstacles = []

	for i in range(10):

		car = [2*(i-2), 5.5, 0.5 , 0.25]
		dynamic_obstacles.append(car)

	for i in range(10):

		car = [2*(i-2), 6.5, 0.5 , 0.25]
		dynamic_obstacles.append(car)

	for i in range(7):

		car = [4.875, (i-2), 0.25, 0.5]
		dynamic_obstacles.append(car)

	for i in range(9,13):

		car = [4.875, (i-2), 0.25, 0.5]
		dynamic_obstacles.append(car)

	CURR_DIR = os.path.dirname(os.path.realpath(__file__))
	for filename in glob.glob(CURR_DIR + "/test5/*.png"):
		 os.remove(filename)

	graph_prm, x_array_prm, y_array_prm = prm(static_obstacles, N=3000, k=10, eps=0.25)
	qinit, qgoal = pick_endpoints((0.5, 0.5), (8.5, 9.5), x_array_prm, y_array_prm) 
	path_prm, total_cost = djikstra(qinit, qgoal, graph_prm, x_array_prm, y_array_prm)

	converged = False

	speed = 0.2
	speed_obs1 = 0.05
	x_trajectory = [0.5]
	y_trajectory = [0.5]
	x_robot_len = 0.1
	y_robot_len = 0.1

	i = 0

	while not converged:

		fig, ax = plt.subplots(1,1)
		plt.cla()

		for d_obs in dynamic_obstacles:
			static_obstacles.append(d_obs)

		remove_nodes(graph_prm, x_array_prm, y_array_prm, dynamic_obstacles)

		x_cur = x_trajectory[i]
		y_cur = y_trajectory[i]

		qinit, qgoal = pick_endpoints((x_cur, y_cur), (8.5, 9.5), x_array_prm, y_array_prm)
		path, total_cost = djikstra(qinit, qgoal, graph_prm, x_array_prm, y_array_prm, astar=True)

		if not path:
			print("Path not found!")
			break

		# Calculate direction in which to march in time:

		x0 = (x_array_prm[path[0]], y_array_prm[path[0]])
		x1 = (x_array_prm[path[1]], y_array_prm[path[1]])

		theta = np.arctan2(x1[1] - x0[1], x1[0] - x0[0])
		dx = speed*np.cos(theta)
		dy = speed*np.sin(theta)
		ax.add_patch(plt.Rectangle((x_cur - x_robot_len/2, y_cur - y_robot_len/2), x_robot_len, y_robot_len, 
		edgecolor = "black", facecolor = "red", zorder=1))
		x_cur += dx
		y_cur += dy
		x_trajectory.append(x_cur)
		y_trajectory.append(y_cur)

		# Move dynamic obstacle:

		for c1 in range(20):
			if c1 < 10:
				dynamic_obstacles[c1][0] += speed_obs1
			else:
				dynamic_obstacles[c1][0] -= speed_obs1

		plot_obstacles(static_obstacles, dynamic_obstacles, ax)
		plt.plot(x_trajectory[:-1], y_trajectory[:-1], color="g", label="True Path", lw=1.0)
		#plt.plot(x_trajectory, y_trajectory, color="g", label="True Path", lw=1.0)
		#plt.plot(x_trajectory, y_trajectory, color="r", label="True Path", lw=1.0)
		#plot_path(path, x_array_prm, y_array_prm, "g", ax, legend=False)
		#plot_path(path_prm, x_array_prm, y_array_prm, "r", ax)

		step = '{:03d}'.format(i)

		plt.savefig("test5/Step" + step)
		plt.close()
		print(i)
		ax.patches.pop()
		i += 1

		if np.sqrt((x_cur-8.5)**2 + (y_cur-9.5)**2) < 0.2:
			print("Script terminated!")
			break 

		for j in range(len(dynamic_obstacles)):
			static_obstacles.pop()

		restore_nodes(graph_prm, x_array_prm, y_array_prm)

	make_video("/test5/*.png", "test5")
