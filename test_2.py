# Test 2 is concerned with validating our implementations of PRM, RRT, and Dijkstra
# It also makes extensive use our own plotting and animation functions to visualize the results.

# Python standard libraries:

import matplotlib.pyplot as plt
import numpy as np
import time
import glob
import os

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
from make_video import make_video

# Dynamic visualizer, leading into Test 2 material:

from visualize_dynamic_planner import visualize_dynamic_planner

if __name__ == "__main__":

	CURR_DIR = os.path.dirname(os.path.realpath(__file__))
	for filename in glob.glob(CURR_DIR + "/RRT_Images/*.png"):
		 os.remove(filename)

	converged = False
	static_obstacles = [[1, 1, 2.5, 2.5], [4, 7, 1, 1], [8, 3, 1, 5]]
	dynamic_obstacles = [[5, 7, 0.5, 1]]

	graph_prm, x_array_prm, y_array_prm = prm(static_obstacles, N=2000, k=10)
	qinit, qgoal = pick_endpoints((0.5, 0.5), (9.5, 9.5), x_array_prm, y_array_prm) 
	path_prm, total_cost = djikstra(qinit, qgoal, graph_prm, x_array_prm, y_array_prm)

	speed = 0.25
	t = 0
	dt = 1
	i = 0
	speed_obs = 0.05
	qinit = (0.5, 0.5)
	qgoal = (9.5, 9.5)

	x_robot_len = 0.5
	y_robot_len = 0.5
	x_trajectory = [qinit[0]]
	y_trajectory = [qinit[1]]

	while not converged:

		fig, ax = plt.subplots(1,1)
		plt.cla()
		static_obstacles.append(dynamic_obstacles[0])

		graph, x_array, y_array, path = rrt(2000, static_obstacles, qinit, qgoal, delta=0.5, eps=0.25)

		if not path:
			print("Path not found!")
			break

		# Calculate diretion in which to march in time:

		x0 = (x_array[path[0]], y_array[path[0]])
		x1 = (x_array[path[1]], y_array[path[1]])

		theta = np.arctan2(x1[1] - x0[1], x1[0] - x0[0])
		dx = speed*np.cos(theta)
		dy = speed*np.sin(theta)
		ax.add_patch(plt.Rectangle((qinit[0] - x_robot_len/2, qinit[1] - y_robot_len/2), x_robot_len, y_robot_len, 
		edgecolor = "black", facecolor = "red", zorder=1))
		qinit = (x_array[0]+dx, y_array[0]+dy)
		x_trajectory.append(qinit[0])
		y_trajectory.append(qinit[1])

		# Move dynamic obstacle:

		dynamic_obstacles[0][0] += speed_obs

		if dynamic_obstacles[0][0] + dynamic_obstacles[0][2] + speed_obs > 8 or dynamic_obstacles[0][0] + speed_obs < 5:
			speed_obs *= -1

		plot_obstacles(static_obstacles, dynamic_obstacles, ax)
		plt.plot(x_trajectory, y_trajectory, color="b", label="True Path" ,lw=3.0)
		plot_path(path, x_array, y_array, "g", ax, legend=False)
		plot_path(path_prm, x_array_prm, y_array_prm, "r", ax)

		step = '{:03d}'.format(i)

		plt.savefig("RRT_Images/Step" + step)
		plt.close()
		print(i)
		ax.patches.pop()
		i += 1

		if np.sqrt((qinit[0]-qgoal[0])**2 + (qinit[1]-qgoal[1])**2) < 0.5:
			print("Script terminated!")
			break 

		static_obstacles.pop()

	make_video("/RRT_Images/*.png")

