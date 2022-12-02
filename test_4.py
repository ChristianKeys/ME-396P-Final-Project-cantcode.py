# Test 2 is concerned with validating our implementations of PRM, RRT, and Dijkstra.
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
from remove_nodes import remove_nodes
from restore_nodes import restore_nodes
from make_video import make_video

# Dynamic visualizer, leading into Test 2 material:

from visualize_dynamic_planner import visualize_dynamic_planner

if __name__ == "__main__":

	CURR_DIR = os.path.dirname(os.path.realpath(__file__))
	for filename in glob.glob(CURR_DIR + "/PRM_Images/*.png"):
		 os.remove(filename)

	converged = False
	static_obstacles = [[1, 1, 2.5, 2.5], [4, 7, 1, 1], [8, 3, 1, 5]]
	dynamic_obstacles = [[5, 7, 0.5, 1], [1, 9, 1, 1], [4, 4, 1, 1], [7, 5, 1, 1], [5, 0, 1, 1], [3, 7, 1, 1]]

	graph_prm, x_array_prm, y_array_prm = prm(static_obstacles, N=2000, k=10)
	qinit, qgoal = pick_endpoints((0.5, 0.5), (9.5, 9.5), x_array_prm, y_array_prm) 
	path_prm, total_cost = djikstra(qinit, qgoal, graph_prm, x_array_prm, y_array_prm)

	speed = 0.25/2
	t = 0
	dt = 1
	i = 0
	speed1_obs = 0.05/2
	speed2_obs = 0.05/2
	speed3_obs = 0.05/2
	speed4_obs = 0.05/2

	x_robot_len = 0.5
	y_robot_len = 0.5
	x_trajectory = [0.5]
	y_trajectory = [0.5]

	while not converged:

		fig, ax = plt.subplots(1,1)
		plt.cla()

		for d_obs in dynamic_obstacles:
			static_obstacles.append(d_obs)

		remove_nodes(graph_prm, x_array_prm, y_array_prm, dynamic_obstacles)

		x_cur = x_trajectory[i]
		y_cur = y_trajectory[i]

		qinit, qgoal = pick_endpoints((x_cur, y_cur), (9.5, 9.5), x_array_prm, y_array_prm)
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
		#qinit = (x_array[0]+dx, y_array[0]+dy)
		x_trajectory.append(x_cur)
		y_trajectory.append(y_cur)

		# Move dynamic obstacle:

		dynamic_obstacles[0][0] += speed1_obs
		dynamic_obstacles[1][1] -= speed2_obs
		dynamic_obstacles[2][0] += speed3_obs
		dynamic_obstacles[3][0] -= speed4_obs
		dynamic_obstacles[4][0] -= speed4_obs
		dynamic_obstacles[5][1] -= speed4_obs


		if dynamic_obstacles[0][0] + dynamic_obstacles[0][2] + speed1_obs > 8 or dynamic_obstacles[0][0] + speed1_obs < 5:
			speed1_obs *= -1

		if dynamic_obstacles[1][1] + dynamic_obstacles[1][3] + speed2_obs < 1:
			speed2_obs *= -1

		plot_obstacles(static_obstacles, dynamic_obstacles, ax)
		plt.plot(x_trajectory, y_trajectory, color="b", label="True Path", lw=3.0)
		plot_path(path, x_array_prm, y_array_prm, "g", ax, legend=False)
		plot_path(path_prm, x_array_prm, y_array_prm, "r", ax)

		step = '{:03d}'.format(i)

		plt.savefig("PRM_Images/Step" + step)
		plt.close()
		print(i)
		ax.patches.pop()
		i += 1

		if np.sqrt((x_cur-9.5)**2 + (y_cur-9.5)**2) < 0.5:
			print("Script terminated!")
			break 

		for j in range(len(dynamic_obstacles)):
			static_obstacles.pop()

		restore_nodes(graph_prm, x_array_prm, y_array_prm)

	#make_video()