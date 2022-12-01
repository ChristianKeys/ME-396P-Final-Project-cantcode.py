# Incomplete test attempting to change 2-D simulation into 3-D
# It also makes extensive use our own plotting and animation functions to visualize the results.

# Python standard libraries:

import matplotlib.pyplot as plt
import random as rd
import time

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

# Dynamic visualizer, leading into Test 2 material:

from visualize_dynamic_planner import visualize_dynamic_planner

if __name__ == "__main__":

	# Define obstacles:

	static_obstacles = [[1, 1, 2, 2.5, 2.5, 3],
			   [4, 7, 6, 1, 1, 3],
			   [8, 3, 4, 1, 5, 4]]

	dynamic_obstacles = [[5, 7, 3, 1, 2, 6, "d"]]

	# Test RRT:

	print("Running RRT...")
	begin = time.time()

	graph, x_array, y_array, z_array, path = rrt(500, static_obstacles, (0.5, 0.5, 0.5), (9.5, 9.5, 9.5), delta = 0.5, eps = 0.25)

	# Plotting RRT:

	fig1, ax1 = plt.subplots(1,1)
	plot_obstacles(static_obstacles, dynamic_obstacles, ax1)
	plot_graph(graph, x_array, y_array, z_array, ax1)
	plot_path(path, x_array, y_array, z_array, ax1)
	plt.title("RRT")

	end = time.time()
	if path:
		print("Done! Time taken: " + "{:.2f}".format(end-begin) + "s. Path found!") 
	else:
		print("Done! Time taken: " + "{:.2f}".format(end-begin) + "s. No path found!") 

	# Test PRM:

	graph, x_array, y_array = prm(static_obstacles, N = 500, k = 6)
	qinit, qgoal = pick_endpoints((0.5, 0.5, 0.5), (9.5, 9.5, 9.5), x_array, y_array, z_array) 
	path, total_cost = djikstra(qinit, qgoal, graph, x_array, y_array, z_array)

	# Plotting PRM:

	print("Running PRM...")
	begin = time.time()

	fig2, ax2 = plt.subplots(1,1)
	plot_obstacles(static_obstacles, dynamic_obstacles, ax2)
	plot_graph(graph, x_array, y_array, z_array, ax2)
	plot_path(path, x_array, y_array, z_array, ax2)
	plt.title("PRM")

	end = time.time()
	print("Done! Time taken: " + "{:.2f}".format(end-begin) + "s. Path found!") 

	# Test dynamic visualizer using PRM result:

	print("Running animation...")
	begin = time.time()

	fig3, ax3 = plt.subplots(1,1)
	plot_obstacles(static_obstacles, dynamic_obstacles, ax3)
	plot_path(path, x_array, y_array, z_array, ax3)

	# Testing dynamic visualization using PRM result:

	visualize_dynamic_planner(qinit, qgoal, path, x_array, y_array, z_array, fig3, ax3)
	end = time.time()
	print("Done! Time taken: " + "{:.2f}".format(end-begin) + "s. Results being displayed...") 
	plt.show()
	print("Script terminated!")
