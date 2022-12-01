# Test 3 runs dijkstra and astar 
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

	static_obstacles = [[1, 1, 2.5, 2.5],
					   [4, 7, 1, 1],
					   [8, 3, 1, 5]]

	dynamic_obstacles = [[5, 7, 3, 1]]

	# static_obstacles.append(dynamic_obstacles[0]) 

	# Test PRM:

	graph, x_array, y_array, path = rrt(1000, static_obstacles, (0.5, 0.5), (9.5, 9.5), delta=0.5, eps=0.25)
	fig1, ax1 = plt.subplots(1,1)
	plot_graph(graph, x_array, y_array, ax1)
	ax1.set_aspect(1)
	plt.tick_params(left=False,bottom=False)
	plt.xticks(color='w')
	plt.yticks(color='w')
	plt.axis([0, 10, 0, 10])
	plt.show()

	graph, x_array, y_array = prm(static_obstacles, N=2000, k=10)
	qinit, qgoal = pick_endpoints((5, 4), (9.5, 9.5), x_array, y_array)


	begin1 = time.time()
	path1, total_cost1 = djikstra(qinit, qgoal, graph, x_array, y_array)
	end1 = time.time()
	print("Done! Time taken: " + "{:.5f}".format(end1-begin1) + "s. Path found!")


	begin2 = time.time()
	path2, total_cost2 = djikstra(qinit, qgoal, graph, x_array, y_array, astar=True)
	end2 = time.time()
	print("Done! Time taken: " + "{:.5f}".format(end2-begin2) + "s. Path found!")


	print(total_cost1)
	print(total_cost2)

	# Plotting PRM:

	fig1, ax1 = plt.subplots(1,1)
	plot_obstacles(static_obstacles, dynamic_obstacles, ax1)
	plot_graph(graph, x_array, y_array, ax1)
	plot_path(path1, x_array, y_array, "g", ax1)
	plot_path(path2, x_array, y_array, "c", ax1)
	plt.show()
