#This test shows our RRT is formed

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

def animate_rrt(static_obstacles, dynamic_obstacles, graph, x_array, y_array, ax):

	plt.sca(ax)

	N = len(x_array)

	j = 0

	for i in range(N):

		plt.scatter(x_array[i], y_array[i], color="red", s = 25, zorder=2)
		plt.savefig("RRT_Prog/Step" + str(i))

		if i == N-1:
			break

		edge = graph[i]

		for idx in edge:

			if idx != "parent":

				if edge[idx] == float("Inf"):

					plt.plot([x_array[i], x_array[idx]], [y_array[i], y_array[idx]], 
					color="k", alpha=0.2, zorder=1, linestyle='dashed')

				else:
					plt.plot([x_array[i], x_array[idx]], [y_array[i], y_array[idx]],
					color="k", alpha=0.2, zorder=1)

				plt.savefig("RRT_Prog/Step" + str(i+j))
				j += 1

if __name__ == "__main__":

	CURR_DIR = os.path.dirname(os.path.realpath(__file__))
	for filename in glob.glob(CURR_DIR + "/RRT_Prog/*.png"):
		 os.remove(filename)

	static_obstacles = [[1, 1, 2.5, 2.5],
					   [4, 7, 1, 1],
					   [8, 3, 1, 5]]

	dynamic_obstacles = [[5, 7, 3, 1, "d"]]

	# Test RRT:

	graph, x_array, y_array, path = rrt(1000, static_obstacles, (0.5, 0.5), (9.5, 9.5), delta=0.5, eps=0.25)

	# Plotting RRT:

	fig1, ax1 = plt.subplots(1,1)

	plot_obstacles(static_obstacles, dynamic_obstacles, ax1)
	animate_rrt(static_obstacles, dynamic_obstacles, graph, x_array, y_array, ax1)
	make_video("/RRT_Prog/*.png")
