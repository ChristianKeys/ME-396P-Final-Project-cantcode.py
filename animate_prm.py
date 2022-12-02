# This file has a simulation which shows how PRM is formed.

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

def animate_prm(static_obstacles, dynamic_obstacles, graph, x_array, y_array, ax):

	plt.sca(ax)

	N = len(x_array)

	for i in range(N):

		plt.scatter(x_array[i], y_array[i], color="red", s = 25, zorder=2)

		if i%10 == 0:
			plt.savefig("PRM_Prog/Step" + str(i))
			print(i)

	for j in range(N):

		edge = graph[j]

		for idx in edge:

			if idx != "parent":

				if edge[idx] == float("Inf"):

					plt.plot([x_array[j], x_array[idx]], [y_array[j], y_array[idx]], 
					color="k", alpha=0.2, zorder=1, linestyle='dashed')

				else:
					plt.plot([x_array[j], x_array[idx]], [y_array[j], y_array[idx]],
					color="k", alpha=0.2, zorder=1)

			if (j)%10 == 0:
				plt.savefig("PRM_Prog/Step" + str(i+j))
				print(j)

	return

if __name__ == "__main__":

	CURR_DIR = os.path.dirname(os.path.realpath(__file__))
	for filename in glob.glob(CURR_DIR + "/PRM_Prog/*.png"):
		 os.remove(filename)

	static_obstacles = [[1, 1, 2.5, 2.5],
					   [4, 7, 1, 1],
					   [8, 3, 1, 5]]

	dynamic_obstacles = [[5, 7, 3, 1, "d"]]

	# Test PRM:

	graph, x_array, y_array = prm(static_obstacles, N=250, k=3)

	# Plotting PRM:

	fig1, ax1 = plt.subplots(1,1)
	plot_obstacles(static_obstacles, dynamic_obstacles, ax1)
	animate_prm(static_obstacles, dynamic_obstacles, graph, x_array, y_array, ax1)
	make_video("/PRM_Prog/*.png", "prm_prog")
