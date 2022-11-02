import matplotlib.pyplot as plt
import random as rd
from prm import prm
from plot_map import plot_map
from plot_path import plot_path
from djikstra import djikstra
from visualize_dynamic_planner import visualize_dynamic_planner
from remove_nodes import remove_nodes

if __name__ == "__main__":

	edges, x_array, y_array = prm(N=500, k=6)

	obstacles = [[1, 1, 2.5, 2.5, "s"],
				[4, 7, 1, 1, "s"],
				[8, 3, 1, 5, "s"],
				[5, 7, 3, 1, "d"]] 

	dynamic_obstacles = [[5, 7, 3, 1]]

	edges = remove_nodes(edges, x_array, y_array, dynamic_obstacles)

	fig, ax = plt.subplots(1,1)

	plot_map(edges, x_array, y_array, obstacles, fig, ax)

	N = len(x_array)

	# add_endpoints()

	path = djikstra(qinit, qfinal, edges, x_array, y_array)

	plot_path(path, x_array, y_array, obstacles, fig, ax)

	plt.show()

