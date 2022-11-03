import matplotlib.pyplot as plt
import random as rd
from prm import prm
from plot_map import plot_map
from plot_path import plot_path
from djikstra import djikstra
from astar import astar
from visualize_dynamic_planner import visualize_dynamic_planner

if __name__ == "__main__":

	obstacles = [[1, 1, 2.5, 2.5, "s"],
				[4, 7, 1, 1, "s"],
				[8, 3, 1, 5, "s"],
				[5, 7, 3, 1, "d"]]

	fig1, ax = plt.subplots(1,1)

	edges, x_array, y_array = prm(N=500, k=6) 

	plot_map(edges, x_array, y_array, obstacles, fig1, ax)

	N = len(x_array)

	qinit = rd.randint(0, N-1)
	qfinal = rd.randint(0, N-1)

	while qinit == qfinal:

		qfinal = rd.randint(0, N-1)

	fig2, ax = plt.subplots(1,1)

	path = astar(qinit, qfinal, edges, x_array, y_array)

	plot_path(path, x_array, y_array, obstacles, fig2, ax)

	plt.show()

	'''

	fig3, ax = plt.subplots(1,1)

	visualize_dynamic_planner(qinit, qfinal, path, obstacles, x_array, y_array, fig3, ax)

	'''