import matplotlib.pyplot as plt
import random as rd
from rrt import rrt
from prm import prm
from pick_endpoints import pick_endpoints
from plot_map import plot_map
from plot_path import plot_path
from djikstra import djikstra
from remove_nodes import remove_nodes
from visualize_dynamic_planner import visualize_dynamic_planner

if __name__ == "__main__":

	static_obstacles = [[1, 1, 2.5, 2.5],
					   [4, 7, 1, 1],
					   [8, 3, 1, 5]]

	dynamic_obstacles = [[5, 7, 3, 1, "d"]]

	#fig1, ax = plt.subplots(1,1)

	#edges, x_array, y_array = prm(static_obstacles, N=500, k=6) 
	
	#plot_map(edges, x_array, y_array, static_obstacles, dynamic_obstacles, fig1, ax) 

	fig1, ax = plt.subplots(1,1)
	#edges, x_array, y_array = prm(static_obstacles, N=500, k=6) 
	edges, x_array, y_array = rrt(500, static_obstacles, (5, 5), delta=0.5)

	#print(edges)

	plot_map(edges, x_array, y_array, static_obstacles, dynamic_obstacles, fig1, ax)

	#path, total_cost = djikstra(0, -1, edges, x_array, y_array)

	fig2, ax = plt.subplots(1,1)

	#qinit, qgoal = pick_endpoints((2,0), (10,10), x_array, y_array)

	#print(qinit, qgoal)

	path, total_cost = djikstra(0, len(x_array)-1, edges, x_array, y_array)

	plot_path(path, x_array, y_array, static_obstacles, dynamic_obstacles, fig2, ax, "g")

	#qinit, qfinal = pick_endpoints((2,0), (10,10), x_array, y_array)

	'''

	plot_map(edges, x_array, y_array, static_obstacles, dynamic_obstacles, fig1, ax)

	fig2, ax = plt.subplots(1,1)

	path1, total_cost_1 = djikstra(qinit, qfinal, edges, x_array, y_array)

	edges = remove_nodes(edges, x_array, y_array, dynamic_obstacles)

	path2, total_cost_2 = djikstra(qinit, qfinal, edges, x_array, y_array)

	plot_path(path1, x_array, y_array, static_obstacles, dynamic_obstacles, fig2, ax, "g")
	plot_path(path2, x_array, y_array, static_obstacles, dynamic_obstacles, fig2, ax, "k")

	#print("Djikstra: ", total_cost_1)
	#print("AStar: ", total_cost_2)

	fig3, ax = plt.subplots(1,1)
	#fig4, ax = plt.subplots(1,1)

	visualize_dynamic_planner(qinit, qfinal, path1, static_obstacles, dynamic_obstacles, x_array, y_array, fig3, ax)

	'''

	plt.show()
