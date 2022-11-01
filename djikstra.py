import numpy as np 
import matplotlib.pyplot as plt
import random as rd

def djikstra(qinit, qgoal, edges, x_array, y_array):

	N = len(x_array)

	# Check if given graph is symmetric:

	j = 0

	for i in range(len(x_array)):

		edge = edges[i]

		for connection in edge:

			if i not in edges[connection]:

				j += 1

	if j != 0:

		print("asymmetric")

	# Ignore symmetry issue for now!

	# Move on to djikstra:

	shortest_path = dict()
	previous_nodes = dict()
	unvisited_nodes = list(np.arange(0, N))

	for i in range(N):

		shortest_path[i] = float("Inf")

	shortest_path[qinit] = 0 

	while unvisited_nodes:

		current_min_node = None

		for node in unvisited_nodes:

			if current_min_node == None:

				current_min_node = node

			elif shortest_path[node] < shortest_path[current_min_node]:

				current_min_node = node

		neighbors = edges[current_min_node]

		for neighbor in neighbors:

			tentative_value = shortest_path[current_min_node] +  np.sqrt((x_array[current_min_node] - x_array[neighbor])**2 + (y_array[current_min_node] - y_array[neighbor])**2)

			if tentative_value < shortest_path[neighbor]:

				shortest_path[neighbor] = tentative_value

				previous_nodes[neighbor] = current_min_node

		unvisited_nodes.remove(current_min_node)

	print(qinit, qgoal, previous_nodes)

	return 0