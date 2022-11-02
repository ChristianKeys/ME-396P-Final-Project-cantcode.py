import numpy as np 
import matplotlib.pyplot as plt
import random as rd
import heapq as hp
from prm import prm
from plot_map import plot_map
from plot_path import plot_path
from visualize_dynamic_planner import visualize_dynamic_planner 

def djikstra(qinit, qgoal, edges, x_array, y_array):

	# Performing Djikstra's, minimizing the distances:

	N = len(x_array) # How many nodes I have


	predecessor = dict() # Dictionary that keeps of track of connectivity. 
	shortest_distance = dict() # Dictionary that keeps track of shortest distances.
	unvisited_nodes = list(np.arange(0, N)) # List of unvisited nodes.

	# All edges have to be infinity. 

	for i in unvisited_nodes:
		shortest_distance[i] = float("Inf")

	# Set initial point to 0.

	shortest_distance[qinit] = 0

	# While there are univisted nodes:

	while unvisited_nodes:

		# Node that I am currently visiting:

		minNode = None

		# Choose which node to visit:

		for node in unvisited_nodes:

			if minNode is None:
				minNode = node 

			elif shortest_distance[node] < shortest_distance[minNode]:
				minNode = node

		# Compare current minimum distance to new minimum distance and update if necessary:

		for childNode, weight in edges[minNode].items():

			if weight + shortest_distance[minNode] < shortest_distance[childNode]:

				shortest_distance[childNode] = weight + shortest_distance[minNode]
				predecessor[childNode] = minNode

		# Remove node that I just visited:

		unvisited_nodes.remove(minNode)

	# Reconstructing the path:

	currentNode = qgoal
	path = []

	while currentNode != qinit:

		try:

			path.insert(0, currentNode)
			currentNode = predecessor[currentNode]

		except KeyError:

			print("Unreacheable Path")
			break

	path.insert(0, qinit)

	return path