import numpy as np 
import matplotlib.pyplot as plt
import random as rd
import heapq as hp
from prm import prm
from plot_path import plot_path
from visualize_dynamic_planner import visualize_dynamic_planner 

def djikstra(qinit, qgoal, graph, x_array, y_array, astar=False):  #If you want to use Astar, you must set astar = true

	def heuristic(q1, q2):

		h = np.sqrt((x_array[q1] - x_array[q2])**2 + (y_array[q1] - y_array[q2])**2)
		# h = (x_array[q1] - x_array[q2])**2 + (y_array[q1] - y_array[q2])**2

		return h

	# Performing Djikstra's, minimizing the distances:

	N = len(graph) # How many nodes I have

	predecessor = dict() # Dictionary that keeps of track of connectivity. 
	shortest_distance = dict() # Dictionary that keeps track of shortest distances.
	shortest_distance_heuristic = dict()
	unvisited_nodes = list(np.arange(0, N)) # List of unvisited nodes.

	# All edges have to be infinity. 

	for i in unvisited_nodes:
		shortest_distance[i] = float("Inf")
		shortest_distance_heuristic[i] = float("Inf")

	# Set initial point to 0.

	shortest_distance[qinit] = 0
	h = 0
	if astar:
		h = heuristic(qinit, qgoal)
	shortest_distance[qinit] = 0
	shortest_distance_heuristic[qinit] = h

	# While there are univisted nodes:

	while unvisited_nodes:

		# Node that I am currently visiting:

		minNode = None

		# Choose which node to visit:

		for node in unvisited_nodes:

			if astar:

				if minNode is None:
					minNode = node 

				elif shortest_distance_heuristic[node] < shortest_distance_heuristic[minNode]:
					minNode = node

			else:

				if minNode is None:
					minNode = node 

				elif shortest_distance[node] < shortest_distance[minNode]:
					minNode = node

		# Compare current minimum distance to new minimum distance and update if necessary:

		for childNode, weight in graph[minNode].items():

			if weight + shortest_distance[minNode] < shortest_distance[childNode]:

				shortest_distance[childNode] = weight + shortest_distance[minNode]
				shortest_distance_heuristic[childNode] = weight + shortest_distance[minNode] + heuristic(childNode, qgoal)
				predecessor[childNode] = minNode

		# Remove node that I just visited:

		if minNode == qgoal:
			break

		unvisited_nodes.remove(minNode)

	# Reconstructing the path:

	currentNode = qgoal
	path = []
	total_cost = 0

	while currentNode != qinit:

		try:

			total_cost += graph[currentNode][predecessor[currentNode]]
			path.insert(0, currentNode)
			currentNode = predecessor[currentNode]

		except KeyError:

			print("Unreacheable Path")
			break

	path.insert(0, qinit)

	return path, total_cost
