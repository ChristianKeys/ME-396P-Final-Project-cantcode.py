import numpy as np 
import matplotlib.pyplot as plt
import random as rd

def remove_nodes(edges, x_array, y_array, obstacles):

	def is_valid_node(p, obstacles):
		"""
		Determines whether a point is inside existing static obstacles.

			Parameters:

				p (float 2-tuple): x and y coordinates of the point.
				obstacles (n x 5 list of lists): obstacles' properties.

			Returns:

				True or False (bool).

		"""

		eps = 0.35

		for obstacle in obstacles:

			ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[2] + eps)
			ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[3] + eps)

			if p[0] > ob_x[0] and p[0] < ob_x[1] and p[1] > ob_y[0] and p[1] < ob_y[1]:

				return False

		return True

	for i in range(len(x_array)):

		p = (x_array[i], y_array[i])

		if not is_valid_node(p, obstacles):

			edge = edges[i]

			for key in edge:

				edge[key] = float("Inf")

	return edges





