import numpy as np 
import matplotlib.pyplot as plt
import random as rd

def restore_nodes(edges, x_array, y_array):

	for i in range(len(x_array)):

		edge = edges[i]

		for key in edge:

			edge[key] = np.sqrt((x_array[i] - x_array[key])**2 + (y_array[i] - y_array[key])**2)

	return edges