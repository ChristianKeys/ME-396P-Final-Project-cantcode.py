import numpy as np 
import matplotlib.pyplot as plt

def plot_map(edges, x_array, y_array):

	fig, ax = plt.subplots(1,2)

	plt.sca(ax[0])

	N = len(x_array)

	for j in range(N):

		array = edges[j]

		for k in array:

			plt.plot([x_array[j], x_array[k]], [y_array[j], y_array[k]], color="k", alpha=0.5, zorder=0)

	ax[0].add_patch(plt.Rectangle((1, 1), 2.5, 2.5, edgecolor = "black", facecolor = "blue"))
	ax[0].add_patch(plt.Rectangle((5, 7), 1, 1, edgecolor = "black", facecolor = "blue"))
	ax[0].add_patch(plt.Rectangle((8, 3), 1, 5, edgecolor = "black", facecolor = "blue"))
	ax[0].add_patch(plt.Rectangle((6, 7), 2, 1, edgecolor = "black", facecolor = "green", alpha=0.5))

	plt.scatter(x_array, y_array, color="red", s = 25, zorder=1)
	plt.show()