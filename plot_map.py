import numpy as np 
import matplotlib.pyplot as plt

def plot_map(edges, x_array, y_array, obstacles, fig, ax):

	# ax = ax[0]
	plt.sca(ax)

	N = len(x_array)

	for j in range(N):

		edge = edges[j]

		for idx in edge:

			if edge[idx] == float("Inf"):

				plt.plot([x_array[j], x_array[idx]], [y_array[j], y_array[idx]], 
				color="k", alpha=0.2, zorder=1, linestyle='dashed')

			else:

				plt.plot([x_array[j], x_array[idx]], [y_array[j], y_array[idx]],
				color="k", alpha=0.2, zorder=1)

	for obstacle in obstacles:

		if obstacle[4] == "d": 

			color = "green"
			alpha = 0.2

		else:

			color = "blue"
			alpha = 1.0

		ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3],
		edgecolor = "black", facecolor = color, alpha=alpha))

	plt.scatter(x_array, y_array, color="red", s = 25, zorder=2)
	plt.tick_params(left=False,bottom=False)
	plt.xticks(color='w')
	plt.yticks(color='w')
	ax.set_aspect(1)

	return fig, ax