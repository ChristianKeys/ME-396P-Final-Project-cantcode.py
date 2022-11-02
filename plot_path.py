import numpy as np 
import matplotlib.pyplot as plt

def plot_path(path, x_array, y_array, obstacles, fig, ax):

	# ax = ax[1]
	plt.sca(ax)

	N = len(x_array)

	for i in range(len(path)-1):

		k1 = path[i]
		k2 = path[i+1]

		plt.plot([x_array[k1], x_array[k2]], [y_array[k1], y_array[k2]], color="g", alpha=1.0, zorder=2, lw=3.0)

	for obstacle in obstacles:

		if obstacle[4] == "d": 

			color = "green"
			alpha = 0.2

		else:

			color = "blue"
			alpha = 1.0

		ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3],
		edgecolor = "black", facecolor = color, alpha=alpha))

	plt.scatter(x_array, y_array, color="red", s = 25, zorder=1, alpha=1.0, linewidth=2.0)
	plt.tick_params(left=False,bottom=False)
	plt.xticks(color='w')
	plt.yticks(color='w')
	ax.set_aspect(1)