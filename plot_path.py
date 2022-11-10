import numpy as np 
import matplotlib.pyplot as plt

def plot_path(path, x_array, y_array, static_obstacles, dynamic_obstacles, fig, ax, color):

	# ax = ax[1]
	plt.sca(ax)

	N = len(x_array)

	plt.scatter(x_array[path[0]], y_array[path[0]], color="goldenrod", marker="s", s = 50, zorder=3, alpha=1.0, label="Start")
	plt.scatter(x_array[path[-1]], y_array[path[-1]], color="blueviolet", marker="s", s = 50, zorder=3, alpha=1.0, label="Goal")

	#ax.annotate("Start", xy=(x_array[path[0]], y_array[path[0]]), xytext=(11,5), 
	#	arrowprops=dict(facecolor="black", width=1, headwidth=5, headlength=5, shrink=0.05))

	for i in range(len(path)-1):

		k1 = path[i]
		k2 = path[i+1]

		plt.plot([x_array[k1], x_array[k2]], [y_array[k1], y_array[k2]], color=color, alpha=1.0, zorder=2, lw=3.0)

	for obstacle in static_obstacles:

		color = "blue"
		alpha = 1.0
		ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3],
		edgecolor = "black", facecolor = color, alpha=alpha))

	for obstacle in dynamic_obstacles:

		color = "green"
		alpha = 0.2
		ax.add_patch(plt.Rectangle((obstacle[0], obstacle[1]), obstacle[2], obstacle[3],
		edgecolor = "black", facecolor = color, alpha=alpha))

	plt.scatter(x_array, y_array, color="red", s = 25, zorder=1, alpha=1.0)
	plt.tick_params(left=False,bottom=False)
	plt.xticks(color='w')
	plt.yticks(color='w')
	plt.legend(bbox_to_anchor =(1.05, 1.0))
	ax.set_aspect(1)