import matplotlib.pyplot as plt

def plot_obstacles(static_obstacles, dynamic_obstacles, ax):
	"""
	Plots user-defined obstacles in the specified axis.

		Parameters:

		static_obstacles (list of lists): Shape of static obstacles [X[0], Y[0], DX, DY].
		dynamic_obstacles (list of lists): Shape of dynamic obstacle trajectory [X[0], Y[0], DX, DY].
		ax (matplotlib object): Axis to be plotted on.

		Returns:

			None

	"""

	plt.sca(ax)

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

	ax.set_aspect(1)
	plt.tick_params(left=False,bottom=False)
	plt.xticks(color='w')
	plt.yticks(color='w')
	plt.axis([0, 10, 0, 10])

	return