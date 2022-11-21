import matplotlib.pyplot as plt

def plot_path(path, x_array, y_array, color, ax, legend=True):
	"""
	Plots the path generated.

		Parameters:

		path (N x 1 float array): Node indices that make up the path.
		x_array (N x 1 float array): Nodes' x coordinates.
		y_array (N x 1 float array): Nodes' y coordinates.
		ax (matplotlib object): Axis to be plotted on.

		Returns:

			None

	"""

	# If path is empty, do not plot anything:

	if not path:
		return 

	plt.sca(ax)
	N = len(x_array)

	for i in range(len(path)-1):

		k1 = path[i]
		k2 = path[i+1]

		plt.plot([x_array[k1], x_array[k2]], [y_array[k1], y_array[k2]],
		color=color, alpha=1.0, zorder=2, lw=3.0)

	if legend:
		#plt.plot(-1, -1, color="r", label="Near Optimum")
		plt.plot(-1, -1, color="g", label="Calculated Path")
		plt.scatter(x_array[path[0]], y_array[path[0]], color="goldenrod", 
		marker="s", s = 50, zorder=3, alpha=1.0, label="Start")

		plt.scatter(x_array[path[-1]], y_array[path[-1]], color="blueviolet",
		marker="s", s = 50, zorder=3, alpha=1.0, label="Goal")

		plt.legend(loc="upper left")

	return 