import matplotlib.pyplot as plt

def plot_path(path, x_array, y_array, ax):
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

	plt.scatter(x_array[path[0]], y_array[path[0]], color="goldenrod", 
	marker="s", s = 50, zorder=3, alpha=1.0, label="Start")

	plt.scatter(x_array[path[-1]], y_array[path[-1]], color="blueviolet",
	marker="s", s = 50, zorder=3, alpha=1.0, label="Goal")

	for i in range(len(path)-1):

		k1 = path[i]
		k2 = path[i+1]

		plt.plot([x_array[k1], x_array[k2]], [y_array[k1], y_array[k2]],
		color="g", alpha=1.0, zorder=2, lw=3.0)

	plt.legend(bbox_to_anchor=(1.3, 1.0))

	return 