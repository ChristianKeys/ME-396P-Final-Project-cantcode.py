import matplotlib.pyplot as plt

def plot_graph(graph, x_array, y_array, ax):
	"""
	Plots the nodes and edges of any generated graph.

		Parameters:

		graph (int-list dictionary): Nodes' IDs and connectivities.
		x_array (N x 1 float array): Nodes' x coordinates.
		y_array (N x 1 float array): Nodes' y coordinates.
		ax (matplotlib object): Axis to be plotted on.

		Returns:

			None

	"""

	plt.sca(ax)

	N = len(x_array)

	for j in range(N):

		edge = graph[j]

		for idx in edge:

			if idx != "parent":

				if edge[idx] == float("Inf"):

					plt.plot([x_array[j], x_array[idx]], [y_array[j], y_array[idx]], 
					color="k", alpha=0.2, zorder=1, linestyle='dashed')

				else:
					plt.plot([x_array[j], x_array[idx]], [y_array[j], y_array[idx]],
					color="k", alpha=0.2, zorder=1)

	plt.scatter(x_array, y_array, color="red", s = 25, zorder=2)

	return