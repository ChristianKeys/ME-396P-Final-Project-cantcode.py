import numpy as np 
import matplotlib.pyplot as plt
import random as rd

def prm(N=200, xlim=(0, 10), ylim=(0,10), k=6):
	"""
	Populates space with valid nodes and edges for path-planning algorithms.

		Parameters:

			N (int): Number of samples.
			xlim (float 2-tuple): Enviroment x bounds.
			ylim (float 2-tuple): Enviroment y bounds.
			k (int): Maximum number of neighbors for each node.

		Returns:

			edges (int-list dictionary): Nodes' IDs and connectivities.
			x_array (N x 1 float array): Nodes' x coordinates.
			y_array (N x 1 float array): Nodes' y coordinates.

	"""

	def is_valid_node(p, obstacles):
		"""
		Determines whether a point is inside existing static obstacles.

			Parameters:

				p (float 2-tuple): x and y coordinates of the point.
				obstacles (n x 5 list of lists): obstacles' properties.

			Returns:

				True or False (bool).

		"""

		eps = 0.25

		for obstacle in obstacles:

			if obstacle[4] == "d":

				return True

			else:

				ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[2] + eps)
				ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[3] + eps)

				if p[0] > ob_x[0] and p[0] < ob_x[1] and p[1] > ob_y[0] and p[1] < ob_y[1]:

					return False

		return True

	def shortest_k_neighbors(p, cur_idx, k, x_array, y_array):

		# Watch out for indices names.

		delta_array = np.zeros(len(x_array))

		for i in range(len(x_array)):

			delta_array[i] = np.sqrt((x_array[i] - p[0])**2 + (y_array[i] - p[1])**2)

		min_idx = []
		min_dis = []

		j = 0

		while j < k:

			delta_min = np.argmin(delta_array)

			if delta_min != cur_idx and is_valid_edge(p, (x_array[delta_min], y_array[delta_min])):

				min_idx.append(delta_min)
				min_dis.append(delta_array[delta_min])
				j += 1

			delta_array[delta_min] = float("Inf")

		return min_idx, min_dis

	def is_valid_edge(p1, p2):

		def ccw(p1,p2,p3):

			return (p3[1]-p1[1])*(p2[0]-p1[0]) > (p2[1]-p1[1])*(p3[0]-p1[0])

		for obstacle in obstacles:

			if obstacle[4] == "s":

				p3_l = (obstacle[0], obstacle[1])
				p4_ll = (obstacle[0]+obstacle[2], obstacle[1])
				p4_lr = (obstacle[0], obstacle[1]+obstacle[3])

				p3_r = (obstacle[0]+obstacle[2], obstacle[1]+obstacle[3])

				if ccw(p1,p3_l,p4_lr) != ccw(p2,p3_l,p4_lr) and ccw(p1,p2,p3_l) != ccw(p1,p2,p4_lr): return False
				elif ccw(p1,p3_l,p4_ll) != ccw(p2,p3_l,p4_ll) and ccw(p1,p2,p3_l) != ccw(p1,p2,p4_ll): return False
				elif ccw(p1,p3_r,p4_ll) != ccw(p2,p3_r,p4_ll) and ccw(p1,p2,p3_r) != ccw(p1,p2,p4_ll): return False
				elif ccw(p1,p3_r,p4_lr) != ccw(p2,p3_r,p4_lr) and ccw(p1,p2,p3_r) != ccw(p1,p2,p4_lr): return False

		return True

	# Define data structures:

	x_array = np.zeros(N)
	y_array = np.zeros(N)
	edges = dict()
	eps = 0.5

	obstacles = [[1, 1, 2.5, 2.5, "s"],
				[4, 7, 1, 1, "s"],
				[8, 3, 1, 5, "s"],
				[5, 7, 3, 1, "d"]]

	i = 0

	while i < N:

		# Sample random point in configuration space:

		x = rd.uniform(xlim[0]+eps, xlim[1]-eps)
		y = rd.uniform(ylim[0]+eps, ylim[1]-eps)
		p = (x,y)

		# Check if point is valid:

		if is_valid_node(p, obstacles):

			# Store point's coordinates:

			x_array[i] = x
			y_array[i] = y
			i += 1

	# Add k closest neighbors to each node:

	for i in range(N):

		p = (x_array[i],y_array[i])
		min_idx, min_dis = shortest_k_neighbors(p, i, k, x_array, y_array)
		edges[i] = dict()

		for j in range(k):

			edges[i][min_idx[j]] = min_dis[j]

	# Enforce symmetry:

	for i in range(N):

		edge = edges[i]

		for idx in edge:

			if i not in edges[idx]:

				edges[idx][i] = edge[idx]

	return edges, x_array, y_array





