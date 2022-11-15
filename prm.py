import numpy as np 
import matplotlib.pyplot as plt
import random as rd

def prm(obstacles, N=200, xlim=(0, 10), ylim=(0, 10), k=6, eps=0.25):
	"""
	A simple implementation of PRM.

		Parameters:

			obstacles (list of lists): Shape of static obstacles.
			N (int): Number of samples.
			xlim (float 2-tuple): Enviroment x bounds.
			ylim (float 2-tuple): Enviroment y bounds.
			k (int): Maximum number of neighbors for each node.
			eps (float): Tolerance to obstacles.

		Returns:

			edges (int-list dictionary): Nodes' IDs and connectivities.
			x_array (N x 1 float array): Nodes' x coordinates.
			y_array (N x 1 float array): Nodes' y coordinates.

	"""

	def is_valid_node(p, obstacles, eps):
		"""
		Determines whether a point is inside given obstacles.

			Parameters:

				p (float 2-tuple): Coordinates of the point.
				obstacles (n x 5 list of lists): Obstacles' properties.

			Returns:

				True or False (bool).

		"""

		for obstacle in obstacles:

			ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[2] + eps)
			ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[3] + eps)

			if p[0] > ob_x[0] and p[0] < ob_x[1] and p[1] > ob_y[0] and p[1] < ob_y[1]:

				return False

		return True

	def shortest_k_neighbors(cur_idx, k, x_array, y_array):
		"""
		Finds the nearest k-neighbors to the current index.

			Parameters:

				cur_idx (int): ID of current node being connected
				k (int): maximum number of nearest neighbors to be connected
				x_array (N x 1 float array): Nodes' x coordinates.
				y_array (N x 1 float array): Nodes' y coordinates.

			Returns:

				True or False (bool).

		"""

		# This requires a future overhaul (to ensure completeness and symmetry).

		p = (x_array[cur_idx],y_array[cur_idx])

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

		# This needs better coding practices and better explanation.

		def ccw(p1,p2,p3):

			return (p3[1]-p1[1])*(p2[0]-p1[0]) > (p2[1]-p1[1])*(p3[0]-p1[0])

		for obstacle in obstacles:

			p3_l = (obstacle[0]-eps, obstacle[1]-eps)
			p4_ll = (obstacle[0]+obstacle[2]+eps, obstacle[1]-eps)
			p4_lr = (obstacle[0]-eps, obstacle[1]+obstacle[3]+eps)
			p3_r = (obstacle[0]+obstacle[2]+eps, obstacle[1]+obstacle[3]+eps)

			if ccw(p1,p3_l,p4_lr) != ccw(p2,p3_l,p4_lr) and ccw(p1,p2,p3_l) != ccw(p1,p2,p4_lr): return False
			elif ccw(p1,p3_l,p4_ll) != ccw(p2,p3_l,p4_ll) and ccw(p1,p2,p3_l) != ccw(p1,p2,p4_ll): return False
			elif ccw(p1,p3_r,p4_ll) != ccw(p2,p3_r,p4_ll) and ccw(p1,p2,p3_r) != ccw(p1,p2,p4_ll): return False
			elif ccw(p1,p3_r,p4_lr) != ccw(p2,p3_r,p4_lr) and ccw(p1,p2,p3_r) != ccw(p1,p2,p4_lr): return False

		return True

	# Define data structures:

	x_array = np.zeros(N)
	y_array = np.zeros(N)
	edges = dict()

	i = 0

	while i < N:

		# Sample random point in configuration space:

		x = rd.uniform(xlim[0]+eps, xlim[1]-eps)
		y = rd.uniform(ylim[0]+eps, ylim[1]-eps)
		p = (x,y)

		# Check if point is valid:

		if is_valid_node(p, obstacles, eps):

			# Store point's coordinates:

			x_array[i] = x
			y_array[i] = y
			i += 1

	# Add k closest neighbors to each node:

	for i in range(N):

		min_idx, min_dis = shortest_k_neighbors(i, k, x_array, y_array)
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
