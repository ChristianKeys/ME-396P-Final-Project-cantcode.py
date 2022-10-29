import numpy as np 
import matplotlib.pyplot as plt
import random as rd
from scipy.spatial import cKDTree

obstacles = [[1, 1, 2.5, 2.5, "s"],
			[5, 7, 1, 1, "s"],
			[8, 3, 1, 5, "s"],
			[6, 7, 1, 1, "d"]]

def is_inside_rectangle(p, a1, a2, a3, a4):

	t1 = (a1, p, a4)
	t2 = (a4, p, a3)
	t3 = (a3, p, a2)
	t4 = (a1, p, a2)

	tlist = [t1, t2, t3, t4]

	for t in tlist:

		pass

	return 0 

def is_valid_node(p):

	eps = 0.1

	ob1_x = (1 - eps, 3.5 + eps)
	ob2_x = (5 - eps, 6 + eps)
	ob3_x = (8 - eps, 9 + eps)

	ob1_y = (1 - eps, 3.5 + eps)
	ob2_y = (7 - eps, 8 + eps)
	ob3_y = (3 - eps, 8 + eps)

	if p[0] > ob1_x[0] and p[0] < ob1_x[1] and p[1] > ob1_y[0] and p[1] < ob1_y[1]:

		return False

	if p[0] > ob2_x[0] and p[0] < ob2_x[1] and p[1] > ob2_y[0] and p[1] < ob2_y[1]:

		return False

	if p[0] > ob3_x[0] and p[0] < ob3_x[1] and p[1] > ob3_y[0] and p[1] < ob3_y[1]:

		return False

	return True

def shortest_kpoints(p, x_array, y_array):

	min_dist, min_dist_idx = cKDTree(p).query([x_array, y_array], 1)

	print(min_dist_idx)

	return min_dist_idx

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

# Building Phase:

N = 30
xlim = [0, 10]
ylim = [0, 10]

nodes = []
x_array = np.zeros(N)
y_array = np.zeros(N)
edges = dict()

i = 0
k = 3

while i < N:

	# Sample random point in configuration space:

	x = rd.uniform(xlim[0], xlim[1])
	y = rd.uniform(ylim[0], ylim[1])

	# Remove redudant points:

	p = (x,y)

	if is_valid_node(p):

		nodes.append(i)
		edges[i] = []

		for j in range(len(nodes)):

			if i != j and is_valid_edge(p, (x_array[j], y_array[j])):

				# For now put this here: we want the k-nearest neighbors so, we will remove n-k edges:

				if len(edges[i]) > k:

					continue

					#distance_array = []
					#candidate_point = (x_array[j], y_array[j])

					#for edge in edges[i]:

				else:

					edges[i].append(j)

		x_array[i] = x
		y_array[i] = y
		i += 1

fig, ax = plt.subplots(1,1)

for j in range(N):

	array = edges[j]

	for k in array:

		plt.plot([x_array[j], x_array[k]], [y_array[j], y_array[k]], color="k", alpha=0.5, zorder=0)

ax.add_patch(plt.Rectangle((1, 1), 2.5, 2.5, edgecolor = "black", facecolor = "blue"))
ax.add_patch(plt.Rectangle((5, 7), 1, 1, edgecolor = "black", facecolor = "blue"))
ax.add_patch(plt.Rectangle((8, 3), 1, 5, edgecolor = "black", facecolor = "blue"))
ax.add_patch(plt.Rectangle((6, 7), 2, 1, edgecolor = "black", facecolor = "green", alpha=0.5))

plt.scatter(x_array, y_array, color="red", s = 50, zorder=1)
plt.scatter(0.5,0.5,color="green", marker="x", s = 100)
plt.scatter(9.5,9.5,color="green", marker="x", s = 100)
plt.show()
plt.close()





