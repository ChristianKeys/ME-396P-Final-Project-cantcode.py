import numpy as np 
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from scipy.spatial import distance
from functools import partial

def init():
	pass # Stupid matplotlib!

def visualize_dynamic_planner(qinit, qgoal, djikstra_path, x_array, y_array, fig, ax):

	def update_mobile(x_robot, y_robot, x_robot_len, y_robot_len, path, i):

		x_center = x_robot[0] + x_robot_len
		y_center = y_robot[0] + y_robot_len

		return path[i][0], path[i][1], np.pi

	# Plotting:

	ax = ax
	plt.sca(ax)

	plt.tick_params(left=False,bottom=False)
	plt.xticks(color='w')
	plt.yticks(color='w')
	ax.set_aspect(1)
	plt.title("Continuous Representation")
	tx = ax.text(0, -0.5, "Mobile Robot X-Pos: 1")
	ty = ax.text(0, -1, "Mobile Robot Y-Pos: 1")

	# Mobile robot:

	x_robot = np.array([qinit])
	y_robot = np.array([qgoal])
	x_robot_len = 0.5
	y_robot_len = 0.5

	# Create path based on djikstra:

	x_path = np.array([])
	y_path = np.array([])
	speed = 0.3

	for i in range(len(djikstra_path) - 1):

		k1 = djikstra_path[i]
		k2 = djikstra_path[i+1]

		d = np.sqrt((x_array[k1] - x_array[k2])**2 + (y_array[k1] - y_array[k2])**2)

		params = np.polyfit(np.array([x_array[k1], x_array[k2]]), np.array([y_array[k1], y_array[k2]]), deg=1)

		x_range = np.linspace(x_array[k1], x_array[k2], int(d/speed))
		y_range = np.linspace(y_array[k1], y_array[k2], int(d/speed))

		y_line = x_range*params[0] + params[1]

		x_path = np.concatenate((x_path, x_range), axis=0)
		y_path = np.concatenate((y_path, y_line), axis=0)

	path = np.array(list(zip(x_path, y_path)))

	n_path = len(path)

	ax.add_patch(plt.Rectangle((x_robot, y_robot), x_robot_len, y_robot_len, edgecolor = "black", facecolor = "red", zorder=1))

	def update_plot(i, x_robot, y_robot):

		ax.patches.pop()

		# Update mobile robot:

		dx, dy, theta = update_mobile(x_robot, y_robot, x_robot_len, y_robot_len, path, i)

		ax.add_patch(plt.Rectangle((dx - x_robot_len/2, dy - y_robot_len/2), x_robot_len, y_robot_len, 
			edgecolor = "black", facecolor = "red", zorder=1))

		tx.set_text("Mobile Robot X-Pos: " + "{:.2f}".format(dx))
		ty.set_text("Mobile Robot Y-Pos: " + "{:.2f}".format(dy))

		return ax, tx, ty

	anim = FuncAnimation(fig, init_func=init, func=partial(update_plot, x_robot=x_robot, y_robot=y_robot),
	repeat=True, frames=np.arange(0,n_path), interval=50)

	anim.save("python_simulator.gif", dpi=300, writer=PillowWriter(fps=25))