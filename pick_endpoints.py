import numpy as np

def pick_endpoints(pinit, pgoal, x_array, y_array):

		delta_array1 = np.zeros(len(x_array))
		delta_array2 = np.zeros(len(x_array))

		for i in range(len(x_array)):

			delta_array1[i] = np.sqrt((x_array[i] - pinit[0])**2 + (y_array[i] - pinit[1])**2)
			delta_array2[i] = np.sqrt((x_array[i] - pgoal[0])**2 + (y_array[i] - pgoal[1])**2)

		delta_min1 = np.argmin(delta_array1)
		delta_min2 = np.argmin(delta_array2)

		return delta_min1, delta_min2

