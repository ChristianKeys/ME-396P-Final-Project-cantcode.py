import random
import numpy as np
import math
import copy
import matplotlib.pyplot as plt

nodes = []
nodelist = []
cost = []
obs = []

def rrt(xlim, ylim, limit, obs, delta, goal):
    
    #xlim=(0, 10), ylim=(0,10), limit = 500, obs = [], delta=0.5, goal = []
            # Assume goal = [goal_x, goal_y]
    # get random point in the free space
    # find closest node in the tree
    # compute the position of the new node
    # collision check
    # if collision doesn't happen in extending the nearest node to the new node add it to the tree
    #check if we reached the goal 
    #run rrt
    def randomPoint():
        #goalSampleRate = 10
        #if random.randint(0,100) > goalSampleRate:
        xnew = [random.uniform(xlim[0], xlim[1]), 
               random.uniform(ylim[0], ylim[1])]
        # else:
        #     rnd = [goal[0], goal[1]]
    #gets random point in free space
    def is_valid_nodes(p, obs):
        eps = 0.25
        for obstacle in obs:
            if obstacle[4] == "d":
                return True
            else:
                ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[2] + eps)
                ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[3] + eps)
                if p[0] > ob_x[0] and p[0] < ob_x[1] and p[1] > ob_y[0] and p[1] < ob_y[1]: 
                    return False
        return True
    
    def getNearestListIndex(nodes, xnew):
        distanceList = [((node[0] - xnew[0])**2 + 
                 (node[1] - xnew[1])**2)**(1/2) for node in nodes]
        minIndex = distanceList.index(min(distanceList))
        return minIndex

    def getNewNode(theta, nearind, nearestNode):
        newNode = copy.deepcopy(nearestNode)

        newNode[0] += delta * math.cos(theta)
        newNode[1] += delta * math.sin(theta)
        cost.append(delta)
        newNode.parent = nearind 
        return newNode 
    
    
    
    def is_valid_edge(p1, p2):

        def ccw(p1,p2,p3):

            return (p3[1]-p1[1])*(p2[0]-p1[0]) > (p2[1]-p1[1])*(p3[0]-p1[0])

        for obstacle in obs:

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
    
    #maybe do not need
    def __CollisionCheck(newNode, obs):
        eps = 0.25

        for obstacle in obs:

            if obstacle[4] == "d":

                return True

            else:

                ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[2] + eps)
                ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[3] + eps)

                if newNode.x > ob_x[0] and newNode.x < ob_x[1] and newNode.y > ob_y[0] and newNode.y < ob_y[1]:

                    return False
        return True
        return True

    #def shortest_k_neighbors(p, cur_idx, k, x_array, y_array):
    i = 0
    while i < limit:
        xnew = randomPoint()  
        
        if is_valid_node(xnew, obs):
			# Store point's coordinates:

			x_array[i] = x
			y_array[i] = y
			i += 1
        
        
        