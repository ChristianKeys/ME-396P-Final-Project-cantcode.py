import random as rd
import numpy as np
import math
import copy
import matplotlib.pyplot as plt

nodes = []
nodelist = []
x_array = [0.5]
y_array = [0.5]
eps = 0.25

def rrt(limit, obstacles, goal, delta=0.1, xlim=(0, 10), ylim=(0, 10)):

    """
    Populates space with valid nodes and edges for path-planning algorithm
        
        Parameters:
            
            limit: number of iterations allowable
            obstacles (list of lists): shape of static obstacles
            goal (float 2-tuple): coordination of goal point
            delta (float): distance between current and new node
            xlim (float 2-tuple): environment x bounds
            ylim (float 2-tuple): environment y bounds
            
        Returns: graph, x_array, y_array, path 
        
            graph (int-list dictionary): nodes' IDs and connectivities
            x_array (N x 1 float array): nodes' x coodinates
            y_array (N x 1 float array): nodes' y coodinates
            path (N x 1 float array): node indices that make up the path
            
    """        
    
    def is_valid_node(p, obstacles, eps):
        """
        Determines whether a point is inside given obstacles.

            Parameters:

                p (float 2-tuple): x and y coordinates of the point.
                obstacles (n x 5 list of lists): obstacles' properties.

            Returns:

                True or False (bool).

        """

        for obstacle in obstacles:

            ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[2] + eps)
            ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[3] + eps)

            if p[0] > ob_x[0] and p[0] < ob_x[1] and p[1] > ob_y[0] and p[1] < ob_y[1]:

                return False

        return True

    def getNewNode(theta, nearestNode, nearest):
        
        """
        Creates new node along the same direction to the goal
        
            Parameters:
                
                theta (float): angle created by new node and closest node, relative to the x axis
                nearestNode (float 2-tuple): nearest node to random new node
                nearest (float): distance between nearest node and random new node
            
            Returns:
                newNode (float 2-tuple): new node created
                distance (float): distance between nearest node and random new node
                
        """
        
        if delta < nearest:
            A = delta * math.cos(theta) + nearestNode[0]
            B = delta * math.sin(theta) + nearestNode[1]
            distance = delta
        else:
            A = nearest * math.cos(theta) + nearestNode[0]
            B = nearest * math.sin(theta) + nearestNode[1]
            distance = nearest


        newNode = (A,B)
        
        return newNode, distance
    
    def pNearest(x_array, y_array, p):
        
        """
        Finds the closest existing node to the randomly populated node
        
            Parameters:
                
                x_array (N x 1 list):
                y_array (N x 1 list):
                p (float 2-tuple):
                
            Returns:
                
                minindex (int): index of closest existing node to random new node
                nearest (float): distance between nearest node and random new node
                delta_array (1 x N list): list of distances from populated node to existing nodes
        """
        
        delta_array = []
        for i in range(len(x_array)):
            delta = np.sqrt((x_array[i] - p[0])**2 + (y_array[i] - p[1])**2)
            delta_array.append(delta)
        
        
        minindex = delta_array.index(min(delta_array))
        nearest = min(delta_array)
        return minindex, nearest, delta_array
    
    def is_valid_edge(p1, p2):

        """
        Finds if the edge created is valid or not
        
            Parameters:
                
                p1 (float 2-tuple): coordinates of random new populated node
                p2 (float 2-tuple): coodrinates of closest existing node
            
            Returns:
                
                True or False (bool)
        """

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
    
    i = 1
    graph = dict()
    graph[0] = dict()
    graph[0]["parent"] = None
    pathfound = False
    
    while i < limit:
        
        x = rd.uniform(xlim[0]+eps, xlim[1]-eps)
        y = rd.uniform(ylim[0]+eps, ylim[1]-eps)
        p = (x,y)
        
        if not is_valid_node(p, obstacles, eps):
            continue
        
        #find nearest node
        minindex, nearest, delta_array = pNearest(x_array, y_array, p)
        
        if not is_valid_edge(p, (x_array[minindex], y_array[minindex])):
            continue
        
        theta = math.atan2(p[1] - y_array[minindex], p[0] - x_array[minindex])
        newNode, nearest = getNewNode(theta, (x_array[minindex], y_array[minindex]), nearest)
        nodes.append(newNode)
        x_array.append(newNode[0])
        y_array.append(newNode[1])
        
        
        graph[i] = dict()
        graph[i][minindex] = nearest
        graph[i]["parent"] = minindex
        
        if np.sqrt((x_array[-1] - goal[0])**2 + (y_array[-1] - goal[1])**2) < delta:
            print("great success")
            pathfound = True
            break
    
        i += 1
    #find valid path
    path = []
    path.append(i)
    if pathfound == True:
        while graph[i]["parent"] != None:
            i = graph[i]["parent"]
            path.append(i)
    print(path)      
    return graph, x_array, y_array, path