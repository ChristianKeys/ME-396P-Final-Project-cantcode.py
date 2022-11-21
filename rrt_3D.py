import random as rd
import numpy as np
import math
import copy
import matplotlib.pyplot as plt

def rrt(limit, obstacles, start, goal, delta=0.1, xlim=(0, 10), ylim=(0, 10), zlim=(0, 10), eps=0.25):
    """
    A simple implementation of RRT.
        
        Parameters:
            
            limit (int): Maximum number of iterations allowed.
            obstacles (list of lists): Shape of static obstacles.
            start (float 2-tuple): Coordinates of start.
            goal (float 2-tuple): Coordinates of goal.
            delta (float): Distance between current and new node.
            xlim (float 2-tuple): Environment x bounds.
            ylim (float 2-tuple): Environment y bounds.
            eps (float): Tolerance to obstacles.
            
        Returns: graph, x_array, y_array, path 
        
            graph (int-list dictionary): Nodes' IDs and connectivities.
            x_array (N x 1 float array): Nodes' x coodinates.
            y_array (N x 1 float array): Nodes' y coodinates.
            path (N x 1 float array): Node indices that make up the path.
            
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

            ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[3] + eps)
            ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[4] + eps)
            ob_z = (obstacle[2] - eps, obstacle[2] + obstacle[5] + eps)
            
            # Checking whether the point is inside the obstacle; Inside -> True
            
             # Obstacle x coordination
            inside_obstacle_x = False
            if p[0] > ob_x[0] and p[0] < ob_x[1]:
                inside_obstacle_x = True
                
             # Obstacle y coordination
            inside_obstacle_y = False
            if p[1] > ob_y[0] and p[1] < ob_y[1]:
                inside_obstacle_y = True
                
             # Obstacle z coordination
            inside_obstacle_z = False
            if p[2] > ob_z[0] and p[2] < ob_z[1]:
                inside_obstacle_y = True
            
             # Check all of the coordinate(x, y, z); Inside -> False
            if inside_obstacle_x and inside_obstacle_y and inside_obstacle_z is True:
                return False
            
        # If the point is outside of the obstacle -> True
        return True

    def getNewNode(theta, gamma, nearestNode, nearest):
        
        """
        Creates new node along the same direction to the goal
        
            Parameters:
                
                theta (float): angle created by new node and closest node, relative to the x axis
                gamma (flaot): angle created by new node and closest node, relative to the xy plane
                nearestNode (float 2-tuple): nearest node to random new node
                nearest (float): distance between nearest node and random new node
            
            Returns:
                newNode (float 2-tuple): new node created
                distance (float): distance between nearest node and random new node
                
        """

        if delta < nearest:
            A = delta * math.cos(gamma) * math.cos(theta) + nearestNode[0]
            B = delta * math.cos(gamma) * math.sin(theta) + nearestNode[1]
            C = delta * math.sin(gamma) + nearestNode[2]
            distance = delta
        else:
            A = nearest * math.cos(gamma) * math.cos(theta) + nearestNode[0]
            B = nearest * math.cos(gamma) * math.sin(theta) + nearestNode[1]
            C = nearest * math.sin(gamma) + nearestNode[2]
            distance = nearest


        newNode = (A,B,C)
        
        return newNode, distance
    
    def pNearest(x_array, y_array, z_array, p):
        
        """
        Finds the closest existing node to the randomly populated node
        
            Parameters:
                
                x_array (N x 1 list):
                y_array (N x 1 list):
                z_array (N x 1 list):
                p (float 2-tuple):
                
            Returns:
                
                minindex (int): index of closest existing node to random new node
                nearest (float): distance between nearest node and random new node
                delta_array (1 x N list): list of distances from populated node to existing nodes
        """
        
        delta_array = []
        for i in range(len(x_array)):
            delta = np.sqrt((x_array[i] - p[0])**2 + (y_array[i] - p[1])**2 + (z_array[i] - p[2])**2)
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
            
            # Acronym after the underscore:
                # [ z-axis /  'b': bottom, 't': top   ],
                # [ y-axis /  'd': down,   'u': up    ],
                # [ x-axis /  'l': left,   'r': right ]
            
            # Generating vertex lists of the obstacles
            
            """
            obstacle_vertex_lists[0] = left,  down, bottom
            obstacle_vertex_lists[1] = left,  down, top
            obstacle_vertex_lists[2] = left,  up,   bottom
            obstacle_vertex_lists[3] = left,  up,   top
            obstacle_vertex_lists[4] = right, down, bottom
            obstacle_vertex_lists[5] = right, down, top
            obstacle_vertex_lists[6] = right, up,   bottom
            obstacle_vertex_lists[7] = right, up,   top
            """

            obstacle_vertex_lists = []
            demension = 3
            
            for i in range(2): # left: 0, right: 1 / x-axis
                
                for j in range(2): # down: 0, up: 1 / y-axis
                    
                    for k in range(2): # bottom: 0, top: 1 / z-axis
                        
                        vertex = []
                        iteration_num = 0
                        for m in (i, j, k):
                            
                            if m % 2 == 0:
                                vertex_value = obstacle[iteration_num] - eps
                            elif m % 2 == 1:
                                vertex_value = obstacle[iteration_num] + obstacle[iteration_num + demension] + eps
                                
                            vertex.append(vertex_value)
                            iteration_num += 1
                            
                        obstacle_vertex_lists.append(vertex)
                        
            ###### Start from here #####
            
            # Definition of checking if the edge passes through the face of the obstacles on the plane between two axis
            
            """
            p1: start point of the edge
            p2: end point of the edge
            obv1: axis1_low, axis2_low
            obv2: axis1_low, axis2_high
            obv3: axis1_high, axis2_low
            obv4: axis1_high, axis2_high
            """
            def faceCheck(p1, p2, obv1, obv2, obv3, obv4):
                if ccw(p1,obv1,obv2) != ccw(p2,obv1,obv2) and ccw(p1,p2,obv1) != ccw(p1,p2,obv2): return False
                elif ccw(p1,obv1,obv3) != ccw(p2,obv1,obv3) and ccw(p1,p2,obv1) != ccw(p1,p2,obv3): return False
                elif ccw(p1,obv4,obv3) != ccw(p2,obv4,obv3) and ccw(p1,p2,obv4) != ccw(p1,p2,obv3): return False
                elif ccw(p1,obv4,obv2) != ccw(p2,obv4,obv2) and ccw(p1,p2,obv4) != ccw(p1,p2,obv2): return False
            
            return True

            # Checking the edge is inside the hexahedron
            if faceCheck(p1, p2, obstacle_vertex_lists[0], obstacle_vertex_lists[1], obstacle_vertex_lists[2], obstacle_vertex_lists[3]) == True:
                return True
            elif faceCheck(p1, p2, obstacle_vertex_lists[0], obstacle_vertex_lists[1], obstacle_vertex_lists[4], obstacle_vertex_lists[5]) == True:
                return True
            elif faceCheck(p1, p2, obstacle_vertex_lists[0], obstacle_vertex_lists[2], obstacle_vertex_lists[4], obstacle_vertex_lists[6]) == True:
                return True
            else:
                return False
    
    
    graph = dict()
    graph[0] = dict()
    graph[0]["parent"] = None
    pathfound = False
    x_array = [start[0]]
    y_array = [start[1]]
    z_array = [start[2]]

    i = 1
    
    while i < limit:
        
        x = rd.uniform(xlim[0]+eps, xlim[1]-eps)
        y = rd.uniform(ylim[0]+eps, ylim[1]-eps)
        z = rd.uniform(zlim[0]+eps, zlim[1]-eps)
        p = (x,y,z)
        
        if not is_valid_node(p, obstacles, eps):
            continue
        
        # Find nearest node:
        minindex, nearest, delta_array = pNearest(x_array, y_array, z_array, p)
        
        if not is_valid_edge(p, (x_array[minindex], y_array[minindex], z_array[minindex])):
            continue
        
        theta = math.atan2(p[1] - y_array[minindex], p[0] - x_array[minindex])
        gamma = math.atan2(p[2] - z_array[minindex], np.sqrt((p[0] - x_array[minindex])**2 + (p[1] - y_array[minindex])**2))
        newNode, nearest = getNewNode(theta, gamma, (x_array[minindex], y_array[minindex], z_array[minindex]), nearest)
        x_array.append(newNode[0])
        y_array.append(newNode[1])
        z_array.append(newNode[2])
        
        # Could you explain about how this code functions?
        graph[i] = dict()
        graph[i][minindex] = nearest
        graph[i]["parent"] = minindex
        
        if np.sqrt((x_array[-1] - goal[0])**2 + (y_array[-1] - goal[1])**2 + (z_array[-1] - goal[2]**2)) < delta:
            pathfound = True
            break
    
        i += 1

    # If available, backtrack to find path:

    path = []
    if pathfound == True:
        path.append(i)
        while graph[i]["parent"] != None:
            i = graph[i]["parent"]
            path.append(i)
    path.reverse()

    return graph, x_array, y_array, z_array, path
