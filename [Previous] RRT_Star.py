#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  3 14:24:50 2022

@author: karanchawla
	Modified: jameshous, scratcho
"""
import random
import numpy as np 
import math 
import copy
import matplotlib.pyplot as plt

show_animation = True 

class RRTFamilyPlanners():

    def __init__(self, start, goal, obstacleList, randArea, expandDis=0.5, goalSampleRate=10, maxIter=200):

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList
    
    
    def sampleFreeSpace(self):
        if random.randint(0,100) > self.goalSampleRate:
            rnd = [random.uniform(self.minrand, self.maxrand),
                   random.uniform(self.minrand, self.maxrand)]
        else:
            rnd = [self.goal.x, self.goal.y]

        return rnd

    def getNearestListIndex(self, nodes, rnd):
        dList = [(node.x - rnd[0])**2 + 
                 (node.y - rnd[1])**2 for node in nodes]
        minIndex = dList.index(min(dList))
        return minIndex

    def getNewNode(self, theta, nind, nearestNode):
        newNode = copy.deepcopy(nearestNode)

        newNode.x += self.expandDis * math.cos(theta)
        newNode.y += self.expandDis * math.sin(theta)

        newNode.cost += self.expandDis
        newNode.parent = nind 
        return newNode
    
    
    def __CollisionCheck(self, newNode, obstacleList):

        eps = 0.25

        for obstacle in obstacleList:

            if obstacle[4] == "d":

                return True

            else:

                ob_x = (obstacle[0] - eps, obstacle[0] + obstacle[2] + eps)
                ob_y = (obstacle[1] - eps, obstacle[1] + obstacle[3] + eps)

                if newNode.x > ob_x[0] and newNode.x < ob_x[1] and newNode.y > ob_y[0] and newNode.y < ob_y[1]:

                    return False

        return True

    def isNearGoal(self, node):
        d = self.lineCost(node, self.goal)
        if d < self.expandDis:
            return True 
        return False  
    
    
##################################################################################

    # Make a new node perpendicular to exist path with minimum distance
   
    def RRTStarSearch(self, animation=True):
        self.nodeList = [self.start]
        iter = 1
        while True:
            rnd = self.sampleFreeSpace()
            nind = self.getNearestListIndex(self.nodeList, rnd)
            nearestNode = self.nodeList[nind]
            # steer 
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            newNode = self.getNewNode(theta, nind, nearestNode)

            if self.__CollisionCheck(newNode, self.obstacleList):
                nearinds = self.findNearNodes(newNode)
                newNode = self.chooseParent(newNode, nearinds)
                self.nodeList.append(newNode)
                self.rewire(newNode, nearinds)

            iter += 1
            if(iter == self.maxIter):
                break

            if animation:
                self.drawGraph(rnd)

            if self.isNearGoal(newNode):
                break

        # get path 
        lastIndex = len(self.nodeList) -1
        path = self.getFinalCourse(lastIndex)

        return path

    # Rewire new node which is perpendicular to a existing line  with previous node
    def rewire(self, newNode, nearInds):
        nnode = len(self.nodeList)
        for i in nearInds:
            nearNode = self.nodeList[i]

            d = math.sqrt((nearNode.x - newNode.x)**2 +
                          (nearNode.y - newNode.y)**2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(newNode.y - nearNode.y , 
                                   newNode.x - nearNode.x)
                if self.check_collision_extend(nearNode, theta, d):
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost

    def check_collision_extend(self, nearNode, theta, d):
        tmpNode = copy.deepcopy(nearNode)

        for i in range(int(d / self.expandDis)):
            tmpNode.x += self.expandDis * math.cos(theta)
            tmpNode.y += self.expandDis * math.sin(theta)
            if not self.__CollisionCheck(tmpNode, self.obstacleList):
                return False

        return True

    # [Modify] Finding Nodes for rewire
    def findNearNodes(self, newNode):
        nnode = len(self.nodeList)
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        dlist = [(node.x - newNode.x) ** 2 +
                (node.y - newNode.y) ** 2 for node in self.nodeList]
        nearinds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return nearinds

    def chooseParent(self, newNode, nearInds):
        if len(nearInds) == 0:
            return newNode

        dList = []
        for i in nearInds:
            dx = newNode.x - self.nodeList[i].x
            dy = newNode.y - self.nodeList[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.nodeList[i], theta, d):
                dList.append(self.nodeList[i].cost + d)
            else:
                dList.append(float('inf'))

        minCost = min(dList)
        minInd = nearInds[dList.index(minCost)]

        if minCost == float('inf'):
            print("mincost is inf")
            return newNode

        newNode.cost = minCost
        newNode.parent = minInd

        return newNode

    def getFinalCourse(self, lastIndex):
        path = [[self.goal.x, self.goal.y]]
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
        return path

    def getBestLastIndex(self):
        disgList = [self.calcDistToGoal(node.x, node.y) 
                    for node in self.nodeList]
        goalInds = [disgList.index(i) for i in disgList if i <= self.expandDis]

        if len(goalInds) == 0:
            return None 

        minCost = min([self.nodeList[i].cost for i in goalInds])
        for i in goalInds:
            if self.nodeList[i].cost == minCost:
                return i

        return None 

    def calcDistToGoal(self, x, y):
        return np.linalg.norm([x - self.goal.x, y - self.goal.y])
    
    
    def lineCost(self, node1, node2):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
##################################################################################
    def drawGraph(self, rnd=None):
        
        plt.clf()
        ax = plt.gca()
        if rnd is not None: 
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None: 
                if node.x or node.y is not None: 
                    plt.plot([node.x, self.nodeList[node.parent].x], [
                          node.y, self.nodeList[node.parent].y], "-g")

        for [ox, oy, lx, ly, ob_type] in self.obstacleList:
            
            if ob_type == "d":

                color = "green"
                alpha = 0.2

            else:

                color = "blue"
                alpha = 1.0
                
            ax.add_patch(plt.Rectangle((ox, oy), lx, ly,
            edgecolor = "black", facecolor = color, alpha=alpha))
            
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis([self.minrand, self.maxrand, self.minrand,self.maxrand])
        plt.grid(True)
        plt.pause(0.01)

class Node():

    def __init__(self, x, y):
        self.x = x 
        self.y = y
        self.cost = 0.0 
        self.parent = None 


def main():
    print("Start rrt planning")

    # ====Search Path with RRT====
    obstacleList = [[1, 1, 2.5, 2.5, "s"],
                [4, 7, 1, 1, "s"],
                [8, 3, 1, 5, "s"],
                [5, 7, 3, 1, "d"]]

    # Set Initial parameters
    rrt = RRTFamilyPlanners(start = [0, 0], goal = [11, 7],
              randArea = [-1, 14], obstacleList = obstacleList)
    path = rrt.RRTStarSearch(animation = show_animation)

    # Draw final path
    
    if show_animation:
        rrt.drawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()
