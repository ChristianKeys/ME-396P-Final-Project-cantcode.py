# -*- coding: utf-8 -*-
"""
Created on Mon Nov  7 13:15:58 2022

@author: houst
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

##################################################################################
	def RRTSearch(self, animation=True):
		self.nodeList = [self.start]
		while True: 
			# get random point in the free space
			rnd = self.sampleFreeSpace()
			# find closest node in the tree
			nind  = self.getNearestListIndex(self.nodeList, rnd)
			nearestNode = self.nodeList[nind]
			theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
			# compute the position of the new node
			newNode = self.getNewNode(theta, nind, nearestNode)
			# collision check
			if not self.__CollisionCheck(newNode, self.obstacleList):
				continue 
			# if collision doesn't happen in extending the nearest node to the new node
			# add it to the tree
			self.nodeList.append(newNode)

			#check if we reached the goal 
			if self.isNearGoal(newNode):
				break

			if animation:
				self.drawGraph(rnd)

		# compute the path 
		lastIndex = len(self.nodeList) -1 
		path = self.getFinalCourse(lastIndex)

		return path	

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
		for (ox, oy, size) in obstacleList:
			dx = ox - newNode.x 
			dy = oy - newNode.y 
			d = dx * dx  + dy * dy
			if d <= 1.1 * size**2:
				return False #collision

		return True # safe

	def isNearGoal(self, node):
		d = self.lineCost(node, self.goal)
		if d < self.expandDis:
			return True 
		return False  

##################################################################################
	def getFinalCourse(self, lastIndex):
		path = [[self.goal.x, self.goal.y]]
		while self.nodeList[lastIndex].parent is not None:
			node = self.nodeList[lastIndex]
			path.append([node.x, node.y])
			lastIndex = node.parent
		path.append([self.start.x, self.start.y])
		return path

##################################################################################

	def lineCost(self, node1, node2):
		return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
##################################################################################
	def drawGraph(self, rnd=None):

		plt.clf()
		if rnd is not None: 
			plt.plot(rnd[0], rnd[1], "^k")
		for node in self.nodeList:
			if node.parent is not None: 
				if node.x or node.y is not None: 
					plt.plot([node.x, self.nodeList[node.parent].x], [
						  node.y, self.nodeList[node.parent].y], "-g")

		for (ox, oy, size) in self.obstacleList:
			plt.plot(ox, oy, "ok", ms = 30 * size)

		plt.plot(self.start.x, self.start.y, "xr")
		plt.plot(self.goal.x, self.goal.y, "xr")
		plt.axis([-2, 15, -2, 15])
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
    obstacleList = [
        (5, 5, 0.5),
        (9, 6, 1),
        (7, 5, 3),
        (1, 5, 1),
        (2, 2, 1), 
        (7, 9, 1)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt = RRTFamilyPlanners(start = [0, 0], goal = [5, 10],
              randArea = [-2, 15], obstacleList = obstacleList)
    path = rrt.RRTSearch(animation = show_animation)

    # Draw final path
    if show_animation:
        rrt.drawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    main()