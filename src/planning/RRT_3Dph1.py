#!/usr/bin/env python
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import rospy
import json
import tf
import sys
import numpy as np
from math import cos, sin, atan2, fabs, hypot, floor, pi, sqrt
import matplotlib.pyplot as plt 
import random as rnd
from mpl_toolkits.mplot3d import Axes3D
from copy import copy

class Planner:
    class Node:
        def __init__(self, x, y, z, yaw):
            self.x = x
            self.y = y
            self.z = z 
            self.yaw = yaw
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.path_yaw = []
            self.parent = None
    
    class ptNode:
        def __init__(self, x, y, z, yaw):
            self.x = x
            self.y = y
            self.z = z 
            self.yaw = yaw
    
    def __init__(self, start, goal, iterations, res, inflateradius, obstacles, ext, args, maxdist = 0.2, path_res = 0.2, sample_rate = 5):
        
        self.startNode = self.Node(start[0], start[1], start[2], start[3])
        self.goalNode = self.Node(goal[0], goal[1], goal[2], goal[3])
        self.path_res = path_res
        self.sample_rate = sample_rate 
        self.iterations = iterations
        self.maxdist = maxdist
        self.nodes = []
        self.path = []
        self.obstacles = obstacles
        self.ext = ext
        self.yaw_cont = rospy.get_param("yaw_towards_motion")
        self.yaw_const_angle = rospy.get_param("yaw_const_angle")
        self.yaw_control = rospy.get_param("yaw_control")
        self.pts_in_bet = rospy.get_param("pts_in_bet")
        self.markers = []

        with open(args[1], 'rb') as f:
            world = json.load(f)

        for m in world["markers"]:
            self.markers.append(self.Node(m['pose']['position'][0], m['pose']['position'][1], m['pose']['position'][2],0))

        for s in world["roadsigns"]:
            self.markers.append(self.Node(s['pose']['position'][0], s['pose']['position'][1], s['pose']['position'][2], 0))
        
    
    def rrt(self):
        self.nodes = [self.startNode]

        for _ in range(self.iterations):
            randomNode = self.generaterandomNode()

            closestNode_idx = self.findclosestNode(randomNode)
            closestNode = self.nodes[closestNode_idx]

            tempNode = self.movetoNode(closestNode, randomNode, self.maxdist)

            if self.accidentCheck(tempNode):
                self.nodes.append(tempNode)

            goaldist, _, _ = self.linebetnodes(self.nodes[-1], self.goalNode)
            
            if goaldist <= 0.5:
                lastNode = self.movetoNode(self.nodes[-1], self.goalNode, self.maxdist)
                if self.accidentCheck(lastNode):
                    self.path = self.getFinalpath()
                    break

        rospy.loginfo("RRT Done")
        return self.path ,self.nodes
    
    def closetoGoal(self, goal):
        distances = []
        for node in self.nodes:
            dist = (node.x - goal.x)**2 + (node.y - goal.y)**2
            distances.append(dist)
        
        minDist = min(distances)
        if minDist <= 0.5**2:
            nearestidx = distances.index(minDist)
        else:
            return -1

        return(nearestidx)

    def getFinalpath(self):
        plannedpath  = [self.ptNode(self.goalNode.x, self.goalNode.y, self.goalNode.z, self.goalNode.yaw)]
        pathNode = self.nodes[(len(self.nodes)-1)]

        while pathNode.parent is not None:
            plannedpath.append(self.ptNode(pathNode.x, pathNode.y, pathNode.z, pathNode.yaw))
            pathNode = pathNode.parent
        
        plannedpath.append(self.ptNode(pathNode.x, pathNode.y, pathNode.z, pathNode.yaw))

        return plannedpath

    def accidentCheck(self, node):
        if node is None:
           return False
        dist_list = []
        #i = False
        #j = 1
        for obs in self.obstacles:
            for x, y, z in zip(node.path_x, node.path_y, node.path_z):
                if [x,y,z] != [self.startNode.x, self.startNode.y, self.startNode.z]:
                    #rospy.loginfo(j)
                    distance = (obs.x - x)**2 + (obs.y - y)**2
                    
                    if distance < (obs.size)**2:
                        if z < (obs.z + 0.2):
                            if z > (obs.zmin - 0.2):
                                return False

                    dist_list.append(distance)
        
        return True


    def movetoNode(self, fromNode, toNode, maxdist):
        tempNode = self.Node(fromNode.x, fromNode.y, fromNode.z, fromNode.yaw)
        dist, azimuth, altitude = self.linebetnodes(tempNode, toNode)

        tempNode.path_x = [tempNode.x]
        tempNode.path_y = [tempNode.y]
        tempNode.path_z = [tempNode.z]
        tempNode.path_yaw = [tempNode.yaw]

        if maxdist > dist:
            maxdist = dist

        steps = int(floor(maxdist/self.path_res))

        for _ in range(steps):
            tempNode.x = tempNode.x + self.path_res*cos(azimuth)
            tempNode.y = tempNode.y + self.path_res*sin(azimuth)
            tempNode.z = tempNode.z + self.path_res*sin(altitude)
            
            tempNode.path_x.append(tempNode.x)
            tempNode.path_y.append(tempNode.y)
            tempNode.path_z.append(tempNode.z)
            tempNode.path_yaw.append(tempNode.yaw)
        
        dist, azimuth, altitude = self.linebetnodes(tempNode, toNode)

        if dist <= self.path_res:
            tempNode.path_x.append(toNode.x)
            tempNode.path_y.append(toNode.y)
            tempNode.path_z.append(toNode.z)
            tempNode.path_yaw.append(toNode.yaw)
            tempNode.x = toNode.x
            tempNode.y = toNode.y
            tempNode.y = toNode.z
            tempNode.yaw = toNode.yaw
        
        tempNode.parent = fromNode

        return tempNode
        
        

    def linebetnodes(self, fromNode, toNode):
        dx = toNode.x - fromNode.x
        dy = toNode.y - fromNode.y
        dz = toNode.z - fromNode.z

        base = hypot(dx,dy)
        dist = sqrt(dx**2 + dy**2 + dz**2)#hypot(dx,dy)
        azimuth = atan2(dy,dx)
        altitude = atan2(dz,base) 
        
        return dist, azimuth, altitude


    def findclosestNode(self, currentNode):
        distances = []
        for node in self.nodes:
            dist = (node.x - currentNode.x)**2 + (node.y - currentNode.y)**2 + (node.z - currentNode.z)**2 
            distances.append(dist)
        
        nearestidx = distances.index(min(distances))

        return(nearestidx) 


    def generaterandomNode(self):
        #Narrow the random area
        if rnd.randint(0,100) > self.sample_rate:

            randx = rnd.uniform(self.ext[0], self.ext[1])
            randy = rnd.uniform(self.ext[2],self.ext[3])
            randz = rnd.uniform(0, self.ext[4])
            randyaw = rnd.uniform(-pi/2, pi/2)
            randNode = self.Node(randx, randy, randz, randyaw)
        else:
            randNode = self.goalNode
        
        return randNode

    def rayTrace(self, start, end, resolution):
        # Bresenham's Line Algorithm adopted from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python

        x1, y1, z1 = (start.x, start.y, start.z)
        x2, y2, z2 = (end.x, end.y, end.z)
        x1 = x1/resolution
        x2 = x2/resolution

        y1 = y1/resolution
        y2 = y2/resolution

        z1 = z1/resolution
        z2 = z2/resolution

        dx = x2 - x1
        dy = y2 - y1
        dz = z2 - z1


        is_steep = abs(dy) > abs(dx)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        dx = x2 - x1
        dy = y2 - y1


        errory = int(dx/2)
        errorz = int(dx/2)
        ystep = 1 if y1 < y2 else -1
        zstep = 1 if z1 < z2 else -1

        y = y1
        z = z1
        points = []

        for x in range(int(x1), int(x2) + 1):
            coord = (y, x, z) if is_steep else(x, y, z)
            points.append(coord)
            errory -= abs(dy)
            errorz -= abs(dz)
            if errory < 0:
                y += ystep
                errory += dx
            if errorz < 0:
                z += zstep
                errorz += dx

        if swapped:
            points.reverse()

        traversed = []
        for pt in points:
            x,y,z = pt
            x = round(x*resolution,3)
            y = round(y*resolution,3)
            z = round(z*resolution,3)
            traversed.append((x,y,z))

        return traversed
    
    def accidentCheckbetnodes(self, nodeA, nodeB):

        dist_list = []
        intpath = self.rayTrace(nodeA, nodeB, 0.1)
        for obs in self.obstacles:
            for x, y, z  in intpath:
                distance = (obs.x - x)**2 + (obs.y - y)**2
                dist_list.append(distance)
            #if min(dist_list) <= (obs.size)**2 :
            #    return False
                if distance < (obs.size)**2:
                    if z < (obs.z + 0.3):
                        return False
        return True

    def finalpathcheck(self, path):

        for obs in self.obstacles:
            for p in path:
                distance = (obs.x - p.x)**2 + (obs.y - p.y)**2

                if distance < (obs.size)**2:
                    if p.z < (obs.z + 0.1):
                        rospy.loginfo("Point Z %f,obs Z%f",p.z, obs.z)
                        return False
        return True

        
    def points(self, start, end):

        path = []
        dist, azimuth, altitude = self.linebetnodes(start, end)

        steps = int(floor(dist/self.pts_in_bet))
        tempNode = self.Node(start.x, start.y, start.z, start.yaw)
        path.append(start)

        for i in range(steps):
            tempNode.x = tempNode.x + self.pts_in_bet*cos(azimuth)
            tempNode.y = tempNode.y + self.pts_in_bet*sin(azimuth)
            tempNode.z = tempNode.z + self.pts_in_bet*sin(altitude)

            d = np.linalg.norm([tempNode.x - end.x, tempNode.y - end.y, tempNode.z - end.z])
            if d > 0.0001:
                path.append(self.Node(tempNode.x, tempNode.y, tempNode.z, end.yaw))

        return path

    def getpointsinbetween(self, path):
        path_new = copy(path)
        idx = 0
        paths = []

        for j in range(0,(len(path_new)-1)):
            int_path = self.points(path_new[j],path_new[j+1])
            paths = paths + int_path 
        
        paths.append(path_new[-1])      
        return paths
    
    def assignYaw(self, path):
        distances = []
        new_path = []
        for wp in path:
            for m in self.markers:
                dist = (m.x - wp.x)**2 + (m.y - wp.y)**2+ (m.z - wp.z)**2
                distances.append(dist)
            nearestidx = distances.index(min(distances))
            marker = self.markers[nearestidx]
            _, angle, _= self.linebetnodes(wp, marker)
            new_path.append(self.Node(wp.x,wp.y,wp.z,angle))
            distances *= 0
        
        return new_path

    def pathsmoothing(self, path):
        if path:
            pathrev = list(reversed(path))

            length = 0
            goal = (pathrev[-1].x, pathrev[-1].y, pathrev[-1].z)

            for _ in range(10):
                ite = len(pathrev)
                if ite > 2:
                    for i in range(ite - 1):
                        if i < len(pathrev)-1:
                            dist, ang, _ = self.linebetnodes(pathrev[i+1], pathrev[i])
                            #pathrev[i].yaw = ang + pi

                            if self.yaw_cont:
                                pathrev[i-1].yaw = ang + pi
                            else:
                                pathrev[i].yaw = self.yaw_const_angle#pi

                            length = length + dist
                            if i==0:
                                stdist = dist
                            else:
                                stdist, angle, _ = self.linebetnodes(pathrev[i+1],pathrev[i-1])

                            if stdist < length:
                                if self.accidentCheckbetnodes(pathrev[i-1], pathrev[i+1]):
                                    if i != 0:
                                        if (pathrev[i].x, pathrev[i].y, pathrev[i].z) != goal:
                                            pathrev.pop(i)
                                            #pathrev[i-1].yaw = angle + pi
                                            if self.yaw_cont:
                                                pathrev[i-2].yaw = angle + pi
                                            else:
                                                pathrev[i-1].yaw = self.yaw_const_angle#pi

                            if (i+1)%3 ==0:
                                length = 0
            
            paths = self.getpointsinbetween(copy(pathrev))
    
            if self.yaw_control:
                paths = self.assignYaw(paths)
            
            if self.finalpathcheck(paths):
                return list(reversed(paths)) #list(reversed(pathrev))
            else:
                rospy.loginfo("Path is empty by MEEEEE wefwefweffwefwefwfeefwe")
                paths = []
                return paths
        else:
            rospy.loginfo("Path is empty wefwefweffwefwefwfeefwe")
            return path

            
    def plotobstacles(self, goal, path):
        ax1 = plt.gca()
        ax = plt.axes(projection='3d')

        for node in self.nodes:
            ax.plot3D(node.x, node.y, node.z, color = 'green')
        
        ax.plot3D(self.startNode.x, self.startNode.y, self.startNode.z, color = 'lime')
        ax.plot3D(goal[0], goal[1], goal[2], color = 'deeppink')

        x = []
        y = []
        z = []
        for coord in path:
            x.append(coord[0])
            y.append(coord[1])
            z.append(coord[2])

        ax.plot3D(x, y, z)

        
