#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import Pose
import rospy
import json
import tf
import sys
import numpy as np
from math import cos, sin, atan2, fabs
import matplotlib.pyplot as plt

class Map:
    class obstacleNode:
        def __init__(self, x, y, z, zmin, size):
            self.x = x
            self.y = y
            self.z = z
            self.zmin = zmin
            self.size = size

    def __init__(self, frame_id, resolution,  radius, args):
        self.frame_id = frame_id
        self. resolution = resolution
        self.radius = radius
        wallextx = []
        wallexty = []
        wallextz = []
        self.obstacles = []
        with open(args[1], 'rb') as f:
            data = json.load(f)
            self.xw = (data['airspace']['max'][0] + abs(data['airspace']['min'][0]))/resolution
            self.yh = (data['airspace']['max'][1] + abs(data['airspace']['min'][1]))/resolution

            self.origin = (data['airspace']['min'][0], data['airspace']['min'][1])
            
            for wall in data["walls"]:
                w_start = wall["plane"]["start"]
                w_stop = wall["plane"]["stop"]
                wallextx.append(w_start[0])
                wallextx.append(w_stop[0])
                wallexty.append(w_start[1])
                wallexty.append(w_stop[1])
                wallextz.append(w_stop[2])

                w_coords = self.rayTrace(w_start[:2],w_stop[:2])
                for coord in w_coords:
                    #self.addTogrid(coord)
                    x,y = coord
                    if self.is_in_bounds(x,y):
                        ob = self.obstacleNode(x, y, w_stop[2], w_start[2], radius)
                        self.obstacles.append(ob)
            

        self.extermities = [min(wallextx), max(wallextx), min(wallexty), max(wallexty), max(wallextz)]

    def getobstacleList(self):
        return self.obstacles, self.extermities

    def is_in_bounds(self, x, y):
        if x < abs(self.origin[0]) and x >= -abs(self.origin[0]):
            if y < abs(self.origin[1]) and y >= -abs(self.origin[1]):
                return True
        return False  

    def rayTrace(self, start, end):
        # Bresenham's Line Algorithm adopted from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm#Python 
        x1, y1 = start
        x2, y2 = end
        x1 = x1/self.resolution
        x2 = x2/self.resolution
        y1 = y1/self.resolution
        y2 = y2/self.resolution
        dx = x2 - x1
        dy = y2 - y1


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

        error = int(dx/2)
        ystep = 1 if y1 < y2 else -1

        y = y1
        points = []

        for x in range(int(x1), int(x2) + 1):
            coord = (y, x) if is_steep else(x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        if swapped:
            points.reverse()

        traversed = []
        for pt in points:
            x,y = pt
            x = round(x*self.resolution,3)
            y = round(y*self.resolution,3)
            traversed.append((x,y))

        return traversed

    def inflate(self):
        h = int(self.xw)
        w = int(self.yh)
        for i in range(h):
            for j in range(w):
                if self.map_matrix[i,j] == 100: 
                    for k in range(i - self.radius, i + self.radius):
                        for l in range(j - self.radius, j + self.radius):
                            if ((i-k)**2 + (j-l)**2)**0.5 <= self.radius:
                                if not(self.map_matrix[k, l] == 100):                                   
                                    self.map_matrix[k][l] = self.inflateProb

    def plotobstacles(self):
        h = self.xw
        w = self.yh
        ax = plt.gca()
        for obs in self.obstacles:
            circle = plt.Circle((obs.x, obs.y), obs.size, color='r')
            plt.plot(obs.x, obs.y, '.', color = 'yellow')
            ax.add_patch(circle)
        
        
        #ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
        # (or if you have an existing figure)
        # fig = plt.gcf()
        

        


        #for node in self.nodes:
        #    plt.plot(node.x, node.y, 's', color = 'green')
        
        #plt.plot(self.startNode.x, self.startNode.y, 'X', color = 'lime')
        #plt.plot(goal[0], goal[1], 'X', color = 'deeppink')

        #x = []
        #y = []
        #for coord in path:
        #    x.append(coord[0])
         #   y.append(coord[1])

        #plt.plot(x, y, marker = 's', mfc = 'cyan', ms = '2', lw = '2', ls = '-')


def main(argv=sys.argv):
    
    rospy.init_node('mapNode', anonymous=True)

    args = rospy.myargv(argv=argv)

    inflateradius = 0.1
    inflateprob = 20

    mapping = Map('map', 0.1, inflateradius, args)
    obstacles, ext = mapping.getobstacleList()
    mapping.plotobstacles()
    plt.axis([-3.5, 3.5, -3.5, 3.5])
    plt.show()

    


if __name__ == '__main__':
    try:
        print("Running....")
        main()
        print("Closed")
    except rospy.ROSInterruptException:
        pass
