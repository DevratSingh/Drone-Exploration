#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid 
from geometry_msgs.msg import Pose
import rospy
import json
import tf
import sys
import numpy as np
from math import cos, sin, atan2, fabs
from visualization_msgs.msg import MarkerArray, Marker
from map_msgs.msg import OccupancyGridUpdate
path = rospy.get_param("filepath")
sys.path.append(path)
#sys.path.append('/home/d/e/devrat/dd2419_ws/src/DD2419_proj/src/planning')
from functionsph1 import*
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import tf2_ros

class GridMap:
    def __init__(self, frame_id, resolution, args, radius, inflateprob):
        self.frame_id = frame_id
        self. resolution = resolution
        self.origin = Pose()
        self.grid = OccupancyGrid()
        self.radius = int(radius)
        self.inflateProb = inflateprob
        self.wallpts = []
        wallextx = []
        wallexty = []
        
        with open(args[1], 'rb') as f:
            data = json.load(f)
            
            self.xw = (data['airspace']['max'][0] + abs(data['airspace']['min'][0]))/resolution
            self.yh = (data['airspace']['max'][1] + abs(data['airspace']['min'][1]))/resolution
            
            self.map_matrix = -1*np.ones((int(self.xw), int(self.yh))) # change 10 to -1

            self.origin.position.x = data['airspace']['min'][0]
            self.origin.position.y = data['airspace']['min'][1]

            self.trax = abs(self.origin.position.x)
            self.tray = abs(self.origin.position.y)
            
            for wall in data["walls"]:
                w_start = wall["plane"]["start"]
                w_stop = wall["plane"]["stop"]
                wallextx.append(w_start[0])
                wallextx.append(w_stop[0])
                wallexty.append(w_start[1])
                wallexty.append(w_stop[1])

                w_coords = self.rayTrace(w_start[:2],w_stop[:2])
                for coord in w_coords:
                    self.addTogrid(coord)

        self.extermities = [min(wallextx), max(wallextx), min(wallexty), max(wallexty)]
        rospy.loginfo(np.shape(self.map_matrix))
        self.inflate()
        

        
                
    
    def gridmap(self, map_matrix):
        self.grid.header.frame_id = self.frame_id
        self.grid.info.resolution = self.resolution
        self.grid.info.width = self.xw
        self.grid.info.height = self.yh
        self.grid.info.origin = self.origin
        #self.inflate()
        try:
            self.grid.data =  map_matrix.flatten('C')
        except:
            map_matrix = self.map_matrix
            self.grid.data =  map_matrix.flatten('C')
        self.wallpts = np.where(self.grid.data > 50)
        self.wallpts = np.reshape(self.wallpts, (np.shape(self.wallpts)[1],1))

        return self.grid, map_matrix, self.extermities, self.wallpts
    
    
    def addTogrid(self, coord):
        x,y = coord
        if self.is_in_bounds(x,y):
            j = int((self.tray + x)/self.resolution)
            i = int((self.trax + y)/self.resolution)
            self.map_matrix[i][j] = 100
        else:
            rospy.loginfo("Out of bounds")
            rospy.loginfo(coord)
            
    def pttoidx(self, x, y):
        j = int((self.tray + x)/self.resolution)
        i = int((self.trax + y)/self.resolution)
        return i, j

    def is_in_bounds(self, x, y):
        if x < self.trax and x >= -self.trax:
            if y < self.tray and y >= -self.tray:
                return True
        return False 
    
    def indextocoord(self, i, j):
        x = j*self.resolution - self.trax
        y = i*self.resolution - self.tray
        return np.array([x,y]) 

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
                    #x, y = self.indextocoord(self, i, j)
                    for k in range(i - self.radius, i + self.radius):
                        for l in range(j - self.radius, j + self.radius):
                            if ((i-k)**2 + (j-l)**2)**0.5 <= self.radius:
                                if not(self.map_matrix[k, l] == 100):                                   
                                    self.map_matrix[k][l] = self.inflateProb



"""
def main(argv=sys.argv):
    
    rospy.init_node('mapNode', anonymous=True)
    map_pub = rospy.Publisher('gridmap', OccupancyGrid, queue_size=10, latch=True)
    update_pub = rospy.Publisher('gridmap_updates', OccupancyGridUpdate, queue_size=10)
    cent_pub = rospy.Publisher('centroids', MarkerArray, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    args = rospy.myargv(argv=argv)

    inflateradius = 40
    inflateprob = 20

    mapping = GridMap('map',0.1, args, inflateradius, inflateprob)
    grid, _, _ , pts= mapping.gridmap(None)
    rospy.loginfo(" point 1, %s",len(pts))
    
    map_pub.publish(grid)
    width = 0.6
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        location = droneLocation(tfBuffer, 'mapfront')
        up, k1, k2 = grid_update(location, width)
        a,b,c,d, k_min, k_max = ROI([location[0],location[1], location[2]], width)

        f = indextoPoint(k_min)
        rospy.loginfo(" point 1, %s",f)
        f = indextoPoint(k_max)
        rospy.loginfo(" point , %s",f)
        cent_pub.publish(displaypathNodes([indextoPoint(k_min),indextoPoint(k_max), [location[0],location[1]], indextoPoint(0)],[1,0,0]))

        newdata = mark_explored(location, grid, width)
        

        up.data = newdata

        update_pub.publish(up)
        #map_pub.publish(grid)

        rate.sleep()

    rospy.spin()
    


if __name__ == '__main__':
    try:
        print("Running....")
        main()
        print("Closed")
    except rospy.ROSInterruptException:
        pass
"""