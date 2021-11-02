#!/usr/bin/env python

import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import json
import tf
import tf2_ros
import sys
import numpy as np
from math import cos, sin, atan2, fabs, hypot, floor, pi, sqrt
import matplotlib.pyplot as plt 
import random as rnd
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from crazyflie_driver.msg import Position
from geometry_msgs.msg import TransformStamped, Vector3
import sys

#sys.path.append('/home/d/e/devrat/dd2419_ws/src/DD2419_proj/src/planning')
path = rospy.get_param("filepath")
sys.path.append(path)

from RRT_map import Map
from RRT_frontph1 import Planner
from map_frontph1 import GridMap
from functionsph1 import*

import matplotlib.pyplot as plt
from itertools import cycle
from copy import copy

frontiers = []
gmap = OccupancyGrid()

def map_callback(msg):
    global gmap
    gmap = msg

def glbpt_callback(msg):
    global frontiers
    pt = np.zeros(2)

    pt[0] = msg.pose.position.x
    pt[1] = msg.pose.position.y

    if len(frontiers) > 0:
        frontiers = np.vstack((frontiers, pt))
    else:
        frontiers = np.reshape(pt,(1,2))   

def locpt_callback(msg):
    global frontiers
    pt = np.zeros(2)

    pt[0] = msg.pose.position.x
    pt[1] = msg.pose.position.y

    if len(frontiers) > 0:
        frontiers = np.vstack((frontiers, pt))
    else:
        frontiers = np.reshape(pt,(1,2))



def main():
    global frontiers, gmap

    rospy.init_node('filterNode', anonymous=True)

    sub1 = rospy.Subscriber('globalfrontpt', PoseStamped, glbpt_callback)
    sub2 = rospy.Subscriber('localfrontpt', PoseStamped, locpt_callback)
    sub3 = rospy.Subscriber('gridmap', OccupancyGrid, map_callback)
    filt_pub = rospy.Publisher('filt_pt', PoseArray, queue_size=10)

    front_pub = rospy.Publisher('filtered_points', MarkerArray, queue_size=10)
    del_pub = rospy.Publisher('delpts', MarkerArray, queue_size=10)

    filt_points = PoseArray()
    
    cnt = 1
    while len(frontiers) < 1:
        if cnt%10 == 0: rospy.loginfo("[Filter]: Waiting for frontier points")
        pass

    cnt = 1
    while len(gmap.data) < 1:
        if cnt%150 == 0: rospy.loginfo("[Filter]: Waiting for map")
        pass

    rate = rospy.Rate(rospy.get_param("node_rate"))
    i=1
    del_indices = []
    temp = []
    while not rospy.is_shutdown():
        #rospy.loginfo("iteration, %d", i)
        front = copy(frontiers)
        front1 = copy(frontiers)
        mapi = copy(gmap)
        if len(front) > 1:
            #rospy.loginfo("wioefhweiofhwefiohweiohgiohweioghwiehghwioeghioghoiegheioghwgiowhiowh %i",len(front))
            for i in range(0, len(front)):
                #rospy.loginfo("%i, %s",i,front[i])
                idx = pointtoIndex(mapi.info,front[i])
                if mapi.data[idx] >= 20:
                    del_indices.append(i)
            front = np.delete(front, del_indices,axis=0)
            
            for k in del_indices:
                temp.append(front1[k])
            del_pub.publish(displaypathNodes(temp, [1,0.5,1]))

            del_indices *= 0

            if len(front) < 2:
                continue

            
            bandwidth = 0.3#estimate_bandwidth(front, quantile=0.2)#n_samples=100) #0.3
            ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
            ms.fit(front)
            cluster_centers = ms.cluster_centers_

            #frontiers = copy(cluster_centers)
            front_new = copy(cluster_centers)

            for i in range(0, len(front_new)):
                #rospy.loginfo("%i, %s",i,front[i])
                idx = pointtoIndex(mapi.info,front_new[i])
                if mapi.data[idx] >= 20:
                    del_indices.append(i)
            front_new = np.delete(front_new, del_indices,axis=0)
            
            for k in del_indices:
                temp.append(front1[k])
            del_pub.publish(displaypathNodes(temp, [1,0.5,1]))

            del_indices *= 0
            
            frontiers = copy(front_new)
            #rospy.loginfo("################################################################## %i",len(frontiers))

            filt_pub.publish(createptArray(frontiers)) #filered points 
            front_pub.publish(displaypathNodes(frontiers, [1,1,1])) #filered points 

        i = i+1
        rate.sleep()
            


if __name__ == '__main__':
    try:
        print("Running....")
        main()
        print("Closed")
    except rospy.ROSInterruptException:
        pass
