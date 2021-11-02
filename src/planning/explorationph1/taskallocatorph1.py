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
from std_msgs.msg import Bool

path = rospy.get_param("filepath")
sys.path.append(path)

#sys.path.append('/home/d/e/devrat/dd2419_ws/src/DD2419_proj/src/planning')

from RRT_map import Map
from RRT_frontph1 import Planner
from map_frontph1 import GridMap
from functionsph1 import*

import matplotlib.pyplot as plt
from itertools import cycle
from copy import copy

frontiers = []
gmap = OccupancyGrid()
tfBuffer = None
listener = None
robot_busy = False

def frontier_callback(msg):
    global frontiers
    
    for pose in msg.poses:
        x = pose.position.x
        y = pose.position.y
        #z = pose.position.z

        _, _, yaw = euler_from_quaternion((pose.orientation.x,
                                        pose.orientation.y,
                                        pose.orientation.z,
                                        pose.orientation.w))
        frontiers.append([x,y,yaw])

def status_callback(msg):
    global robot_busy
    robot_busy = msg.data

def map_callback(msg):
    global gmap
    gmap = msg

def pub_msg(point):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'map'
    
    msg.pose.position.x = point[0]
    msg.pose.position.y = point[1]
    msg.pose.position.z = 0.4#point[0]

    quat = quaternion_from_euler(0, 0, pi)

    msg.pose.orientation.x = quat[0] 
    msg.pose.orientation.y = quat[1] 
    msg.pose.orientation.z = quat[2] 
    msg.pose.orientation.w = quat[3]

    return msg

def main():
    global frontiers, gmap, tfBuffer, listener, robot_busy, robot_busy
    # Setup
    rospy.init_node('AllocatorNode', anonymous=True)

    sub1 = rospy.Subscriber('filt_pt', PoseArray, frontier_callback)
    sub2 = rospy.Subscriber('gridmap', OccupancyGrid, map_callback)
    robot_status_sub = rospy.Subscriber('robotStatus', Bool, status_callback)

    taskpt = rospy.Publisher('taskpoint', MarkerArray, queue_size=10, latch=True)
    task_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    cent_pub = rospy.Publisher('centroids', MarkerArray, queue_size=10)
    allexplored_pub = rospy.Publisher('allexplored', Bool, queue_size=2)


    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    cnt = 1
    while len(frontiers) < 1:
        if cnt%10 == 0: rospy.loginfo("[TaskAssigner]: Waiting for frontier points")
        pass
    cnt = 1
    while len(gmap.data) < 1:
        if cnt%10 == 0: rospy.loginfo("[TaskAssigner]: Waiting for map")
        pass

    location = droneLocation(tfBuffer, 'taskAssigner')
    ap_prev = np.array([location[0], location[1]])
    ## Assigner paramaters
    info_r = rospy.get_param("infogain_radius")
    h_gain = rospy.get_param("h_gain")#2
    h_rad = rospy.get_param("h_radius")#0.1
    lbda = rospy.get_param("infogain_weight")
    weight = rospy.get_param("cost_weight")

    rate = rospy.Rate(rospy.get_param("task_node_rate"))
    rate1 = rospy.Rate(1)
    explored_points = []
    counter = 1
    rospy.sleep(rospy.get_param("wait_time"))
    #ospy.loginfo("sjofghweiuwgiluwbggw")
    while not rospy.is_shutdown():

        front = copy(frontiers)
        mapi = copy(gmap)
        front = np.reshape(np.array(front), (len(front),3))
        

        if len(front) > 1: 
            
            #rospy.loginfo("inside while")
            location = droneLocation(tfBuffer, 'taskAssigner')

            # Filtering more points
            front = np.unique(front, axis = 0)
            n = len(front)
            n_prev = n

            if n >= 31:
                dist = np.zeros(n)
                location = droneLocation(tfBuffer, 'taskAssigner')
                for f in range(0,n):
                    dist[f] = int(np.linalg.norm([location[0]-front[f][0],location[1]-front[f][1]]))

                dn = n - 30 
                ind = p = (-dist).argsort()[:dn]
                front = np.delete(front, ind,axis=0)

            
            del_ind = []
            n_prev = len(front)

            if explored_points:
                for pt in range(0, len(front)):
                    for p in explored_points:
                        if np.linalg.norm([p[0]-front[pt][0],p[1]-front[pt][1]]) == 0:
                            del_ind.append(pt)
                front = np.delete(front, del_ind,axis=0)

            if len(front) == 0:
                counter = counter + 1 
                continue

            cent_pub.publish(displaypathNodes(front,[1,0,0]))


            infogains = np.zeros(len(front))
            for i in range(0,len(front)):
                infogains[i] = infoGain(mapi, [front[i][0], front[i][1]], info_r)

            location = droneLocation(tfBuffer, 'taskAssigner')
            if not robot_busy:
                revs = np.zeros(len(front))
                for j in range(0,len(front)):
                    d2r = np.linalg.norm([location[0]-front[j][0],location[1]-front[j][1]])
                    cost = d2r

                    if d2r <= h_rad:
                        infogains[j] = infogains[j]*h_gain

                    revs[j] = lbda*infogains[j] - cost*weight
                    
                    if rospy.is_shutdown():
                        exit()
                
                max_rev_idx = np.argmax(revs)
                ap = np.array(front[max_rev_idx])
                ap_p = ap 
                rospy.loginfo("ap: %s", ap)
                task_pub.publish(pub_msg(ap))
                taskpt.publish(displaypathNodes([ap], [0,0,1], 0.08))
                explored_points.append(ap)

                frontiers *= 0

        rate.sleep()



if __name__ == '__main__':
    try:
        print("Running....")
        main()
        print("Closed")
    except rospy.ROSInterruptException:
        pass
