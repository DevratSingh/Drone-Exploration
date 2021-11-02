#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import json
import tf2_ros
import sys
import numpy as np
from math import cos, sin, atan2, fabs, hypot, floor, pi
import matplotlib.pyplot as plt 
import random as rnd
from RRT_map import Map
from RRT_3Dph1 import Planner
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from crazyflie_driver.msg import Position
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from copy import copy


mapgoal = None
gmap = OccupancyGrid()

def map_callback(msg):
    global gmap
    gmap = msg

def goal_callback(msg):
    global mapgoal

    # RViz's "2D Nav Goal" publishes z=0, so add some altitude if needed.
    if msg.pose.position.z < 0.0:
        msg.pose.position.z = 0.0

    if msg.pose.position.z == 0.0:
        msg.pose.position.z = 0.7
    
    _, _, yaw = euler_from_quaternion((msg.pose.orientation.x,
                                              msg.pose.orientation.y,
                                              msg.pose.orientation.z,
                                              msg.pose.orientation.w))
    #rospy.loginfo('New goal set:\n%s', msg)
    mapgoal = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, yaw]
    #rospy.loginfo(mapgoal)

def createPathmsg(path):
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = rospy.Time.now()

    for pt in path:
        node = PoseStamped()
        node.header.frame_id  = path_msg.header.frame_id
        node.header.stamp = path_msg.header.stamp
        node.pose.position.x = pt.x
        node.pose.position.y = pt.y
        node.pose.position.z = pt.z
        
        quat = quaternion_from_euler(0, 0, pt.yaw)

        node.pose.orientation.x = quat[0] 
        node.pose.orientation.y = quat[1] 
        node.pose.orientation.z = quat[2] 
        node.pose.orientation.w = quat[3]

        path_msg.poses.append(node)
    
    #rospy.loginfo("Message created \n")
    
    return path_msg

def displaypathNodes(nodes, it):
    markers = []
    markerArray_msg = MarkerArray()

    for pt in nodes:
        node = Pose()
        node.position.x = pt.x #pt[0] #pt.x
        node.position.y = pt.y #pt[1] #pt.y
        node.position.z = pt.z #pt[2]
        if it == 0:
            pt.yaw = 0

        quat = quaternion_from_euler(0, 0, pt.yaw)#pt[3]) #pt.yaw)

        node.orientation.x = quat[0] 
        node.orientation.y = quat[1] 
        node.orientation.z = quat[2] 
        node.orientation.w = quat[3]

        markers.append(node)

        if it == 0:
            node = Pose()
            node.position.x = pt.x #pt[0] #pt.x
            node.position.y = pt.y #pt[1] #pt.y
            node.position.z = pt.zmin #pt[2]
            if it == 0:
                pt.yaw = 0

            quat = quaternion_from_euler(0, 0, pt.yaw)#pt[3]) #pt.yaw)

            node.orientation.x = quat[0] 
            node.orientation.y = quat[1] 
            node.orientation.z = quat[2] 
            node.orientation.w = quat[3]

            markers.append(node)

    for i in range(len(markers)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = 2
        marker.id = (i+1)
        #marker.action = 1
        marker.pose = markers[i]
        if it == 0:
            marker.color.r = 0.23
            marker.color.g = 0.37
            marker.color.b = 0.8
        elif it ==2:
            marker.color.r = 0.4
            marker.color.g = 0.9
            marker.color.b = 0.93
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        if it==2:
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04

        marker.frame_locked = False
        marker.ns = "TreeNode"
        
        markerArray_msg.markers.append(marker)

    return markerArray_msg

def getrobotPose(cf):
    #rospy.loginfo("CrazyFly Pose Recieved")
    #rospy.loginfo(cf.transform.translation)
    _, _, yaw = euler_from_quaternion((cf.transform.rotation.x,
                                        cf.transform.rotation.y,
                                        cf.transform.rotation.z,
                                        cf.transform.rotation.w))
    if cf.transform.translation.z < 0.0:
        z = 0.0
    else:
        z = cf.transform.translation.z

    start = [cf.transform.translation.x, cf.transform.translation.y, z, yaw]
    
    return start


def main(argv=sys.argv):
    global mapgoal, gmap
    path_pub = rospy.Publisher('RRTpath', Path, queue_size=10)
    #path1_pub = rospy.Publisher('RRTpath1', Path, queue_size=10)
    marker_pub = rospy.Publisher('RRTnodes', MarkerArray, queue_size=10)
    marker1_pub = rospy.Publisher('obstacles', MarkerArray, queue_size=10)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    sub3 = rospy.Subscriber('gridmap', OccupancyGrid, map_callback)


    rospy.init_node('pathNode', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    args = rospy.myargv(argv=argv)

    inflateradius = 0.2
    res = 0.1
    mapping = Map('map', res, inflateradius, args)
    obstacles, ext = mapping.getobstacleList()

    #mapgoal = [-1, 1, 0.5, pi/3]
    #mapgoal = [obstacles[1].x, obstacles[1].y, 0, 0]
    #start = [0.5, -0.5, 0, 0]

    rate = rospy.Rate(10)
    prevgoal = [-5,-5, 0]

    markerArray1_msg = displaypathNodes(obstacles, 0)

    cnt = 1
    while len(gmap.data) < 1:
        if cnt%150 == 0: rospy.loginfo("[Filter]: Waiting for map")
        pass
    mapi = copy(gmap)
    while not rospy.is_shutdown():
        
        try:
            cfpose = tfBuffer.lookup_transform('map','cf1/base_link', rospy.Time(0))
            start = getrobotPose(cfpose)
            #poseReceived = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        if mapgoal is not None:
            if mapgoal != prevgoal:
                print("RRT")
                print(" ")
                print("Start", start)
                print("Goal", mapgoal)
                RRT = Planner(start, mapgoal, 500, res, inflateradius, obstacles, ext, args)
                path, nodes= RRT.rrt()
                path1 = RRT.pathsmoothing(path)
                markerArray_msg = displaypathNodes(path1, 1)
                path1_msg = createPathmsg(path1)
                path_pub.publish(path1_msg)
                #path1_pub.publish(path_msg)
                marker_pub.publish(markerArray_msg)
                marker1_pub.publish(markerArray1_msg)
                
            prevgoal = mapgoal
            
        if rospy.is_shutdown():
            rospy.loginfo("Breaking")
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Running....")
        main()
        print("Closed")
    except rospy.ROSInterruptException:
        pass

