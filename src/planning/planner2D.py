#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import json
import tf2_ros
import sys
import numpy as np
from math import cos, sin, atan2, fabs, hypot, floor, pi, sqrt
import matplotlib.pyplot as plt 
import random as rnd
from RRT_map import Map
from RRT_2D import Planner
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from crazyflie_driver.msg import Position
from geometry_msgs.msg import TransformStamped, Vector3
from std_msgs.msg import Bool

from functionsph1 import*

mapgoal = None
robot_busy = False

def status_callback(msg):
    global robot_busy
    robot_busy = msg.data

def goal_callback(msg):
    global mapgoal

    # RViz's "2D Nav Goal" publishes z=0, so add some altitude if needed.
    if msg.pose.position.z == 0.0:
        msg.pose.position.z = 0.4
    
    _, _, yaw = euler_from_quaternion((msg.pose.orientation.x,
                                              msg.pose.orientation.y,
                                              msg.pose.orientation.z,
                                              msg.pose.orientation.w))
    #rospy.loginfo('New goal set:\n%s', msg)
    mapgoal = [msg.pose.position.x, msg.pose.position.y, yaw]
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
        
        quat = quaternion_from_euler(0, 0, pt.yaw)

        node.pose.orientation.x = quat[0] 
        node.pose.orientation.y = quat[1] 
        node.pose.orientation.z = quat[2] 
        node.pose.orientation.w = quat[3]

        path_msg.poses.append(node)
    
    #rospy.loginfo("Message created \n")
    
    return path_msg

def displaypathNodeslocal(nodes, size = 0.06):
    markers = []
    markerArray_msg = MarkerArray()

    for pt in nodes:
        node = Pose()
        node.position.x = pt.x
        node.position.y = pt.y
        
        quat = quaternion_from_euler(0, 0, pt.yaw)

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
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
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
    start = [cf.transform.translation.x, cf.transform.translation.y, yaw]
    
    return start

def main(argv=sys.argv):
    global mapgoal
    path_pub = rospy.Publisher('RRTpath', Path, queue_size=10)
    path1_pub = rospy.Publisher('patcbfe', Path, queue_size=10)
    marker_pub = rospy.Publisher('RRTnodes', MarkerArray, queue_size=10)
    #Mmarker_pub = rospy.Publisher('patsh', MarkerArray, queue_size=10)
    edpts_pub = rospy.Publisher('edpts', MarkerArray, queue_size=10)
    sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

    rospy.init_node('pathNode', anonymous=True)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    args = rospy.myargv(argv=argv)

    inflateradius = 0.25
    res = 0.2
    mapping = Map('map', res, inflateradius, args)
    obstacles, ext = mapping.getobstacleList()

    #mapgoal = [-1, 1, pi/3]
    
    #start = [0.1, 1.5, pi/3]
    rate = rospy.Rate(10)
    prevgoal = [-5,-5, 0]

    while not rospy.is_shutdown():
        
        try:
            cfpose = tfBuffer.lookup_transform('map','cf1/base_link', rospy.Time(0))
            start = getrobotPose(cfpose)
            #poseReceived = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        if not robot_busy:
            if mapgoal is not None:
                if mapgoal != prevgoal:
                    print("RRT")
                    print(" ")
                    print("Start", start)
                    print("Goal", mapgoal)
                    edpts_pub.publish(displaypathNodes([start, mapgoal], [1,1,0], 0.056, 1))
                
                    RRT = Planner(start, mapgoal, 500, res, inflateradius, obstacles, ext, args)
                    path, nodes = RRT.rrt()
                    #path1 = RRT.pathsmoothing(path)
                    path1 = RRT.pathsmoothing(path)
                    markerArray_msg = displaypathNodeslocal(path1)
                    path1_msg = createPathmsg(path1)
                    
                    #RRT.plotobstacles(mapgoal, path)
                    #plt.axis([-3, 3, -3, 3])
                    #plt.show()
                    path_pub.publish(path1_msg)
                    marker_pub.publish(markerArray_msg)
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

