#!/usr/bin/env python
from codecs import lookup
import json
from logging import shutdown
from tf.transformations import euler_from_quaternion
import tf
import sys
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from crazyflie_driver.msg import Position, Hover
import rospy
import numpy as np
import tf2_ros
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from nav_msgs.msg import OccupancyGrid
from itertools import cycle
from copy import copy


#path = rospy.get_param("filepath")
#sys.path.append(path)

#from functionsph1 import*


global locflag

locflag = False
lastpoint = None
loc = True
stop = False
wp_reach_dist = rospy.get_param("wp_reach_dist")

gmap = OccupancyGrid()

def loc_callback(msg):
    global loc
    loc = msg.data

def map_callback(msg):
    global gmap
    gmap = msg

def marker_callback(msg):
    global locflag
    locflag = True

def hover_cmd(rot):
    hov = Hover()
    hov.header.stamp = rospy.Time.now()
    hov.header.frame_id = 'cf1/odom'
    
    hov.vx = 0#0.1
    hov.vy = 0#0.1
    hov.yawrate = rot
    hov.zDistance = 0.4
    pub_hover.publish(hov)

def readjsonfile(argv=sys.argv):
    args = rospy.myargv(argv=argv)

    with open(args[1], 'rb') as f:
        world = json.load(f)
    
    markers = []
    ids = []
    for m in world["markers"]:
        markers.append([m['pose']['position']])
        #ids.append(m["id"])
    
    #for signs in world["roadsigns"]:
    #    markers.append(signs['pose']['position'])

    return markers, ids

def dist_angle(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]

    dist = hypot(dx,dy)
    angle = atan2(dy,dx)
    
    return dist, angle

def checkmarkervisiblity(angle):
    return True
    
def posestamped2position(pose):
    position = Position()

    position.x = pose.pose.position.x
    position.y = pose.pose.position.y
    
    position.z = pose.pose.position.z#0.4
    
    if position.z == 0.0:
        position.z = 0.4


    _, _, position.yaw = euler_from_quaternion((pose.pose.orientation.x,
                                              pose.pose.orientation.y,
                                              pose.pose.orientation.z,
                                              pose.pose.orientation.w))

    position.yaw = np.rad2deg(position.yaw)

    position.header = pose.header
    

    return position


def tfstamped2position(tf):

    position = Position()

    position.x = tf.transform.translation.x
    position.y = tf.transform.translation.y
    position.z = tf.transform.translation.z

    if position.z == 0.0:
        position.z = 0.4



    _, _, position.yaw = euler_from_quaternion((tf.transform.rotation.x,
                                              tf.transform.rotation.y,
                                              tf.transform.rotation.z,
                                              tf.transform.rotation.w))

    position.header = tf.header

    return position


def distance2wp(wp):
    if not tf_buf.lookup_transform('cf1/odom', 'cf1/base_link', rospy.Time(0), rospy.Duration(tf_timeout)):
        rospy.logwarn_throttle(
            5.0, 'No transform from cf1/base_link to map')
        return
    while not rospy.is_shutdown():
        try:
            current_pose = tf_buf.lookup_transform('map', 'cf1/base_link', rospy.Time(0), rospy.Duration(tf_timeout))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("[brain]:trying again")

    return np.linalg.norm([wp.pose.position.x - current_pose.transform.translation.x, wp.pose.position.y - current_pose.transform.translation.y])#, wp.pose.position.z - current_pose.transform.translation.z])


def path_callback(msg, d = wp_reach_dist):
    global waypoint, lastpoint, stop
    if msg.poses:
        try:
            lastpoint = msg.poses[0]
        except:
            lastpoint = None

        path_rev = reversed(msg.poses)
        length = len(msg.poses)
        
        i = 0
        for wp in path_rev: # for each waypoint in the path
            wp.header.frame_id = 'map' # not all waypoints has a frame_id, try to fix in planning script
            
            waypoint = wp # map frame
            """
            if loc == False:
                rospy.loginfo("Spinning")
                rate1 = rospy.Rate(10)
                while not loc:
                    stop = True
                    hover_cmd(40)
                    rate1.sleep()
                stop = False
            """
            while not distance2wp(waypoint) < d:
                rate.sleep()

            rospy.loginfo("[Brain]: Reached waypoint %i", i+1)
            i = i + 1
    else:
        rospy.loginfo("[Brain:] Path empty")


rospy.init_node('brain', anonymous=True)
sub_marker = rospy.Subscriber('/aruco/markers', MarkerArray, marker_callback, queue_size=1)
path_sub = rospy.Subscriber('RRTpath', Path, path_callback)
loc_sub = rospy.Subscriber('localized_flag', Bool, loc_callback)
sub3 = rospy.Subscriber('gridmap', OccupancyGrid, map_callback)

busy_pub = rospy.Publisher('robotStatus', Bool, queue_size=2)
pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
pub_hover = rospy.Publisher('/cf1/cmd_hover', Hover, queue_size=2)



tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)
tf_timeout = 1
cnt = 1

while len(gmap.data) < 1:
    if cnt%150 == 0: rospy.loginfo("[Filter]: Waiting for map")
    pass
rate = rospy.Rate(rospy.get_param("node_rate"))

markers, ids = readjsonfile()


def main():
    global waypoint, locflag, lastpoint, stop
    stop = False
    if not stop:
        # All waypoints are transformed to the cf1/odom frame
        if not tf_buf.lookup_transform('cf1/odom', 'cf1/base_link', rospy.Time(0), rospy.Duration(tf_timeout)):
            rospy.logwarn_throttle(
                5.0, 'No transform from cf1/base_link to cf1/base_link')

        # Create initial waypoint to make drone hover
        initial_pose = tf_buf.lookup_transform('cf1/odom', 'cf1/base_link', rospy.Time(0), rospy.Duration(tf_timeout))
        waypoint = PoseStamped()
        waypoint.header = initial_pose.header
        waypoint.pose.position = initial_pose.transform.translation
        waypoint.pose.orientation = initial_pose.transform.rotation
        waypoint.pose.position.z= 0.4

        status = Bool()
        reach_dist = rospy.get_param("reach_dist")
        flag = True
        while not rospy.is_shutdown():
            # need to check map -> odom tf here
            waypoint.header.stamp = rospy.Time.now()
            wp = posestamped2position(tf_buf.transform(waypoint, 'cf1/odom', rospy.Duration(tf_timeout)))
            if not stop:
                pub_cmd.publish(wp)
            
            if lastpoint is not None:
                if distance2wp(lastpoint) < reach_dist:
                    if flag: 
                        rospy.loginfo("[Brain]: Final point")
                        flag = False
                    busy_pub.publish(False)
                else:
                    busy_pub.publish(True)
                    flag = True
            else:
                busy_pub.publish(False)

            rate.sleep()
        

if __name__ == '__main__':
    try:
        print("Running....")
        main()
        print("Closed")
    except rospy.ROSInterruptException:
        pass
