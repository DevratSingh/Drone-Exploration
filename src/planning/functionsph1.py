#!/usr/bin/env python
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
import rospy
import json
import tf
import tf2_ros
import sys
import numpy as np
from math import cos, sin, atan2, fabs, hypot, floor, pi, sqrt, trunc
import matplotlib.pyplot as plt 
import random as rnd
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from crazyflie_driver.msg import Position
from geometry_msgs.msg import TransformStamped, Vector3
from map_msgs.msg import OccupancyGridUpdate
 

#sys.path.append('/home/d/e/devrat/dd2419_ws/src/DD2419_proj/src/planning')
path = rospy.get_param("filepath")
sys.path.append(path)

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

def displaypathNodes(nodes, color,size = 0.06, shape = 2):
    markers = []
    markerArray_msg = MarkerArray()

    for pt in nodes:
        node = Pose()
        try:
            node.position.x = pt.x
            node.position.y = pt.y
        
            quat = quaternion_from_euler(0, 0, pt.yaw)
        except:
            node.position.x = pt[0]
            node.position.y = pt[1]
        
            quat = quaternion_from_euler(0, 0, 0)

        node.orientation.x = quat[0] 
        node.orientation.y = quat[1] 
        node.orientation.z = quat[2] 
        node.orientation.w = quat[3]

        markers.append(node)

    for i in range(len(markers)):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = shape
        marker.id = (i+1)
        #marker.action = 1
        marker.pose = markers[i]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.frame_locked = False
        marker.ns = "TreeNode"
        
        markerArray_msg.markers.append(marker)

    return markerArray_msg

def createptArray(points):
    msg = PoseArray()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()

    for pt in points:
        node = Pose()
        node.position.x = pt[0]
        node.position.y = pt[1]

        quat = quaternion_from_euler(0, 0, 0)

        node.orientation.x = quat[0] 
        node.orientation.y = quat[1] 
        node.orientation.z = quat[2] 
        node.orientation.w = quat[3]

        msg.poses.append(node)

    return msg

def dist_angle(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]

    dist = hypot(dx,dy)
    angle = atan2(dy,dx)
    
    return dist, angle

def indextoPoint(gridinfo, index):
    res = gridinfo.resolution
    x_origin = gridinfo.origin.position.x
    y_origin = gridinfo.origin.position.y
    width = gridinfo.width


    i, j = divmod(int(index), width)
    
    return indextocoord(gridinfo ,i-1, j-1)

def pointtoIndex(gridinfo, point):
    res = gridinfo.resolution
    x_origin = gridinfo.origin.position.x
    y_origin = gridinfo.origin.position.y
    width = gridinfo.width


    x = point[0]
    y = point[1]

    j = int((abs(y_origin) + x)/res)
    i = int((abs(x_origin) + y)/res)
    
    k = ((i+1)*width + j+1)
    return k

def indextocoord(gridinfo, i, j):

    res = gridinfo.resolution
    x_origin = gridinfo.origin.position.x
    y_origin = gridinfo.origin.position.y
    width = gridinfo.width

    
    x = j*res - abs(y_origin)
    y = i*res -abs(x_origin)
    
    return np.array([x, y]) 

def pointtomatIndex(gridinfo, point):
    res = gridinfo.resolution
    x_origin = gridinfo.origin.position.x
    y_origin = gridinfo.origin.position.y
    width = gridinfo.width


    x = round(point[0],4)
    y = round(point[1],4)

    #res = gridmap.info.resolution
    #x_origin = gridmap.info.origin.position.x
    #y_origin = gridmap.info.origin.position.y
    #width = gridmap.info.width

    j = int((abs(y_origin) + x)/res)
    i = int((abs(x_origin) + y)/res)
    
    return i,j


def infoGain(gridmap, point, rad):

    ig = 0
    #s = []
    #e = []
    #pts = []
    width = gridmap.info.width
    res = gridmap.info.resolution
    data = gridmap.data
    
    info_r = int(rad/res)
    idx = pointtoIndex(gridmap.info, point)
    box_idx = idx - (info_r*(width +1))
    
    for i in range(0, 2*info_r+1):
        st_idx = i*width + box_idx
        ed_idx = st_idx + 2*info_r

        #s.append(st_idx)
        #e.append(ed_idx)

        lim = ((st_idx/width)+2)*width
        for k in range(st_idx, ed_idx+1):
            if (k >= 0 and k < lim and k < len(data)):
                pt = indextoPoint(gridmap.info, k)
                #pts.append(pt)
                if(data[i] == -1 and np.linalg.norm(np.array(point) - pt) <= rad):
                    ig = ig + 1
    ig = ig*(res**2)
    return ig

def droneLocation(tfBuffer, node_name):
    ## Get robot location form TF tree
    while not rospy.is_shutdown():
        try:
            cf = tfBuffer.lookup_transform('map','cf1/base_link', rospy.Time(0), rospy.Duration(1))
            _, _, yaw = euler_from_quaternion((cf.transform.rotation.x,
                                        cf.transform.rotation.y,
                                        cf.transform.rotation.z,
                                        cf.transform.rotation.w))
            location = [cf.transform.translation.x, cf.transform.translation.y, yaw]
            return location

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("[%s]: No TF found, trying again", node_name)


def ROI(gridinfo, point, w):
    res = gridinfo.resolution
    x_origin = gridinfo.origin.position.x
    y_origin = gridinfo.origin.position.y
    width = gridinfo.width


    point = [round(point[0], 3), round(point[1], 3), round(point[2], 3)]
    ll_corner = [round((point[0]-(w*0.5)),4), round((point[1]-(w*0.5)),4)]
    ur_corner = [round((point[0]+(w*0.5)),4), round((point[1]+(w*0.5)),4)]
    ul_corner = [round((point[0]-(w*0.5)),4), round((point[1]+(w*0.5)),4)]
    
    #index = int(pointtoIndex(gridmap, [point[0], point[1]]))
    index_ll  = pointtomatIndex(ll_corner)
    index_ur = pointtomatIndex(ur_corner)

    index_ll_k  = int(pointtoIndex(ll_corner))
    index_ur_k = int(pointtoIndex(ur_corner))

    return ll_corner, ur_corner, index_ll, index_ur, index_ll_k, index_ur_k 

def grid_update(gridinfo, gridheader, point, w):

    res = gridinfo.resolution
    x_origin = gridinfo.origin.position.x
    y_origin = gridinfo.origin.position.y
    width = gridinfo.width
    f_id = gridinfo.frame_id

    point = [round(point[0], 4), round(point[1], 4), round(point[2], 4)]
    ll_corner, ur_corner, index_ll, index_ur, k_min, k_max = ROI(point, w)
    #k_min = int((index_ll[0]+1)*w + index_ll[1]+1)
    #k_max = int((index_ur[0]+1)*w + index_ur[1]+1)
    #point = [0,0]
    #rospy.loginfo("%i,%i,%i", k_min, k_max, pointtoIndex(point))
    #rospy.loginfo("%s,%s,%s",  ll_corner, ur_corner, point)#indextoPoint(1), indextoPoint(k_max), indextoPoint(pointtoIndex(point)))

    
    update =  OccupancyGridUpdate()
    update.header.stamp = rospy.Time.now()
    update.header.frame_id = f_id

    update.x = index_ll[1]
    update.y = index_ll[0]

    update.width = ((index_ur[0] - index_ll[0])+ 1)
    update.height = update.width
    #update.data = 2*np.ones(update.width**2)
    
    return update, k_min, k_max








