#!/usr/bin/env python
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
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
from functionsph1 import createPathmsg, displaypathNodes

tfBuffer = None
listener = None


class GlobalFrontier(Planner):
    class Node:
        def __init__(self, x, y, yaw):
            self.x = x
            self.y = y
            self.yaw = yaw
            self.path_x = []
            self.path_y = []
            self.path_yaw = []
            self.parent = None

    def __init__(self, start, goal, iterations, res, inflateradius, obstacles, ext, args, map_res = 0.01, radius = 40, maxdist = 0.2, path_res = 0.2, sample_rate = 5):
        

        self.point_pub = rospy.Publisher('globalfrontpt', PoseStamped, queue_size=10)
        self.frontpt_pub = rospy.Publisher('glfrontnodes', MarkerArray, queue_size=10)

        map_res = rospy.get_param("map_resolution")
        inflate_prob = rospy.get_param("inflate_value")
        inflate_radius = rospy.get_param('inflate_radius')/map_res

        self.rrt_2d = Planner(start, goal, iterations, res, inflateradius, obstacles, ext)
        self.grid_map = GridMap('map', map_res, args, inflate_radius, inflate_prob)
        self.grid, self.map_matrix, _, _  = self.grid_map.gridmap(None)

        self.initNode = self.Node(start[0], start[1], start[2])
        self.goalNode = self.Node(goal[0], goal[1], goal[2])
        self.path_res = path_res
        self.sample_rate = sample_rate 
        self.iterations = iterations
        self.maxdist = 0.5
        self.vertices = []
        self.edges = []
        self.path = []
        self.obstacles = obstacles
        self.ext = ext
        self.radius = int(rospy.get_param('explored_radius')/map_res)
        self.map_res = map_res
        self.plan_3d = rospy.get_param('plan_3D')
    

    def frontier(self):
        self.vertices = [self.initNode]
        self.edges *= 0
        displaynodes = []

        i = 1
        rate = rospy.Rate(rospy.get_param("node_rate"))
        while not rospy.is_shutdown():
            randomNode = self.rrt_2d.generaterandomNode()

            closestNode_idx = self.rrt_2d.findclosestNode(self.vertices, randomNode)
            closestNode = self.vertices[closestNode_idx]

            tempNode = self.steer(closestNode, randomNode, self.maxdist)

            if self.gridCheck(tempNode):
                #if true tempnode is considered a frontier point and published
                self.publishPoint(tempNode)
                displaynodes.append(tempNode)
                
                msg = displaypathNodes(displaynodes, [0,1,0])
                
                self.frontpt_pub.publish(msg)
            
            if self.plan_3d:
                self.vertices.append(tempNode)
                edge, _ = self.rrt_2d.linebetnodes(closestNode, tempNode)
                self.edges.append(edge)
            else:   
                if self.rrt_2d.accidentCheck(tempNode):
                    # if no obstacle in path, add new point to vertices and add edge between xclosest and xnew
                    self.vertices.append(tempNode)
                    edge, _ = self.rrt_2d.linebetnodes(closestNode, tempNode)
                    self.edges.append(edge)
                
            rate.sleep()

    def generaterandincircle(self, center):
        p = rnd.random() * 2 * pi
        r = self.radius * self.map_res * sqrt(rnd.random())
        randx = center.x + cos(p) * r
        randy = center.y + sin(p) * r
        randyaw = rnd.uniform(-pi/2, pi/2)
        randNode = self.Node(randx, randy, randyaw)

        return randNode

    def markexplored(self, x, y):
        if self.grid_map.is_in_bounds(x,y):
            i, j = self.grid_map.pttoidx(x,y)
            for k in range(i - self.radius, i + self.radius):
                for l in range(j - self.radius, j + self.radius):
                    if ((i-k)**2 + (j-l)**2)**0.5 <= self.radius:
                        if not(self.map_matrix[k, l] == 100):                                   
                            self.map_matrix[k][l] = 30
            
        grid, _, _, _ = self.grid_map.gridmap(self.map_matrix)
        return grid

    def steer(self, fromNode, toNode, maxdist):
        tempNode = self.Node(fromNode.x, fromNode.y, fromNode.yaw)
        dist, angle = self.rrt_2d.linebetnodes(tempNode, toNode)

        tempNode.path_x = [tempNode.x]
        tempNode.path_y = [tempNode.y]
        tempNode.path_yaw = [tempNode.yaw]

        if maxdist > dist:
            maxdist = dist
        
        steps = int(floor(maxdist/self.map_res))

        for i in range(steps):
            tempNode.x = tempNode.x + self.map_res*cos(angle)
            tempNode.y = tempNode.y + self.map_res*sin(angle)
            
            tempNode.path_x.append(tempNode.x)
            tempNode.path_y.append(tempNode.y)
            tempNode.path_yaw.append(tempNode.yaw)
        
        dist, angle = self.rrt_2d.linebetnodes(tempNode, toNode)

        if dist <= self.path_res:
            tempNode.path_x.append(toNode.x)
            tempNode.path_y.append(toNode.y)
            tempNode.path_yaw.append(toNode.yaw)
            tempNode.x = toNode.x
            tempNode.y = toNode.y
            tempNode.yaw = toNode.yaw
        
        tempNode.parent = fromNode

        for x, y in zip(tempNode.path_x, tempNode.path_y):
            plt.plot(x, y, '.', color = 'brown')

        return tempNode
    
    def gridCheck(self, node):
        if node is None:
            return False

        for x, y in zip(node.path_x, node.path_y):
            if self.grid_map.is_in_bounds(x,y):
                i, j = self.grid_map.pttoidx(x,y)
                if self.map_matrix[i,j] == -1:
                    #rospy.loginfo("point belongs to unknown")
                    return True

        #rospy.loginfo("[Global]: No point in unknown found")
        return False

    def publishPoint(self, node):
        ## supposed to send the point to the filter node
        point = PoseStamped()
        point.header.frame_id  = 'map'
        point.header.stamp = rospy.Time.now()
        point.pose.position.x = node.x
        point.pose.position.y = node.y
        
        quat = quaternion_from_euler(0, 0, node.yaw)

        point.pose.orientation.x = quat[0] 
        point.pose.orientation.y = quat[1] 
        point.pose.orientation.z = quat[2] 
        point.pose.orientation.w = quat[3]

        
        self.point_pub.publish(point)


    def getLocation(self):
        ## Get robot location form TF tree
        try:
            cf = tfBuffer.lookup_transform('map','cf1/base_link', rospy.Time(0))
            _, _, yaw = euler_from_quaternion((cf.transform.rotation.x,
                                        cf.transform.rotation.y,
                                        cf.transform.rotation.z,
                                        cf.transform.rotation.w))
            location = self.Node(cf.transform.translation.x, cf.transform.translation.y, yaw)
            return location

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #rate.sleep()
            rospy.loginfo("[Global]: No TF found")
            return None

    def plotobstacles(self, goal, path):
        ax = plt.gca()

        for obs in self.obstacles:
            circle = plt.Circle((obs.x, obs.y), obs.size, color='r')
            plt.plot(obs.x, obs.y, '.', color = 'yellow')
            ax.add_patch(circle)

        for node in self.vertices:
            plt.plot(node.x, node.y, 's', color = 'green')
        
        plt.plot(self.initNode.x, self.initNode.y, 'X', color = 'lime')
        plt.plot(goal[0], goal[1], 'X', color = 'deeppink')


def main(argv=sys.argv):
    global mapgoal, tfBuffer, listener
    #path_pub = rospy.Publisher('RRTpath', Path, queue_size=10)
    #path1_pub = rospy.Publisher('RRTpath1', Path, queue_size=10)
    #marker_pub = rospy.Publisher('randNodes', MarkerArray, queue_size=10)
    #sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    
    rospy.init_node('globalfrontier', anonymous=True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    args = rospy.myargv(argv=argv)

    inflateradius = rospy.get_param("rrt_inflate_radius")
    res = 0.2
    mapping = Map('map', res, inflateradius, args)
    obstacles, ext = mapping.getobstacleList()

    

    #mapgoal = [-1, 1, pi/3]
    
    start = [0.5, 0, pi/3]
    mapgoal = [-0.5, 0.5, 0]
    prevgoal = [-5,-5, 0]
    path = []

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            cf = tfBuffer.lookup_transform('map','cf1/base_link', rospy.Time(0))
            _, _, yaw = euler_from_quaternion((cf.transform.rotation.x,
                                        cf.transform.rotation.y,
                                        cf.transform.rotation.z,
                                        cf.transform.rotation.w))
            start = [cf.transform.translation.x, cf.transform.translation.y, yaw]
            #rospy.loginfo("[Global]: breaking")
            break

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("[Global]: No TF found")
            rate.sleep()
            continue

    globalfront = GlobalFrontier(start, prevgoal, 100, res, inflateradius, obstacles, ext, args)
    globalfront.frontier()
    #localfront.plotobstacles(mapgoal, path)
    #plt.axis([-3, 3, -3, 3])
    #plt.show()
    #msg = displaypathNodes(rpts)
    #print(len(rpts))
    #for pt in rpts:
    #    print(round(pt.x,3), round(pt.y, 3))


    
    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    map_pub.publish(grid)
    #    #marker_pub.publish(msg)
    #    rate.sleep()
        
    


if __name__ == '__main__':
    try:
        print("Running....")
        main()
        print("Closed")
    except rospy.ROSInterruptException:
        pass

