#!/usr/bin/env python
from __future__ import print_function

from perception.detector import Detector
from PIL import Image as Img
import torchvision.transforms.functional as TF
import torch

import sys
import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from tf.transformations import quaternion_from_euler
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from aruco_msgs.msg import MarkerArray, Marker

import math

import time

import matplotlib.pyplot as plt

category_dict = {
  0: 'nobicycle',
  1: 'airport',
  2: 'dangerousleft',
  3: 'dangerousright',
  4: 'followleft',
  5: 'followright',
  6: 'junction',
  7: 'noheavytruck',
  8: 'noparking',
  9: 'nostoppingandparking',
  10:'residential',
  11:'narrowsfromleft',
  12:'narrowsfromright',
  13:'roundabout',
  14:'stop'
}

class image_converter:

  def __init__(self, device='cpu', res_directory='./'):
    # Create network
    self.detector = Detector().to(device) 

    self.res_directory = res_directory

    self.drawpoints = []

    # Load the weights
    self.detector.load_state_dict(torch.load(self.res_directory + 'squeezenet_ver14.pt', map_location=torch.device(device))) # Make sure to have correct path
    self.detector.eval()

    px = 313.121754
    py = 265.424811

    self.detected_pub = rospy.Publisher('/trafficsign/detected', MarkerArray, queue_size=10)

    # Features for ground truth signs
    self.kp = []
    self.des = []
    self.widths = []
    self.imgs = []
    sift = cv.xfeatures2d.SIFT_create()
    for i in range(15):
        #img1 = cv.imread(self.res_directory + category_dict[i]+'.png', cv.IMREAD_GRAYSCALE)
        img1 = cv.imread(self.res_directory + category_dict[i]+'.png', cv.IMREAD_COLOR)
        #img1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
        if img1 is None:
            print(self.res_directory + category_dict[i] + '.png not found')
            self.kp.append([])
            self.des.append([])
            self.widths.append(0)
            self.imgs.append([])
            continue
        kp1, des1 = sift.detectAndCompute(img1,None)
        self.imgs.append(img1)
        self.kp.append(kp1)
        self.des.append(des1)
        self.widths.append(img1.shape[0])

    # Create tf2 broadcaster
    self.broadcaster = tf2_ros.TransformBroadcaster()

    # Initialize publisher, subscriber
    self.image_pub = rospy.Publisher("/myresult", Image, queue_size=1)
    # Debug publisher
    self.debug_image_pub = rospy.Publisher("/myresult_debug", Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cf1/camera/image_raw", Image, self.callback, queue_size=1, buff_size = 2**28) # queue_size and buff_size to remove camera_lag

  def callback(self,data,device='cpu'):
    # Convert the image from OpenCV to ROS format
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image_grayscale = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    # Convert ROS message (Image) to compatible tensor
    # pass ROS message through the network
    with torch.no_grad():
        image = torch.stack([TF.to_tensor(Img.fromarray(cv_image))]).to(device)
        out = self.detector(image).cpu()

    #image = cv_image_grayscale

    # decode the bounding boxes
    bbs = self.detector.decode_output(out, 0.3)[0]

    # Add bounding boxes to image
    if bbs:
        markers_msg = MarkerArray()
        best_bb = -1
        best_conf = 0
        for bb in bbs:
            if best_bb == -1 or bb['confidence'] > best_conf:
                best_bb = bb
                best_conf = bb['confidence']

        x = int(best_bb['x'].item())
        y = int(best_bb['y'].item())
        w = int(best_bb['width'].item())
        h = int(best_bb['height'].item())

        cv.rectangle(cv_image, (x, y), (x+w, y+h), (0,0,255), 1)
        cv.putText(cv_image, category_dict[best_bb['category']], (x, y), 0, 1.5, (255, 0, 0), thickness=2)
        
        # Publish tf and add marker object
        outp = self.publish_sign(cv_image, best_bb, data.header)
        outp = None
        if outp:
            markers_msg.markers.append(outp)

        markers_msg.header.stamp = data.header.stamp
        markers_msg.header.frame_id = 'camera_link'
        if markers_msg.markers:
          self.detected_pub.publish(markers_msg)
    
    for point in self.drawpoints:
        #print('drawing...', point)
        cv_image = cv.circle(cv_image, (int(point[0]), int(point[1])), radius=10, thickness=3, color=(0,0,255))
    # Publish the image
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

  # Returns a aruco Markerpose
  def publish_sign(self, image, bb, header):
    frame_width = 640.0
    frame_height = 480.0

    print(bb)

    w = bb['width'].item()
    h = bb['height'].item()
    x = bb['x'].item()# + w / 2 - frame_width / 2
    y = bb['y'].item()# + h / 2 - frame_height / 2

    z = 0.5

    #center_x = x + width / 2
    #center_y = y + height / 2

    t = TransformStamped()
    t.header.frame_id = 'cf1/camera_link'
    t.header.stamp = header.stamp
    t.child_frame_id = 'trafficsign/sign{}'.format(bb['category'])

    res = self.get_transform(image, bb['category'], x, y, w, h)
    if res == []:
        return
    rvec, tvec = res

    t.transform.translation = Vector3(*tvec)
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(*rvec)

    print(t)
    self.broadcaster.sendTransform(t)

    marker = Marker()
    marker.header.stamp = header.stamp
    marker.header.frame_id = 'cf1/camera_link'
    marker.pose.pose.position.x = t.transform.translation.x
    marker.pose.pose.position.y = t.transform.translation.y
    marker.pose.pose.position.z = t.transform.translation.z

    marker.pose.pose.orientation.x = t.transform.rotation.x
    marker.pose.pose.orientation.y = t.transform.rotation.y
    marker.pose.pose.orientation.z = t.transform.rotation.w
    marker.pose.pose.orientation.w = t.transform.rotation.z

    marker.pose.covariance = [1]*36
    marker.id = bb['category']
    marker.confidence = bb['confidence']

    return marker

  def get_transform(self, img, sign, x, y, width, height):
      ref_filename = self.res_directory + category_dict[sign]+'.png'
      if width < 30 or height < 30 or x < 0 or y < 0:
          return []
      print(ref_filename)

      upscale_factor = 2
      #img2 = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
      img2 = img

      img2 = img2[int(y):int(y+height), int(x):int(x+width)]
      new_shape = (upscale_factor*img2.shape[1], upscale_factor*img2.shape[0])
      print(img2.shape)
      img2 = cv.resize(img2, new_shape)

      #print((x, y, width, height))
      markerwidth = 18.8 #width of all markers in cm (I think)
      cm_per_pix = markerwidth/self.widths[sign]
     
      # Initiate SIFT detector
      sift = cv.xfeatures2d.SIFT_create(nfeatures=1000)

      # find the keypoints and descriptors with SIFT
      kp1 = self.kp[sign]
      des1 = self.des[sign]
      kp2, des2 = sift.detectAndCompute(img2,None)
      print(width, height)

      # FLANN parameters
      bf = cv.BFMatcher()#normType=cv.NORM_L1)
      matches = bf.match(des1, des2)

      p1 = []
      p2 = []
      self.drawpoints = []
      for match in matches:
        if match.distance > 400: #Remove some of the bad matches
            continue
        #print(match.distance)
        imgpoint = kp2[match.trainIdx].pt
        imgpoint = (float(imgpoint[0])/upscale_factor+x, float(imgpoint[1])/upscale_factor+y)
        self.drawpoints.append(imgpoint)

        p1.append(list(kp1[match.queryIdx].pt) + [0])
        p2.append(imgpoint)

      # Publish debug view
      #self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(img2, "mono8"))
      self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(img2, "bgr8"))

      print('number of points: ', len(p1))
      if len(p1) < 3:
        return []

      #px = img2.shape[1] // 2
      #py = img2.shape[0] // 2
      px = 313.121754
      py = 265.424811

      camera_matrix = np.array(
              [[184.895559,   0.,        px],
       [  0.,        184.879499, py],
       [  0.,          0.,          1.        ]])
      distortion = np.array([-0.059981, 0.002146, 0.000905, -0.000644, 0.000000])
      
      try:
        retval, rvec, tvec, inliers = cv.solvePnPRansac(np.array(p1), np.array(p2), camera_matrix, distortion, iterationsCount=10000, reprojectionError=10.5, confidence=0.999)
        print('inliers:' + str(len(inliers)))
        tvec = tvec.T[0]

        # Rotate to be same rotation as aruco markers
        rvec[0] += math.pi / 2.0
        #rvec[1] += math.pi / 2.0
        rvec[2] -= math.pi / 2.0

        # Shift to center of sign (instead of top left corner)
        tvec[0] += (markerwidth/2.0)/cm_per_pix
        tvec[1] += (markerwidth/2.0)/cm_per_pix

        if (tvec*cm_per_pix)[2] > 150 or (tvec*cm_per_pix)[2] < 10:
          print('probabaly wrong...')
          print(tvec*cm_per_pix)
          return []
        return rvec, tvec*cm_per_pix/100.0
      except:
        print('failed')
        return []



def main(args):
  if len(args) < 2:
      sys.exit('please specify model filename')
  rospy.init_node('detect', anonymous=True)

  ic = image_converter(res_directory=args[1])

  print("running...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



