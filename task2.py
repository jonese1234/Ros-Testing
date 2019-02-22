#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Fri Feb 15 12:12:21 2019

@author: student
"""

import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import mean
import numpy as np
from geometry_msgs.msg import Twist

class colorContour():

    def __init__(self):

        self.node_name = "color_contour"
        rospy.init_node(self.node_name)

        self.cv_window_name = self.node_name
        self.bridge = CvBridge()

        self.pub = rospy.Publisher("/result_topic", String, queue_size=10)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        rospy.Timer(rospy.Duration(0.03), self.open_windows) # timer for displaying windows
        
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()

    def open_windows(self,event):
        try:
            cv2.namedWindow("Cammy", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Slice", cv2.WINDOW_NORMAL)
            
            cv2.imshow("Cammy",self.cam_view)
            cv2.imshow("Slice",self.processed_image)
    
            cv2.waitKey(3)
        except:
            pass
            
    def image_callback(self, data):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            self.cam_view = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e: # try and catch called from Cv2
            print e
            pass

        cam_view = np.array(self.cam_view, dtype = np.uint8)
        self.processed_image = self.color_slice(cam_view)

    def color_slice(self, cam_view):
        hsv = cv2.cvtColor(cam_view, cv2.COLOR_BGR2HSV)
    

        # define range of blue color in HSV
        lower_blue = np.array([30,30,30])
        upper_blue = np.array([100,255,255])
        
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        masked = cv2.bitwise_and(self.cam_view, self.cam_view, mask=mask)
        
        h, w, d = cam_view.shape
        M = cv2.moments(mask)
        print(M)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cam_view, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            print self.twist.angular.z

            self.cmd_vel_pub.publish(self.twist)
        #cv2.imshow("window", cam_view)
       # cv2.waitKey(3)
        
        return masked

"""
Example 1: Get the mean
print np.mean(hsv[:, :, 0]) #H?
        print np.mean(hsv[:, :, 1]) #S?
        print np.mean(hsv[:, :, 2]) #V?
print mean(hsv)
#-----------------------------------------
Example 2: Get the mean
# the shape gives you the dimensions
h = img3.shape[0]
w = img3.shape[1]
# loop over the image, pixel by pixel
count = 0
# a slow way to iterate over the pixels
for y in range(0, h):
    for x in range(0, w):
    
# threshold the pixel
        if img3[y, x] > 0:
            count += 1
print('count edge pixels: %d' % count)
"""
if __name__ == '__main__':
     cv2.startWindowThread()
     colorContour()
     rospy.spin()
