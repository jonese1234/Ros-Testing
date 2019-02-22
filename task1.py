#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 22 11:25:43 2019

@author: student
"""

# Task 1

import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from numpy import mean
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

wheel_radius = 0.05 # 0.05
robot_radius = 0.25# 0.25

class task1():

    def __init__(self):
        self.node_name = "task1"
        print("Running")
        rospy.init_node(self.node_name)
        self.cv_window_name = self.node_name
        
        self.w_left = rospy.Subscriber("/wheel_vel_left", Float32, self.callbackRun)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.twist = Twist()
        
        # computing the forward kinematics for a differential drive
    def forward_kinematics(self, w_l, w_r):
        c_l = wheel_radius * w_l
        c_r = wheel_radius * w_r
        v = (c_l + c_r) / 2
        a = (c_r - c_l) / (2 * robot_radius)
        return (v, a)
    
    
    # computing the inverse kinematics for a differential drive
    def inverse_kinematics(self, v, a):
        c_l = v - (robot_radius * a)
        c_r = v + (robot_radius * a)
        w_l = c_l / wheel_radius
        w_r = c_r / wheel_radius
        return (w_l, w_r)
    
    
    # inverse kinematics from a Twist message (This is what a ROS robot has to do)
    def inverse_kinematics_from_twist(self,t):
        return self.inverse_kinematics(t.linear.x, t.angular.z)
        
    def callbackRun(self, data):        
        (v, a) = self.forward_kinematics(data.data, 0)
        print( "v = %f,\ta = %f" % (v, a))
    
        left_cmd = Twist()
        left_cmd.linear.x = v
        left_cmd.angular.z = a
       # left_cmd.a
        
        self.cmd_vel_pub.publish(left_cmd)

if __name__ == '__main__':
     task1()
     rospy.spin()
