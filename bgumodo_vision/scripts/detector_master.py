#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import argparse
import numpy as np
from bgumodo_vision.srv import *


def get_distance_client(x, y):
    rospy.wait_for_service('get_distance')
    try:
        get_distance = rospy.ServiceProxy('get_distance', GetDistance)
        res = get_distance(x, y)
        return res # object containing- res.x, res.y, res.z, res,d
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
# ADD IF NO IMAGE YET, SERVICE RETURNS 0 OR SOMTHING AND THAM CLIENT WAIT A SEC AND ASK AGAIN -20 TIMES TIMEOUT 

def get_xy_client(obj_type):
    rospy.wait_for_service('get_xy')
    try:
        get_xy = rospy.ServiceProxy('get_xy', GetXY)
        res = get_xy(obj_type)
        return res # object containing- res.x, res.y, res.z, res,d
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
# ADD IF NO IMAGE YET, SERVICE RETURNS 0 OR SOMTHING AND THAM CLIENT WAIT A SEC AND ASK AGAIN -20 TIMES TIMEOUT 

class Detector():
    def __init__(self):
        self.pubCupRes = rospy.Publisher('detector/observe_red_res', PoseStamped, queue_size=10)

    def observe_cup_callback(self, cmd):
        if (not cmd.data == "Start"):
            return

        rospy.loginfo("Got cup observe command")
        result = PoseStamped()
        
        
        cup = get_xy_client(2) # 1 is for detecting a table, 2 is for detecting a cup, 
        print ("cup in image", cup)
        if cup is not None:
            cup = get_distance_client(cup.x[0], cup.y[0])
            print('cup in world',cup)
            result.header.frame_id='kinect2_depth_optical_frame'
            result.pose.position.x = cup.x
            result.pose.position.y = cup.y 
            result.pose.position.z = cup.z
            result.pose.orientation.x = 0.0
            result.pose.orientation.y = 0.0 
            result.pose.orientation.z = 0.0
	    result.pose.orientation.w = 1.0
        else:
            print 'cup is none'

        rospy.loginfo("Published cup result")
        self.pubCupRes.publish(result)


    def observe_button_callback(self, cmd):
        return

if __name__ == '__main__':
    try:
        detector_obj = Detector()
        rospy.init_node('detector_master', anonymous=False)
        rospy.Subscriber('detector/observe_red_cmd', String, detector_obj.observe_cup_callback)


        rospy.spin()
    except rospy.ROSInterruptException:
        pass

