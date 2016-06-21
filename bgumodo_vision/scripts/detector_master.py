#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from bgumodo_arm.msg import ThreePoints
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
        self.pubCupRes = rospy.Publisher('detector/observe_cup_res', ThreePoints, queue_size=5)
        self.pubButtonRes = rospy.Publisher('detector/observe_button_res', ThreePoints, queue_size=5)

    def observe_cup_callback(self, cmd):
        if (not cmd.data == "Start"):
            return

        rospy.loginfo("Got cup observe command")
        result = ThreePoints()
        
        
        cup = get_xy_client(2) # 1 is for detecting a table, 2 is for detecting a cup, 
        print ("cup", cup)
        if cup.x is not None:
            cup = get_distance_client(cup.x[0], cup.y[0])
            print('cup1',cup)
            result.object.x = cup.z + 0.031 - 0.34
            result.object.y = -cup.x 
            result.object.z = cup.y + 0.863 - 0.16
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
        rospy.Subscriber('detector/observe_cup_cmd', String, detector_obj.observe_cup_callback)
        rospy.Subscriber('detector/observe_button_cmd', String, detector_obj.observe_button_callback)


        rospy.spin()
    except rospy.ROSInterruptException:
        pass

