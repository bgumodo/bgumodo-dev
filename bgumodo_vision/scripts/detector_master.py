#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse
import numpy as np
from BGUModo.srv import *


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


#def main(args):
# main detection body
cup=get_xy_client(2) # 1 is for detecting a table, 2 is for detecting a cup, 
cup=get_distance_client(cup.x[0], cup.y[0])
print "************************************"
print "cup points: "
print cup.x,cup.y,cup.z,cup.d
print ""

table=get_xy_client(1) # 1 is for detecting a table, 2 is for detecting a cup, 
table_left=get_distance_client(table.x[0], table.y[0])
print "************************************"
print "table left: "
# the data is stored in the following format: 
#                                               table_left.x,table_left.y,table_left.z,table_left.d
print table_left
print ""

table_right=get_distance_client(table.x[1], table.y[1])
print "************************************"
print "table right: "
print table_right
print ""
print "************************************"
