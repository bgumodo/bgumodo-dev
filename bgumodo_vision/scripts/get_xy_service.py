#!/usr/bin/env python

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from bgumodo_vision.srv import *
import numpy as np
import math

image_array=0 # global

bridge = CvBridge()

#service callback
def handle_get_xy(req):
  print (req.obj_type)
  x,y=detect_obj(req.obj_type)
  print('handle_get_xy', x,y)
  return GetXYResponse(x,y)


def detect_obj(obj_type):

  MAX_NUM_OBJECTS=75
  MIN_OBJECT_AREA=15*15
  
  hsv_image = cv2.cvtColor( image_array, cv2.COLOR_BGR2HSV );

  if obj_type==1:   #Table
    hue_image  = cv2.inRange( hsv_image, (50, 50, 50), (80, 255, 255) ) # green

    cv2.namedWindow( "green_hue_image", cv2.WINDOW_NORMAL )
    cv2.imshow( "green_hue_image", hue_image )

  if obj_type==2:   #Cup
    red_hue_range_lower = cv2.inRange( hsv_image, (0, 100, 100), (10, 255, 255) ) # red
    red_hue_range_upper = cv2.inRange( hsv_image, (170, 100, 100), (179, 255, 255) )
    hue_image = cv2.addWeighted( red_hue_range_lower, 1.0, red_hue_range_upper, 1.0, 0.0 ); # red is on both sides of the hsv circle
    cv2.namedWindow( "red_hue_image", cv2.WINDOW_NORMAL )
    cv2.imshow( "red_hue_image", hue_image )


  debug=False

  if True == debug:
    cv2.namedWindow( "hue_image", cv2.WINDOW_NORMAL )
    cv2.imshow( "hue_image", hue_image )
    cv2.waitKey(2)

  hue_range_blured = cv2.GaussianBlur(hue_image, (9, 9), 2, 2)
  
  if True == debug:
    cv2.namedWindow( "mask_Morph", cv2.WINDOW_NORMAL )
    cv2.imshow("mask_Morph", hue_range_blured )
    cv2.waitKey(5)

  [cx,cy]= contoursOP(hue_range_blured,obj_type,image_array) # the cup point
  return cx, cy


# contoursOP finds contours in an image and shows it on the screen.
# Type 1 is for ellipse shapes (cup...), type 2 is for rectangles.
def contoursOP(img,obj_type,src):
  contours=[]
  ret,thresh = cv2.threshold(img ,127,255,0)
  cv2.namedWindow( "thresh", cv2.WINDOW_NORMAL )
  cv2.imshow('thresh',thresh)
  cv2.waitKey(10)
  contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
  cnts                = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  cx=[-1,-1];cy=[-1,-1]
  cv2.namedWindow( "src", cv2.WINDOW_NORMAL )
  cv2.imshow('src',src)
  cv2.waitKey(10)
  if contours:
    numObjects = len(cnts)
    MAX_NUM_OBJECTS=20; MIN_OBJECT_AREA=10*10; areaArray=[]
    contours = sorted(contours, key=cv2.contourArea,reverse=True)[:numObjects]

    if numObjects<MAX_NUM_OBJECTS:
      stop=len(contours)
      #print "stop: ", stop
      if stop>2: stop=2
      #for i in range(len(contours)):
      for i in range(0,stop):
      # approximate the contour
        #print "i: ", i
        M = cv2.moments(contours[i])
        area = cv2.contourArea(contours[i])
        if area>MIN_OBJECT_AREA:
          if (stop>1):
            x,y,w,h = cv2.boundingRect(contours[i])
            cx[i]=x
            cy[i]=y
          else:
            (x,y),radius = cv2.minEnclosingCircle(contours[i])
            cx[i] = int(M['m10']/M['m00'])
            cy[i] = int(M['m01']/M['m00'])
          ellipse = cv2.fitEllipse(contours[i])
          cv2.ellipse(src,ellipse,(0,255,0),2)
          cv2.imshow('src',src)
          cv2.waitKey(5)

  return cx, cy

def nothing(_): pass  


def callback(data):
  global image_array

  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  image_array = cv_image
  #x,y=detect_obj(2)
  #print('handle_get_xy', x,y)

def get_xy_server():
  
  rospy.init_node('get_xy_server')
  #rospy.Subscriber("/kinect2/hd/image_color",Image,callback)
  rospy.Subscriber("depth_test/output_video",Image,callback)


  s = rospy.Service('get_xy', GetXY, handle_get_xy)
  print "Ready xy"
  rospy.spin()
	
if __name__ == "__main__":
  #create_trackbars()
  get_xy_server()
