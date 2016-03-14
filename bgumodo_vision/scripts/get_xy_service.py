#!/usr/bin/env python

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from BGUModo.srv import *
import numpy as np
import math

image_array=0 # global

bridge = CvBridge()

# constants of upper and lower image NDI based on color
Red_1_min=226 ; Red_2_min=0 ; Red_3_min=0 ;Red_1_max=255 ; Red_2_max=30 ; Red_3_max=40
Green_1_min=93 ; Green_2_min=136 ; Green_3_min=95 ;Green_1_max=140 ; Green_2_max=158 ; Green_3_max=119

def handle_get_xy(req):
  print (req.obj_type)
  x,y=detect_obj(req.obj_type)
  return GetXYResponse(x,y)

# create trackbars for color changes
def create_trackbars():
  # create a blank image
  image = np.zeros((128, 255, 3), np.uint8)
  cv2.namedWindow('image')
  switch = '0 : OFF \n1 : ON'
  
  cv2.createTrackbar('NDI1_MIN', 'image', 93, 255, nothing)
  cv2.createTrackbar('NDI2_MIN', 'image', 136, 255, nothing)
  cv2.createTrackbar('NDI3_MIN', 'image', 95, 255, nothing)
  cv2.createTrackbar('NDI1_MAX', 'image', 140, 255, nothing)
  cv2.createTrackbar('NDI2_MAX', 'image', 158, 255, nothing)
  cv2.createTrackbar('NDI3_MAX', 'image', 119, 255, nothing)
  return


# contoursOP finds contours in an image and shows it on the screen.
# Type 1 is for ellipse shapes (cup...), type 2 is for rectangles.
def contoursOP(img,obj_type,src):
  contours=[]
  ret,thresh = cv2.threshold(img ,127,255,0)
  contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
  cnts = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  cx=[-1,-1];cy=[-1,-1]
  print "in contoursOP"

  cv2.imshow('src',src)
  cv2.waitKey(10)
  if contours:
    numObjects = len(cnts)
    MAX_NUM_OBJECTS=20; MIN_OBJECT_AREA=10*10; areaArray=[]
    contours = sorted(contours, key=cv2.contourArea,reverse=True)[:numObjects]

    if numObjects<MAX_NUM_OBJECTS:
      stop=len(contours)
      print "stop: ", stop
      if stop>2: stop=2
      #for i in range(len(contours)):
      for i in range(0,stop):
      # approximate the contour
        print "i: ", i
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

def morphOperations(img):
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17))
  closedThresh = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
  img=closedThresh
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
  eroded = cv2.erode(img, kernel)
  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (12,12))
  dilated = cv2.dilate(eroded, kernel)
  
  return dilated,closedThresh


def detect_obj(obj_type):

  NDI1_MIN = cv2.getTrackbarPos('NDI1_MIN', 'image')
  NDI2_MIN = cv2.getTrackbarPos('NDI2_MIN', 'image')
  NDI3_MIN = cv2.getTrackbarPos('NDI3_MIN', 'image')
  NDI1_MAX = cv2.getTrackbarPos('NDI1_MAX', 'image')
  NDI2_MAX = cv2.getTrackbarPos('NDI2_MAX', 'image')
  NDI3_MAX = cv2.getTrackbarPos('NDI3_MAX', 'image')
  # NDI manipulation
  src16=np.array(image_array, dtype=np.float32)
  b,g,r=cv2.split(src16)
  
  NDI1 = ((b-g)/(b+g)+1)*255/2
  NDI2 = ((g-r)/(g+r)+1)*255/2
  NDI3 = ((b-r)/(b+r)+1)*255/2

  NDI_Img= cv2.merge((NDI1,NDI2,NDI3))
  NDI_Img2=np.array(NDI_Img, dtype=np.uint8)
  #cv2.imshow('NDI_Img',NDI_Img2) # use for debugging

  # Threshold the HSV image to get only blue colors
  
  MAX_NUM_OBJECTS=75
  MIN_OBJECT_AREA=15*15
  if obj_type==1:  #Table 
    Green_mask = cv2.inRange( NDI_Img, (Green_1_min, Green_2_min,Green_3_min), (Green_1_max,Green_2_max,Green_3_max) )
    Green_mask_Morph,closedThresh = morphOperations(Green_mask)
    [cx,cy]= contoursOP(Green_mask_Morph,obj_type,image_array) # the table points
    cv2.imshow('Green_mask_Morph',Green_mask_Morph)
    cv2.waitKey(5)
  if obj_type==2:   #Cup
    #Red_mask = cv2.inRange( NDI_Img, (NDI1_MIN, NDI2_MIN, NDI3_MIN), (NDI1_MAX,NDI2_MAX,NDI3_MAX) )
    Red_mask = cv2.inRange( NDI_Img, (Red_1_min, Red_2_min,Red_3_min), (Red_1_max,Red_2_max,Red_3_max) )
    Red_mask_Morph,closedThresh  = morphOperations(Red_mask)
    [cx,cy]= contoursOP(Red_mask_Morph,obj_type,image_array) # the cup point
    cv2.imshow("Red_mask_Morph", Red_mask_Morph )
    cv2.waitKey(5)

  return cx, cy


def callback(data):
  global image_array

  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  image_array = cv_image

def get_xy_server():
  
  rospy.init_node('get_xy_server')
  rospy.Subscriber("/komodo_1/Asus_Camera/rgb/image_raw",Image,callback)

  s = rospy.Service('get_xy', GetXY, handle_get_xy)
  print "Ready"
  rospy.spin()
	
if __name__ == "__main__":
  #create_trackbars()
  get_xy_server()