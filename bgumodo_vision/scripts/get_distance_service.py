#!/usr/bin/env python


import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from BGUModo.srv import *
import numpy as np
import math

depth_array=0 # global

bridge = CvBridge()

def handle_get_distance(req):
    
	print (req.u,req.v)
	d=depth_array.item(req.u,req.v)
	x, y, z=unproject(640, 480, 525, req.u, req.v, d)

	return GetDistanceResponse(x, y, z, d)

def unproject(width, height, focal_length, u_in, v_in, d_in):
  center_w = width/2;
  center_h = height/2;

  dirX = u_in - center_w;
  dirY = v_in - center_h;
  dirZ = focal_length;
  fLength = math.sqrt(dirX*dirX+dirY*dirY+dirZ*dirZ);
  dirX /= fLength;
  dirY /= fLength;
  dirZ /= fLength;

  x_out = (u_in - center_w) * d_in / focal_length;
  y_out = (v_in - center_h) * d_in / focal_length;
  #in case we get the Z value from the camera and not the d value. if we are in the center of the image it should be ~equal
  d_Aux = d_in / dirZ;
  x_out = dirX*d_Aux;
  y_out = -dirY*d_Aux;
  z_out = dirZ*d_in;
  return (x_out, y_out, z_out)


def depth_cb(data):
	global depth_array
  	try:
  		cv_image_depth = bridge.imgmsg_to_cv2(data)
  	except CvBridgeError as e:
  		print(e)

	depth_array = np.array(cv_image_depth, dtype=np.float32)

	# for debugging
	#cv2.normalize(cv_image_depth, cv_image_depth,1,0,cv2.NORM_MINMAX) # normalize just for showing the image
	#cv2.imshow('DEPTH',cv_image_depth)
	#cv2.waitKey(5

def get_distance_server():
    rospy.init_node('get_distance_server')
    s = rospy.Service('get_distance', GetDistance, handle_get_distance)
    rospy.Subscriber("/komodo_1/Asus_Camera/depth_registered/image_raw",Image,depth_cb)
    print "Ready"
    rospy.spin()
	
if __name__ == "__main__":
    get_distance_server()