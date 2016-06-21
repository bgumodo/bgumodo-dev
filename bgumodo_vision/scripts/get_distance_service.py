#!/usr/bin/env python


import sys
import rospy
import cv2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from bgumodo_vision.srv import *
import numpy as np
import math
import pcl

depth_array = 0  # global

bridge = CvBridge()


def handle_get_distance(req):
    req.u = int(req.u)
    req.v = int(req.v)
    print ('handle distance ', req.u, req.v)
    d = depth_array.item(req.u, req.v)
    print ('d', d)

    # for debugging
    # cv2.normalize(depth_array, depth_array,1,0,cv2.NORM_MINMAX) # normalize just for showing the image
    # cv2.namedWindow( "DEPTH", cv2.WINDOW_NORMAL )
    # cv2.rectangle(depth_array,(req.u-20,req.v-20),(req.u+20,req.v+20),(255,255,255),3)
    # cv2.imshow('DEPTH',depth_array)
    # cv2.waitKey(5)


    # x, y, z=unproject(1920, 1080, 525, req.u, req.v, d)
    # return GetDistanceResponse(x, y, z, d)


def unproject(width, height, focal_length, u_in, v_in, d_in):
    center_w = width / 2
    center_h = height / 2

    dirX = u_in - center_w
    dirY = v_in - center_h
    dirZ = focal_length
    fLength = math.sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ)
    dirX /= fLength
    dirY /= fLength
    dirZ /= fLength

    x_out = (u_in - center_w) * d_in / focal_length
    y_out = (v_in - center_h) * d_in / focal_length
    # in case we get the Z value from the camera and not the d value. if we are in the center of the image it should be ~equal
    d_Aux = d_in / dirZ
    x_out = dirX * d_Aux
    y_out = -dirY * d_Aux
    z_out = dirZ * d_in
    return x_out, y_out, z_out


def depth_cb(data):
    for x in range(0, 1920):
        for y in range(0, 1080):
            if data.fields[x + y * 1920] != '\x00':
                print ("dist", x, y, data.data[x + y * 1920])
                # global depth_array
                # try:
                # cv_image_depth = bridge.imgmsg_to_cv2(data)
                # except CvBridgeError as e:
                # print(e)

                # depth_array = np.array(cv_image_depth, dtype=np.float32)
            # for debugging
            # cv2.normalize(cv_image_depth, cv_image_depth,1,0,cv2.NORM_MINMAX) # normalize just for showing the image
            # cv2.namedWindow( "DEPTH", cv2.WINDOW_NORMAL )
            # cv2.imshow('DEPTH',cv_image_depth)
            # cv2.waitKey(5)


def get_distance_server():
    rospy.init_node('get_distance_server')
    # rospy.Subscriber("/kinect2/hd/image_depth_rect",Image,depth_cb)
    rospy.Subscriber("/kinect2/hd/points", PointCloud2, depth_cb)
    s = rospy.Service('get_distance', GetDistance, handle_get_distance)
    print "Ready distance"
    rospy.spin()


if __name__ == "__main__":
    get_distance_server()
