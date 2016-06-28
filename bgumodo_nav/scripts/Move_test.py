#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

class Move_test:

	locations = []
	i = 0

	def __init__(self):

		rospy.init_node("move_test", anonymous=True)

		self.locations = ["room_2", "door_zone_1", "door_zone_2", "room_4"]
		self.i = 1
		self.pub = rospy.Publisher("/navigation/move_cmd", String, queue_size=5)
		self.sub = rospy.Subscriber("/navigation/move_res", String, self.move_call_back)

		self.move_call_back("door_zone_1")


	def move_call_back(self, cmd):
		move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		rospy.loginfo("Waiting for move_base action server...")

		# Wait 60 seconds for the action server to become available
		move_base.wait_for_server(rospy.Duration(60))

		rospy.loginfo("Connected to move base server")

		self.pub.publish(self.locations[self.i])

		self.i += 1
		if self.i == 4:
			self.i = 0

if __name__ == '__main__':

	Move_test()

	rospy.spin()
