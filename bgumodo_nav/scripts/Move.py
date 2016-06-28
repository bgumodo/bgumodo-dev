#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String


class Move:

    def __init__(self):
        rospy.init_node('move_to_goal', anonymous=True)

        self.move_cmd = rospy.Subscriber("/navigation/move_cmd", String, self.move_call_back)

        self.move_res = rospy.Publisher("/navigation/move_res", String, queue_size=5)

        self.locations = dict()

        self.locations["room_2"] = Pose(Point(0.848, -0.953, 0), Quaternion(0, 0, 0.704, 0.710))
        self.locations["door_zone_1"] = Pose(Point(-9, 3.3, 0), Quaternion(0, 0, 0.704, 0.710))
        self.locations["door_zone_2"] = Pose(Point(-9, 6, 0), Quaternion(0, 0, 0.704, 0.710))
        self.locations["room_4"] = Pose(Point(-4.51, 8.347, 0), Quaternion(0, 0, 0.704, 0.710))

    def move_call_back(self, data):
        loc = str(data.data)
        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")

        print "going to " + loc

        goal = MoveBaseGoal()

        goal.target_pose.pose = self.locations[loc]
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        move_base.send_goal(goal)

        move_base.wait_for_result(rospy.Duration(300))

        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            self.move_res.publish("Success")
        else:
            self.move_res.publish("Falied")


if __name__ == '__main__':

    Move()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down'


