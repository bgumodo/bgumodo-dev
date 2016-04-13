#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Point.h"

#include "IK.h"

#define PRE_FINGER_OPEN     0.4
#define GRASPING_FINGER     0.1

std_msgs::String 												                   status;
ros::Publisher 													                   status_pub;
moveit::planning_interface::PlanningSceneInterface 				 *planning_scene_interface;
moveit::planning_interface::MoveGroup 							       *group;
moveit::planning_interface::MoveGroup::Plan 					     *my_plan;
ros::Publisher                                             left_finger_pub;
ros::Publisher                                             right_finger_pub;

void upperGraspingManipulator(const geometry_msgs::Point::ConstPtr& req)
{
  std_msgs::Float64 left_finger_value;
  std_msgs::Float64 right_finger_value;

  left_finger_value.data  = -PRE_FINGER_OPEN;
  right_finger_value.data = PRE_FINGER_OPEN;

  left_finger_pub.publish(left_finger_value);
  right_finger_pub.publish(right_finger_value);

  status.data = "Waiting";
  status_pub.publish(status);
  if(!(req->x == 0.0 && req->y == 0.0 && req->z == 0.0))
  {
  	  status.data = "In progress";
  	  status_pub.publish(status);
	  // desired pos goal
	  geometry_msgs::Pose desired_pose;

	  desired_pose.position.x = req->x;
	  desired_pose.position.y = req->y;
	  desired_pose.position.z = req->z;
	  desired_pose.orientation.w = 1.0;

	  // set the joints target
	  std::vector<double> group_variable_values = IK(desired_pose);

	  cout << group_variable_values[0] << endl;

	  group->setJointValueTarget(group_variable_values);

	  bool success = group->plan(*my_plan);

	  if (success) {
		  group->execute(*my_plan);
      left_finger_value.data  = GRASPING_FINGER;
      right_finger_value.data = -GRASPING_FINGER;
      left_finger_pub.publish(left_finger_value);
      right_finger_pub.publish(right_finger_value);
		  status.data = "Success";
		  status_pub.publish(status);
		  ros::Duration(3.0).sleep();
	  }
  }
  return;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "komodo_upper_grasping_manipulator");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_interface 		= new moveit::planning_interface::PlanningSceneInterface;
  group 						          = new moveit::planning_interface::MoveGroup("arm");
  my_plan 						        = new moveit::planning_interface::MoveGroup::Plan;

  left_finger_pub             = node_handle.advertise<std_msgs::Float64>("left_finger_controller/command", 100);
  right_finger_pub            = node_handle.advertise<std_msgs::Float64>("right_finger_controller/command", 100);

  status_pub 					        = node_handle.advertise<std_msgs::String>("side_grasping_manipulator/status", 100);
  ros::Subscriber sub 			  = node_handle.subscribe("side_grasping_manipulator/command", 1, upperGraspingManipulator);

  ros::Duration(1.0).sleep(); //time to construct ros subscribers and publishers

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

  ROS_INFO("Ready to get cup position...");
  
  ros::spin();
  return 0;
}
