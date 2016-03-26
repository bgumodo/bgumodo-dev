 #include <string>
#include <cmath>
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "control_msgs/JointControllerState.h"
#include "geometry_msgs/Point.h"
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/CollisionObject.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib_msgs/GoalStatus.h"

#define PI 						3.141592654
#define PRE_GRASPING_ELBOW2 	0.4
#define FINGER_CLOSE	 		0.4
#define POST_GRASPING		 	0.2
#define BASE_ROTATION 			0
#define SHOULDER 				1
#define ELBOW1 					2
#define ELBOW2 					3 			
#define WRIST 					4
#define LEFT_FINGER 			5
#define RIGHT_FINGER 			6


/*Prototypes*/
void jointSpaceCalc();
void setPrePushing();
void setPushing();
void setPostPushing();
void pushButtonManipulator(const geometry_msgs::Point::ConstPtr& button);


/*Global variables*/
double 															goal_x;
double 															goal_y;
double 															goal_z;
double 															goal_shoulder_th;
double 															goal_elbow2_th;
std_msgs::String 												status;
std::vector<double> 											group_variable_values;
ros::Publisher 													left_finger_pub;
ros::Publisher 													right_finger_pub;
ros::Publisher 													status_pub;
moveit::planning_interface::PlanningSceneInterface 				*scene;
moveit::planning_interface::MoveGroup 							*group;
moveit::planning_interface::MoveGroup::Plan 					*my_plan;


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "komodo_pushing_manipulator");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
  	spinner.start();

	scene 										= new moveit::planning_interface::PlanningSceneInterface;
	group 										= new moveit::planning_interface::MoveGroup("arm");
	my_plan 									= new moveit::planning_interface::MoveGroup::Plan;
	left_finger_pub 							= n.advertise<std_msgs::Float64>("left_finger_controller/command", 100);
	right_finger_pub 							= n.advertise<std_msgs::Float64>("right_finger_controller/command", 100);
	status_pub 									= n.advertise<std_msgs::String>("push_manipulator/status", 100);
	ros::Subscriber sub 						= n.subscribe("push_manipulator/command", 1, pushButtonManipulator);

	ros::Duration(1.0).sleep(); //time to construct ros subscribers and publishers

	ROS_INFO("Ready to get the button position...");

	ros::spin();
	return 0;
}

void pushButtonManipulator(const geometry_msgs::Point::ConstPtr& button)
{

	goal_x = button->x;
	goal_y = button->y;
	goal_z = button->z;
	status.data = "Waiting";
	status_pub.publish(status);

	if(!(goal_x==0.0 && goal_y==0.0 && goal_z==0.0))
	{
		status.data = "In progress";
		status_pub.publish(status);
		ROS_INFO("Goal x = %f Goal y = %f Goal z = %f",goal_x,goal_y,goal_z);
		if(!(0.65<=goal_z && goal_z<=1.2))
		{
			ROS_ERROR("Can't reach the button!");
			status.data = "Failed";
			status_pub.publish(status);
			return;
		}
		jointSpaceCalc();
		group->setMaxVelocityScalingFactor(0.5); //TODO check why isn't working
		group->getCurrentState()->copyJointGroupPositions(group->getCurrentState()->getRobotModel()->getJointModelGroup(group->getName()), group_variable_values);
		setPrePushing(); 																	//Go above the button and open gripper
		group->setJointValueTarget(group_variable_values);
		if (group->plan(*my_plan))
		{
			ROS_INFO("Moving arm to pre-pushing position");
			if(!group->execute(*my_plan))
			{
				ROS_ERROR("Problem to execute pre-pushing trajectory!");
				status.data = "Failed";
				status_pub.publish(status);
				return;
			}
		}
		else
		{
			ROS_ERROR("Problem to plan trajectory to pre-pushing position!");
			status.data = "Failed";
			status_pub.publish(status);
			return;
		}

		ros::Duration(1.0).sleep();

		setPushing(); 															
		group->setJointValueTarget(group_variable_values);
		if (group->plan(*my_plan))
		{
			ROS_INFO("Moving arm to pushing position stage 1");
			if(!group->execute(*my_plan))
			{
				ROS_ERROR("Problem to execute pushing position stage 1 trajectory!");
				status.data = "Failed";
				status_pub.publish(status);
				return;
			}
		}
		else
		{
			ROS_ERROR("Problem to plan trajectory to pushing position stage 1!");
			status.data = "Failed";
			status_pub.publish(status);
			return;
		}

		ros::Duration(1.0).sleep();

		setPostPushing();
		group->setJointValueTarget(group_variable_values);
		if (group->plan(*my_plan))
		{
			ROS_INFO("Moving arm to post-pushing position");
			if(!group->execute(*my_plan))
			{
				ROS_ERROR("Problem to execute post-pushing position trajectory!");
				status.data = "Failed";
				status_pub.publish(status);
				return;
			}
		}
		else
		{
			ROS_ERROR("Problem to plan trajectory to post-pushing position!");
			status.data = "Failed";
			status_pub.publish(status);
			return;
		}

		ros::Duration(1.0).sleep();


		ROS_INFO("Successfully push the button :)"); 
		status.data = "Success";
		status_pub.publish(status);
		ros::Duration(3.0).sleep();

		return;
	}
}

void jointSpaceCalc()
{ 
	goal_shoulder_th 	= acos((goal_z - 0.7075)/0.4903);
	goal_elbow2_th 		= PI/2 - goal_shoulder_th;
 	
	return;
}

void setPrePushing()
{
	std_msgs::Float64 left_finger_value;
	std_msgs::Float64 right_finger_value;
	left_finger_value.data 	= FINGER_CLOSE;
	right_finger_value.data = -FINGER_CLOSE;
	group_variable_values[BASE_ROTATION] 	= 0.0;										
	group_variable_values[SHOULDER] 		= goal_shoulder_th;						
	group_variable_values[ELBOW1] 			= 0.0;										
	group_variable_values[ELBOW2] 			= goal_elbow2_th - PRE_GRASPING_ELBOW2;	
	group_variable_values[WRIST] 			= 0.0; 									
	left_finger_pub.publish(left_finger_value);
	right_finger_pub.publish(right_finger_value);
	ROS_INFO("Pre-Grasping values has set");	


	return;
}

void setPushing()
{
	group_variable_values[ELBOW2] 			= goal_elbow2_th; 		
	ROS_INFO("Grasping first step values has set");	

	return;
}

void setPostPushing()
{
	group_variable_values[BASE_ROTATION] 	= 0.0;										
	group_variable_values[SHOULDER] 		= 0.0;						
	group_variable_values[ELBOW1] 			= -POST_GRASPING;										
	group_variable_values[ELBOW2] 			= POST_GRASPING;	
	group_variable_values[WRIST] 			= -POST_GRASPING; 									
	ROS_INFO("Post-Grasping values has set");

	return;
}
 

