#include <cmath>
#include "ros/ros.h" 
#include "ros/time.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#define PI 3.141592654

ros::Publisher 				pose_pub;
geometry_msgs::PoseStamped 	res;

void calculate(const geometry_msgs::Point::ConstPtr& req)
{
	double x, y, z, shoulder_th;

	x = req->x;
	y = req->y;
	z = req->z;

	if(!(x == 0 && y == 0 && z == 0))
	{
		shoulder_th 			= acos((z - 0.7075)/0.4903);
		res.pose.position.x 	= -(0.4903*sin(shoulder_th)+0.4469);
		res.pose.position.y 	= 0.00020598;

		res.header.frame_id = "/base_link";
		pose_pub.publish(res);
		ROS_INFO("X = %f Y = %f",res.pose.position.x,res.pose.position.y);
	}

	return;
}

void yaw_update(const geometry_msgs::PoseStamped::ConstPtr& current_base_pose)
{
	res.pose.orientation = current_base_pose->pose.orientation;
	ROS_INFO("Base orientation got update");

	return;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "top_grasping_base_pose_calc");
    ros::NodeHandle n;
   
   	pose_pub 				= n.advertise<geometry_msgs::PoseStamped>("top_grasping_base_pose_calc/desired_pose", 100);
	ros::Subscriber sub 	= n.subscribe("top_grasping_base_pose_calc/input_point", 10, calculate);
	ros::Subscriber yaw_sub = n.subscribe("base_pose_calc/desired_pose", 10, yaw_update); 

	ros::Duration(1.0).sleep(); //time to construct ros subscribers and publishers

    ROS_INFO("Ready to get cup point...");
    ros::spin();
	
	return 0;
} 
