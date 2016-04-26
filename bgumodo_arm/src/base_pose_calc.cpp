#include <cmath>
#include "ros/ros.h" 
#include "ros/time.h"
#include "bgumodo_arm/ThreePoints.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#define PI 3.141592654
//#define BACK_DIS 0.55



class BasePoseCalculator {

	public:
		ros::Publisher pose_pub;

		BasePoseCalculator(const ros::Publisher& pose_pub) {
			this->pose_pub = pose_pub;
		} 

		double back_dis_calc(double x, double y, double z)
		{
			double shoulder_th, back_dis=0.0;

			if(!(x == 0 && y == 0 && z == 0))
			{
				shoulder_th 			= acos((z - 0.7075)/0.4903);
				back_dis 				= (0.4903*sin(shoulder_th)+0.4469);
			}
			return back_dis;
		}

		void calculate(const bgumodo_arm::ThreePoints::ConstPtr& req)
		{
			ROS_INFO("Got ThreePoints...");
			ROS_INFO("X = %f Y = %f Yaw = %f",req->object.x,req->object.y,req->object.z);
			double vec_size, x_l, x_r, y_l, y_r, yaw, cup_x, cup_y, cup_z, back_dis;
			geometry_msgs::Point d_o;
			geometry_msgs::PoseStamped res;

			x_l 	= req->p_left.x;
			x_r 	= req->p_right.x;
			y_l 	= req->p_left.y;
			y_r 	= req->p_right.y;
			cup_x 	= req->object.x;
			cup_y 	= req->object.y;
			cup_z 	= req->object.z;

			back_dis = back_dis_calc(cup_x,cup_y,cup_z);

			if(!(x_l == 0 && x_r == 0 && y_l == 0 && y_r == 0))
			{
				vec_size = sqrt(pow((x_l-x_r),2)+pow((y_l-y_r),2));
				d_o.x = (y_l-y_r)/vec_size;
				d_o.y = -(x_l-x_r)/vec_size;
				d_o.z = 0;
				ROS_INFO("Direction vector: [%f, %f, %f]",d_o.x,d_o.y,d_o.z);
				res.pose.position.x = cup_x - back_dis*d_o.x;
				res.pose.position.y = cup_y - back_dis*d_o.y;
				res.pose.position.z = 0;

				yaw = atan2(d_o.y,d_o.x);

				res.pose.orientation.x = 0.0;
				res.pose.orientation.y = 0.0;
				res.pose.orientation.z = sin(yaw/2);
				res.pose.orientation.w = cos(yaw/2);
				ROS_INFO("Quaternion: [w=%f, x=%f, y=%f, z=%f]",res.pose.orientation.w,res.pose.orientation.x,res.pose.orientation.y,res.pose.orientation.z);
				res.header.frame_id = "/base_link";
				ros::Duration(1.0).sleep();
				pose_pub.publish(res);
				ROS_INFO("X = %f Y = %f Yaw = %f[deg] Back dist = %f",res.pose.position.x,res.pose.position.y,yaw*180/M_PI,back_dis);
			}

			return;
		}
};


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "base_pose_calc");
    ros::NodeHandle n;

   	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("base_pose_calc/desired_pose", 1000);

 	BasePoseCalculator base_calc(pose_pub);

	ros::Subscriber sub = n.subscribe("base_pose_calc/input_points", 1000, &BasePoseCalculator::calculate, &base_calc);

	ros::Duration(1.0).sleep(); //time to construct ros subscribers and publishers

    ROS_INFO("Ready to get ThreePoints...");

    // ros::Rate rate(10.0);
    // while (n.ok())
    // {
    // 	ros::spinOnce();
    // 	rate.sleep();
    // }

    ros::spin();
	
	return 0;
}
