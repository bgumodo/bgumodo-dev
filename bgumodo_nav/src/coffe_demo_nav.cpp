#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string>
using namespace std;

#define CV_PI 3.14

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher nav_pub;


class Location { 
public:
  string name;
  double x,y,z,R,P,Y; //change to float 64
  Location(string name_in, double x_in, double y_in,double z_in,double R_in,double P_in,double Y_in) {
	name=name_in;      
  	x=x_in;y=y_in;z=z_in;R=R_in;P=P_in;Y=Y_in;
  }
};


// list of navigation goals and find by string name method
class Locations {
	public:
	Location * obj= new Location[5]{                //notice to update size of array manually
		{"door_zone_1",-9.50,4.108188,0,0,0,CV_PI},		//1
		{"room_4",-9.0,7.0,0,0,0,CV_PI},							//2
    {"room_2",-3.5,-1.0,0,0,0,1.57},							//3
    {"door_zone_2",-9.0,6.0,0,0,0,-1.57},					//4
    {"move_away",-5.47,6.62,0,0,0,CV_PI/2.0}			//5

};	 //??? warning- non-static data member initializers only available with -std=c++11 or -std=gnu++11 [enabled by default]

	Location * find_loc(string str){
	   for (int i=0; i< sizeof obj; i++){
		if (obj[i].name== str){
				return &obj[i];
			}
		}
	}
};


void nav_to_cb(const std_msgs::String::ConstPtr& msg) {

  // in cb find the right coordinates
  Location *locs;
  Locations loc;
  locs= loc.find_loc(msg->data);
  cout << "bm_nav: got loc: " << locs->name << '\n';//change to ROS_INFO
	
	//tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("bm_nav: Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = locs->x;
  goal.target_pose.pose.position.y = locs->y;
  goal.target_pose.pose.position.z = locs->z;
	goal.target_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(locs->R,locs->P,locs->Y );

  ROS_INFO("bm_nav: Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
	std_msgs::String status;
	
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("bm_nav: nav goal success");
    status.data = "Success";}
  else{
    ROS_INFO("bm_nav: nav goal failed");
		status.data = "Failed";}
	nav_pub.publish(status);

}



int main(int argc, char** argv){
  ros::init(argc, argv, "coffe_demo_nav");

//what are this lines for??
	ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle n;
  ROS_INFO("bm_nav: Hello ");


  ros::Subscriber nav_sub = n.subscribe("/nav_to", 1, nav_to_cb);
	nav_pub=n.advertise<std_msgs::String>("/nav_to_res", 2, true);
  ros::Rate r(50); // 100 hz

  ROS_INFO("bm_nav: Ready!");

  while (ros::ok())
    {
        r.sleep();
    }
  return 0;
}


