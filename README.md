## setting-up the simulation 
* cd catkin_ws/src
* git clone https://github.com/bgumodo/bgumodo-dev.git
* git clone https://github.com/robotican/robotican.git
* cd robotican && git checkout temp_branch
* follow [this](http://wiki.ros.org/robotican/Tutorials) for further robotican installation.
* cd ~/catkin_ws
* catkin_make

## Running armadilo simulation:
* roslaunch robotican_demos armadillo_moveit_and_nav.launch
* rosrun robotican_demos simple_move_group_goals
* rosrun robotican_demos simple_navigation_goals

## example: move arm publish command
rostopic pub /move_arm geometry_msgs/PoseStamped "header:  
  seq: 0  
  stamp:  
    secs: 0  
    nsecs: 0  
  frame_id: 'base_link'  
pose:  
  position:  
    x: 0.37  
    y: 0.0  
    z: 0.8  
  orientation:  
    x: 0.0  
    y: 0.4  
    z: 0.0  
    w: 0.91"  

## example: navigation publish command
rostopic pub /nav_command geometry_msgs/PoseStamped "header:  
  seq: 0  
  stamp:  
    secs: 0  
    nsecs: 0  
  frame_id: 'map'  <- (base_link is also an option)  
pose:  
  position:  
    x: -4.5  
    y: 8.37  
    z: 0.0  
  orientation:  
    x: 0.0  
    y: 0.0  
    z: 0.0  
    w: 1.0"  
 
## adding coke can to the simulation
rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can5 -z 1.1 -y 8.10 -x -8.44  
ROS Node API:  
char exec[128] = {'\0'};  
sprintf(exec, "rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can2 -x %f -y %f -z %f",button_x, button_y, button_z);  
// bash command
FILE *process = popen(exec, "r");  
if(process == 0) {  
    ROS_ERROR("[%s]: can't start the procces shuting down the node :(", ros::this_node::getName().c_str());  
    ros::shutdown();  
}  

## remove the coke can from simulation
rosservice call gazebo/delete_model '{model_name: coke_can2}'

## vision instructions
* make sure to run "catkin_make"
* run with "roslaunch bgumodo_vision bgumodo_vision.launch"
* to initiate detection - call topic "/detector/observe_cup_cmd" with string "Start"
* notice only one red object should be in the scene. if you get NaN reading check if object is to close. (in gazebo table with objects world I have checked with the Cola can position on the far end of the table.)
* you may need to play around with the kinect's tilt to see the objects better - to do so call use the rqt's message publisher to call the topic "/pan_tilt_controller/command" and in the "data" field enter your desired position in radians ([pan, tilt]), e.g [0.0, 0.3]


## Navigation instructions:

Run "roslaunch bgumodo_nav armadillo_moveit_with_nav_alex.launch".
To start navigation publish to topic "/navigation/move_cmd" with type=String the following options: (case sensitive)

	1. "door_zone_1" : door closed position.
	2. "door_zone_2" : door opened position.
	3. "room_4" : target room with coke can.
	4. "room_2" : final target room for coke can.

Subscribe to topic "/navigation/move_res" type=String for the navigation result.
Possible result:
	1. "Success"
	2. "Failed"

