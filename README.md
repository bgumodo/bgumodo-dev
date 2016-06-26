## clone git to new computer
* open catkin_ws/src
* git init
* git clone https://github.com/bgumodo/bgumodo-dev.git

## get last changes 
* git pull https://github.com/bgumodo/bgumodo-dev.git

## editing
* edit your code
* open a terminal and type 'git gui'
* use the gui to stage, commit and push your edit 

### Running armadilo simulation:
* roslaunch robotican_demos armadillo_moveit_and_nav.launch
* rosrun robotican_demos simple_move_group_goals
* rosrun robotican_demos simple_navigation_goals

## example: move arm command
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

## example: navigation command
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
 
## add coke can to the simulation
rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can5 -z 1.1 -y 8.10 -x -8.44

## remove the coke can from simulation
rosservice call gazebo/delete_model '{model_name: coke_can2}'

## vision instructions
* make sure to run "catkin_make"
* run with "roslaunch bgumodo_vision bgumodo_vision.launch"
* to initiate detection - call topic "/detector/observe_cup_cmd" with string "Start"
* notice only one red object should be in the scene. if you get NaN reading check if object is to close. (in gazebo table with objects world I have checked with the Cola can position on the far end of the table.)
