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

## vision instructions
* make sure to run "catkin_make"
* run with "roslaunch bgumodo_vision bgumodo_vision.launch"
* to initiate detection - call topic "/detector/observe_cup_cmd" with string "Start"
* notice only one red object should be in the scene. if you get NaN reading check if object is to close. (in gazebo table with objects world I have checked with the Cola can position on the far end of the table.)
