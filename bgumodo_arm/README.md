Instructions
============
1.
rosrun bgumodo_arm base_pose_calc
rosrun bgumodo_arm side_grasping_manipulator
rosrun bgumodo_arm top_grasping_base_pose_calc
rosrun bgumodo_arm top_grasping_manipulator
rosrun bgumodo_arm push_manipulator

2.
make sure the elevator value set to 0.4

3.
Side grasping | Top grasping
------------- | ------------
get desired base pose (geometry_msgs/PoseStamped) by sending **bgumodo_arm/TwoPoints** message type to **base_pose_calc/input_points**
manipulate by sending **geometry_msgs/Point** message type with the cup position to **side_grasping_manipulator/command**|get final desired base pose (geometry_msgs/PoseStamped) by sending **geometry_msgs/Point** message type with the cup position to **top_grasping_base_pose_calc/input_point**|SAME
|manipulate by sending **geometry_msgs/Point** message type with the cup position to **top_grasping_manipulator/command**