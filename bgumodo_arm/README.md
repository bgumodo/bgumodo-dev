Instructions
============
1.
- rosrun bgumodo_arm base_pose_calc
- rosrun bgumodo_arm top_grasping_manipulator
- rosrun bgumodo_arm push_manipulator

2.
make sure the elevator value set to 0.4

3.
get desired base pose (geometry_msgs/PoseStamped) by sending **bgumodo_arm/ThreePoints** message type to **base_pose_calc/input_points** (two green cards reference position and cup position)

4.
- manipulate by sending **geometry_msgs/ThreePoints** message type (again) to **top_grasping_manipulator/command**
- for pushing manipulator just replace the topic to **push_manipulator/command**
