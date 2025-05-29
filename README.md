# 3806ICT_Assignment2
Mineshaft Exploration and Retrieval ROS

### Group Members
Yannik Zwolsman (s5247589), Daniel Poulis (s), Jacob Barany (s), Tiam Lamb (s)

### Build Instructions
- Copy /turtlebot3_explorer into ~/catkin_ws/src
- cd ~/catkin_ws
- catkin_make

### Run Instructions
- roslaunch turtlebot3_explorer master.launch
- To get rid of warning spam: roslaunch turtlebot3_explorer master.launch 2>&1 | grep -v "TF_REPEATED_DATA" | grep -v "buffer_core.cpp"