# 3806ICT_Assignment2 - Mineshaft Surveyor
A framework for multi-robot exploration and rescue based on ROS Noetic and Gazebo that incorporates an LLM-based rescue planner, map merging, frontier exploration, and classical SLAM.

### Group Members
Yannik Zwolsman (s5247589), Daniel Poulis (s5263785), Jacob Barany (s5278113), Tiam Lamb (s5259308)

---

## Overview

This package coordinates multiple robots to simulatenously explore and map the world using slam gmapping to discover survivors, then coordinates a rescue robot to traverse an optimal path to collect survivors.


---

### Prerequisites

1. **Operating System:** Ubuntu 20.04  
2. **ROS Distribution:** Noetic  
3. **Catkin Workspace:** Already configured (e.g. `~/catkin_ws`)  
4. **TurtleBot3 Packages:** `sudo apt install ros-noetic-turtlebot3`
5. **Gazebo:** Installed via ROS Noetic desktop–full
6. **Python Dependencies:** `sudo apt install python3-rospkg python3-catkin-pkg python3-numpy`

---

### Build Instructions
1. Clone the repository into your catkin workspace:
   `cd ~/catkin_ws/src`
   `git clone -b jacob-working https://github.com/Altidias/3806ICT_Assignment2.git`
2. Build the workspace:
   `cd ~/catkin_ws`
   `catkin_make`
3. Source the workspace:
   `source devel/setup.bash`

---

### Run Instructions
- Launch main script (runs every node needed to coordinate the search and rescue): roslaunch turtlebot3_explorer master.launch
- Launch visualisation: roslaunch turtlebot3_explorer viz.launch
- To get rid of warning spam: roslaunch turtlebot3_explorer master.launch 2>&1 | grep -v "TF_REPEATED_DATA" | grep -v "buffer_core.cpp"



