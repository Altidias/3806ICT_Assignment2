# 3806ICT_Assignment2 - Mineshaft Surveyor
A framework for multi-robot exploration and rescue based on ROS Noetic and Gazebo that incorporates a stubbed LLM-based rescue planner, map merging, frontier exploration, and classical SLAM.

## Group Members
Yannik Zwolsman (s5247589), Daniel Poulis (s), Jacob Barany (s), Tiam Lamb (s5259308)

## Repository Structure
3806ICT_Assignment2/
├── launch/
│ ├── explorer.launch
│ └── master.launch
├── rviz/
│ └── config.rviz
├── src/
│ ├── cpp/
│ │ ├── frontier_detector.cpp
│ │ ├── frontier_planner.cpp
│ │ └── sensor_interface.cpp
│ ├── python/
│ │ ├── map_merger.py
│ │ ├── multi_robot_controller.py
│ │ └── initialize_slam.py
│ └── msg/
│ ├── AgentState.msg
│ ├── ExecutionStatus.msg
│ ├── ExplorationPlan.msg
│ └── SensorData.msg
├── CMakeLists.txt
├── package.xml
└── README.md

## Overview

This package coordinates two TurtleBot3 robots to:

1. **Simultaneously explore** an unknown Gazebo world using `gmapping` SLAM.
2. **Merge their local maps** into a single global occupancy grid.
3. **Detect frontier regions** (free/unknown boundaries) and cluster them.
4. **Assign frontier goals** via a centralized greedy utility allocator (RL upgrade planned).
5. **Teleport** robots step-by-step in Gazebo to validate exploration rapidly.
6. **Switch to rescue mode**, invoke a stubbed 4-stage LLM planner (`/get_plan`), and eventually A⋆.

---

## Prerequisites

1. **Operating System:** Ubuntu 20.04  
2. **ROS Distribution:** Noetic  
3. **Catkin Workspace:** Already configured (e.g. `~/catkin_ws`)  
4. **TurtleBot3 Packages:** `sudo apt install ros-noetic-turtlebot3`
5. **Gazebo:** Installed via ROS Noetic desktop–full
6. **Python Dependencies:** `sudo apt install python3-rospkg python3-catkin-pkg python3-numpy`

## Build Instructions
1. Clone the repository into your catkin workspace:
   `cd ~/catkin_ws/src`
   `git clone -b jacob-working https://github.com/Altidias/3806ICT_Assignment2.git`
2. Build the workspace:
   `cd ~/catkin_ws`
   `catkin_make`
3. Source the workspace:
   `source devel/setup.bash`

## Runn Instructions
- roslaunch turtlebot3_explorer master.launch
- To get rid of warning spam: roslaunch turtlebot3_explorer master.launch 2>&1 | grep -v "TF_REPEATED_DATA" | grep -v "buffer_core.cpp"

## Key Nodes and Topics
| Node                          | Subscribes                            | Publishes                         |
| ----------------------------- | ------------------------------------- | --------------------------------- |
| **sensor\_interface**         | `/scan`, `/imu`, TF                   | `/sensor_data`                    |
| **map\_merger**               | `/robot1/map`, `/robot2/map`          | `/map`                            |
| **frontier\_detector**        | `/map`                                | `/frontiers`, `/frontier_markers` |
| **frontier\_planner**         | `/frontiers`, `/robotX/odom`          | `/exploration_plan`               |
| **multi\_robot\_controller**  | `/exploration_plan`, `/control_mode`  | `/cmd_vel`, `/execution_status`   |
| **plan\_executor**            | `/get_plan` (service), `/robot_state` | `/cmd_vel`                        |
| **initialize\_slam** (helper) | `/robotX/map`                         | `/robotX/cmd_vel`                 |

## Recording & Playback
- Record: `rosbag record -a -O run.bag`
- Playback: `rosbag play run.bag --clock`

## Future Work
- Replace greedy allocator with Reinforcement Learning
- Integrate LLM + A⋆ fully behind `/get_plan`
- Add priority‐based collision avoidance
