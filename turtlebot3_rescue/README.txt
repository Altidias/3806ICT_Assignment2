------- RUN COMMANDS ---------
roslaunch turtlebot3_rescue rescue.launch
rosrun map_server map_server ~/catkin_ws/src/turtlebot3_rescue/maps/test_map_large.yaml
rosrun turtlebot3_rescue llm_service.py
rosrun turtlebot3_rescue rescue_planner

-------- DEBUG ----------
rostopic echo /rescue_status
rostopic echo /move_base_simple/goal

