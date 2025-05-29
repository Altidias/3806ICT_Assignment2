cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_explorer)

## Compile as C++14
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

# catkin_python_setup()

# services
add_service_files(
  FILES
  PlanPath.srv
  GetPathCosts.srv
  FollowPath.srv
  CheckSurvivor.srv
  GetSurvivors.srv
  PlanRescuePath.srv
  SelectFrontiers.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# cpp nodes
add_executable(frontier_detector src/frontier_detector.cpp)
target_link_libraries(frontier_detector ${catkin_LIBRARIES})
add_dependencies(frontier_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(astar_planner src/astar_planner.cpp)
target_link_libraries(astar_planner ${catkin_LIBRARIES})
add_dependencies(astar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(heuristic_calculator src/heuristic_calculator.cpp)
target_link_libraries(heuristic_calculator ${catkin_LIBRARIES})
add_dependencies(heuristic_calculator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})
add_dependencies(path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(survivor_manager src/survivor_manager.cpp)
target_link_libraries(survivor_manager ${catkin_LIBRARIES})
add_dependencies(survivor_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# python scripts
catkin_install_python(PROGRAMS
  scripts/map_merger.py
  scripts/robot_coordinator.py
  scripts/slam_initializer.py
  scripts/robot_utils.py
  scripts/rescue_robot.py
  scripts/llm_frontier_selector.py
  scripts/llm_rescue_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)