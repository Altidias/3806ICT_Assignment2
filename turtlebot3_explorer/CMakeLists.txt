cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_explorer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf2_ros
  visualization_msgs
  std_srvs
  tf
  message_generation
)

add_message_files(
  FILES
  AbstractMap.msg
  AgentState.msg
  ExecutionStatus.msg
  ExplorationPlan.msg
  SensorData.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlebot3_explorer
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs std_srvs tf message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(frontier_detector src/cpp/frontier_detector.cpp)
add_executable(plan_executor src/cpp/plan_executor.cpp)
add_executable(sensor_interface src/cpp/sensor_interface.cpp)

add_dependencies(frontier_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(plan_executor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sensor_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(frontier_detector ${catkin_LIBRARIES})
target_link_libraries(plan_executor ${catkin_LIBRARIES})
target_link_libraries(sensor_interface ${catkin_LIBRARIES})


install(TARGETS
  frontier_detector
  plan_executor
  sensor_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_install_python(PROGRAMS
  src/python/exploration_coordinator.py
  src/python/llm_planner.py
  src/python/map_processor.py
  src/python/map_saver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
