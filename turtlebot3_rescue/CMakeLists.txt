cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_rescue)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_service_files(
  FILES
  OptimizePath.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rescue_planner src/rescue_planner.cpp)
add_dependencies(rescue_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rescue_planner ${catkin_LIBRARIES})


# Ensure Python scripts are installed correctly
catkin_install_python(PROGRAMS
  scripts/llm_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Verify script existence (optional debug)
file(GLOB PYTHON_SCRIPTS scripts/*.py)
if(NOT PYTHON_SCRIPTS)
  message(WARNING "No Python scripts found in ${CMAKE_CURRENT_SOURCE_DIR}/scripts")
endif()
