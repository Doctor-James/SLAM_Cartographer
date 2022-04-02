cmake_minimum_required(VERSION 3.0.2)
project(my_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  serial
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot_start 
          src/publish_node.cpp 
			    src/serial1.cpp 
          src/robot_start.cpp)
add_dependencies(robot_start ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_start ${catkin_LIBRARIES})

add_executable(pose_init 
          node/nav_pub_initpose.cpp)
target_link_libraries(pose_init ${catkin_LIBRARIES})


add_executable(sub_goal 
          node/nav_sub_goal.cpp)
target_link_libraries(sub_goal ${catkin_LIBRARIES})


add_executable(linear_cheack 
          scripts/linear_check.cpp)
target_link_libraries(linear_cheack ${catkin_LIBRARIES})

add_executable(angular_check 
          scripts/angular_check.cpp)
target_link_libraries(angular_check ${catkin_LIBRARIES})

# catkin_install_python(PROGRAMS
#   scripts/action01_server_p.py
#   scripts/action02_client_p.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )