cmake_minimum_required(VERSION 2.8.3)
project(trajectory_sender)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	control_msgs
	tf
	genmsg
	actionlib_msgs
	actionlib	
	)

find_package(Boost REQUIRED COMPONENTS system)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS 
	roscpp
	std_msgs
	geometry_msgs
	nav_msgs
	control_msgs
	tf
	INCLUDE_DIRS
	include
	 )


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${Boost_INCLUDE_DIR})

add_executable(traj_sender src/traj_sender.cpp )
target_link_libraries(traj_sender ${catkin_LIBRARIES})


add_dependencies(traj_sender std_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp nav_msgs_generate_messages_cpp control_msgs_generate_messages_cpp)
