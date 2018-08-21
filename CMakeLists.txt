
################################
### Auto-generated CMakeList ###
################################
cmake_minimum_required(VERSION 2.8.3)
project(room_analysis)

include(FindPkgConfig)
pkg_check_modules(CURLPP REQUIRED curlpp)

set( CMAKE_CXX_STANDARD 11 )
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


################
### Packages ###
################
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	node_base
	ar_track_alvar_msgs
	actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES Explore.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

#############
### Build ###
#############
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)
include_directories(
	${catkin_INCLUDE_DIRS}
	${node_base_INCLUDE_DIRS}
	include
)
add_executable(analyzer.impl src/Analyzer.cpp)
add_dependencies(analyzer.impl room_analysis_generate_messages_cpp)
target_link_libraries(analyzer.impl
	${catkin_LIBRARIES}
	${node_base_LIBRARIES}
	${CURLPP_LDFLAGS}
)
