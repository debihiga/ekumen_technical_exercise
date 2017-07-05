cmake_minimum_required(VERSION 2.8.3)
project(ekumen_technical_exercise)

## Use C++11
add_definitions(--std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    actionlib
    actionlib_msgs
    message_generation
    geometry_msgs
    turtlesim
    std_msgs
    roslaunch
    std_srvs
)

roslaunch_add_file_check(launch)


add_action_files(
	DIRECTORY action
	FILES Turtle.action
)

generate_messages(
	DEPENDENCIES
		actionlib_msgs	# result
		geometry_msgs	# goal
		std_msgs		# feedback
		turtlesim
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES
  CATKIN_DEPENDS
  	actionlib
  	actionlib_msgs
  	message_generation
  	roscpp
  	std_msgs
  	turtlesim
  	std_srvs
  DEPENDS
  	roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare cpp executables
add_executable(turtle_server
  src/turtle_server.cpp
)


## Specify libraries to link executable targets against
target_link_libraries(turtle_server
  ${catkin_LIBRARIES}
)

add_dependencies(turtle_server
  ${PROJECT_NAME}_EXPORTED_TARGETS
  ${catkin_EXPORTED_TARGETS})


add_executable(turtle_client src/turtle_client.cpp)
target_link_libraries(turtle_client ${catkin_LIBRARIES})
add_dependencies(turtle_client ${PROJECT_NAME}_EXPORTED_TARGETS
  ${catkin_EXPORTED_TARGETS})

add_executable(resume_turtle_srv src/resume_turtle.cpp)
target_link_libraries(resume_turtle_srv ${catkin_LIBRARIES})
#add_dependencies(resume_turtle_srv ${PROJECT_NAME}_gencpp)

add_executable(pause_turtle_srv src/pause_turtle.cpp)
target_link_libraries(pause_turtle_srv ${catkin_LIBRARIES})
#add_dependencies(pause_turtle_srv ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
#install(
#  TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# Mark cpp header files for installation
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.hpp"
#)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

