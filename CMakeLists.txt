cmake_minimum_required(VERSION 3.16.3)
project(automotive_robot)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  nav_msgs
  tf
  cv_bridge
  laser_geometry
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES Path.msg Point.msg) 

#add_service_files(FILES )

catkin_python_setup()
generate_messages(DEPENDENCIES std_msgs nav_msgs geometry_msgs sensor_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#INCLUDE_DIRS include
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime nav_msgs tf cv_bridge laser_geometry sensor_msgs
)

## add include files
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-std=c++11) 

###########
## Build ##
###########



## Declare a C++ executable
add_executable(robot_motion_controller_node src/automotive_robot/RobotNodeMain.cpp src/automotive_robot/RobotRotateController.cpp
    src/automotive_robot/RobotMovingController.cpp
    src/automotive_robot/RobotMotionController.cpp
    src/automotive_robot/RobotMotionControllerLib.cpp)
add_dependencies(robot_motion_controller_node automotive_robot_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robot_motion_controller_node ${catkin_LIBRARIES})



catkin_install_python(PROGRAMS src/automotive_robot/RobotGlobalVisionUpdate.py src/automotive_robot/PaperFinder.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############

install(DIRECTORY filter launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS robot_motion_controller_node
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/automotive_robot_headers/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



#############
## Testing ##
#############

