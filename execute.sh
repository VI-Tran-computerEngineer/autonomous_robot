#!/bin/bash

echo "===========	Begin catkin make 	=======!"
cd ~/catkin_ws
catkin_make
echo "===========	Catkin make done 	=======!"

echo "===========	Add sources 		=======!"
source devel/setup.bash

gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash | rosrun automotive_robot PaperFinder.py"
gnome-terminal --tab -- bash -c "source ~/catkin_ws/devel/setup.bash | rosrun automotive_robot robot_motion_controller_node"

echo "===========	RUN Python node	=======!"
rosrun automotive_robot RobotGlobalVisionUpdate.py -gx 2.00 -gy 0.20

