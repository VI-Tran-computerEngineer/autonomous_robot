# Autonomous Robot in unknown environment
Autonomous Robot is the robot that can automatically analyse the terrain, identifies obstacles and directions, then moves to predetermined coordinates.
The Autonomous robot system use ROS (Robot operating system) and is integrated to robot model Turtlebot3-Burger.
# Setup and run system step by step
## 1. First, input the goal that expects the robot to go to:
Go to autonomous_robot directory:\
```$ roscd autonomous_robot```\
\
Change the "(x : y)" coodinate of the goal in line 15 of "execute.sh" file:\
```$ rosrun automotive_robot RobotGlobalVisionUpdate.py -gx {x value} -gy {y value}```
## 2. Run roscore in the master device:
```$ roscore```
## 3. Run the "Bringup" node in the robot's embedded Linux system:
```$ roslaunch turtlebot3_bringup turtlebot3_robot.launch```
## 4. Run the SLAM node:
```$ roslaunch turtlebot3_slam turtlebot3_slam.launch```
## 5. Run execute.sh after all setup steps:
Go to autonomous_robot directory:\
```$ roscd autonomous_robot```\
\
Then run the bash file to start:\
```$ ./execute.sh```
