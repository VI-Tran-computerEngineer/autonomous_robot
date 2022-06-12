#include "automotive_robot_headers/RobotController.hpp"

int main(int argc, char **argv) {
    cout << "start RobotMain\n";
    ros::init(argc, argv, "robot_motion"); 
    Turtlebot3MotionController::setFields(argc, argv);
    Turtlebot3MotionController::robotMotionControllerSpin();
    return 0;
}