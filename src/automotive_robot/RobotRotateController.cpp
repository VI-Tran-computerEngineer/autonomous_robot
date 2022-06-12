#include "automotive_robot_headers/RobotController.hpp"

inline void Turtlebot3_RotateController::rotate_stop(){
    this->p_velMsg->angular.z = 0; // force stop rotating bc the robot might still be rotating
    this->p_RobotController_velPub->publish(*this->p_velMsg);
}    

inline void Turtlebot3_RotateController::clearVelocity(){
    this->p_velMsg->linear.x = 0;
    this->p_velMsg->linear.y = 0;
    this->p_velMsg->linear.z = 0;
    // set a random angular velocity in the y-axis
    this->p_velMsg->angular.x = 0;
    this->p_velMsg->angular.y = 0;
    this->p_velMsg->angular.z = 0;
}

inline void Turtlebot3_RotateController::rotate_accelerate(bool rotate_right, double direction){
    const double rotate_change = ROTATE_STABLE_SPEED/ROTATE_CHANGE_SMOOTH_LEVEL*((rotate_right)?-1:1);  // radian/s

    clearVelocity();

    // cout << "\n\n\nrotate_accelerate:\n";

    for (int i = 0; i < ROTATE_ACCELERATE_TIME*RESOLUTION_RATE && \
    (abs(*this->p_odomAngleXAxis - direction) > ROTATE_ACCELERATE_TOLERANCE*1.1); i++){// 40 degree

        if (i % ROTATE_ACCELERATE_UPDATE_VELOCITY_RATE == 0) this->p_velMsg->angular.z += rotate_change;
        this->p_RobotController_velPub->publish(*this->p_velMsg);
        this->p_ResolutionRate->sleep(); 
        // cout << i << " rotate speed: " << z/M_PI*180 << endl;
    }
}

inline void Turtlebot3_RotateController::rotate_stable(double direction){
    this->p_velMsg->angular.z = ROTATE_STABLE_SPEED*((this->p_velMsg->angular.z < 0)?-1:1);
    
    // cout << "\n\n\nrotate_stable:\n";

    while (abs(direction - *this->p_odomAngleXAxis) > ROTATE_STABLE_TOLERANCE*1.1) { // 40 do
        this->p_RobotController_velPub->publish(*this->p_velMsg);
        this->p_ResolutionRate->sleep();
        // cout << "rotate speed: " << z/M_PI*180 << endl;
    }
}

inline void Turtlebot3_RotateController::rotate_decelerate(bool rotate_right, double direction){
    const double rotate_change = ROTATE_STABLE_SPEED*0.8/ROTATE_CHANGE_SMOOTH_LEVEL*((rotate_right)?-1:1);

    // cout << "\n\n\nrotate_decelerate:\n";

    for (int i = 0; i < RESOLUTION_RATE*ROTATE_DECELERATE_TIME; i++){
        if (i % ROTATE_DECELERATE_UPDATE_VELOCITY_RATE == 0) this->p_velMsg->angular.z -= rotate_change;
        this->p_RobotController_velPub->publish(*this->p_velMsg);
        this->p_ResolutionRate->sleep();
        // cout << i << " rotate speed: " << z/M_PI*180 << endl;
    }
}

inline void Turtlebot3_RotateController::rotate_very_slow(double direction){
        double before = exactAngelDeviationFromOdomAngelXAxis(*this->p_odomAngleXAxis ,direction);

        // cout << "\n\n\nrotate_very_slow\n";

        while (exactAngelDeviationFromOdomAngelXAxis(*this->p_odomAngleXAxis, direction) > ROTATE_VERY_SLOW_TOLERANCE) {
            this->p_RobotController_velPub->publish(*this->p_velMsg);
            this->p_ResolutionRate->sleep();
            // cout << "rotate speed: " << z/M_PI*180 << endl;
            double after = abs(direction - *this->p_odomAngleXAxis);
            if (after > M_PI) after = 2*M_PI - after;
            if (after > before) {
                this->p_velMsg->angular.z = -this->p_velMsg->angular.z;
                this->p_RobotController_velPub->publish(*this->p_velMsg);
                this->p_ResolutionRate->sleep();
            }
            before = after;
        }
}

// rotate clockwise
void Turtlebot3_RotateController::rotate(bool rotate_right, double direction) {
    if (exactAngelDeviationFromOdomAngelXAxis(*this->p_odomAngleXAxis, direction) > 55.00/180*M_PI){
        rotate_accelerate(rotate_right, direction);
        rotate_stable(direction);
        rotate_decelerate(rotate_right, direction);
    }
    else {
        cout << "go to very slow rotate immediately!\n";
        this->p_velMsg->angular.z = 0.2*ROTATE_STABLE_SPEED*(rotate_right?-1:1);
        for (int i = 0; i < 30; i++){
            this->p_RobotController_velPub->publish(*this->p_velMsg);
            this->p_ResolutionRate->sleep();
        }
    }
    rotate_very_slow(direction);
    rotate_stop();
}

void Turtlebot3_RotateController::setField(ros::Rate* p_ResolutionRate, ros::Publisher* p_RobotController_velPub, 
geometry_msgs::Twist* p_velMsg, volatile double* p_odomAngleXAxis){
    this->p_ResolutionRate = p_ResolutionRate;
    this->p_RobotController_velPub = p_RobotController_velPub;
    this->p_velMsg = p_velMsg;
    this->p_odomAngleXAxis = p_odomAngleXAxis;
}