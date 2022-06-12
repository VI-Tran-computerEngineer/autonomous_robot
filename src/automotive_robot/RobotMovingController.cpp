#include "automotive_robot_headers/RobotController.hpp"

void Turtlebot3_MovingController::stop_moving_stop_rotating(){
    this->p_velMsg->linear.x = 0;
    this->p_velMsg->angular.z = 0;
    this->p_RobotController_velPub->publish(*this->p_velMsg);
}


inline void Turtlebot3_MovingController::adjust_moving_direction(double current_angle_diff){
    double exactAngleDiff = (abs(current_angle_diff) > M_PI) ? 2*M_PI - abs(current_angle_diff) : abs(current_angle_diff);

    if (exactAngleDiff > MOVING_ADJUST_ANGLE_DIFFERENCE_TOLERANCE) {
        this->p_velMsg->angular.z = (current_angle_diff < 0) ? 1 : -1;
        this->p_velMsg->angular.z *= MOVING_ADJUST_ANGULAR_SPEED_COEFFICENT*exactAngleDiff;
        if (abs(current_angle_diff) > double(300)/180*M_PI) this->p_velMsg->angular.z = -this->p_velMsg->angular.z ;
    } 
}


inline void Turtlebot3_MovingController::moving_accelerate(const automotive_robot::Point* p_next_point){   // chi lam dung chuc nang tang toc
    // cout << "\n\n\nmoving_accelerate:\n";

    for(int i = 0; i < MOVING_ACCELERATE_TIME*RESOLUTION_RATE && distance(this->p_cposEncoder, p_next_point) > MOVING_ACCELERATE_TOLERANCE*1.25*1.25; i++){

        if (i%MOVING_ACCELERATE_UPDATE_VELOCITY_RATE == 0) this->p_velMsg->linear.x += MOVING_ACCELERATE_VELOCITY_CHANGE;
        this->p_RobotController_velPub->publish(*this->p_velMsg);
        this->p_ResolutionRate->sleep();
        // cout << i << " linear speed: " << actual_odom_velocity << " m/s\n";
        adjust_moving_direction(*this->p_odomAngleXAxis - Direction(this->p_cposEncoder, p_next_point));
    }
}


inline void Turtlebot3_MovingController::moving_stable(const automotive_robot::Point* p_next_point, double keep_moving_distance){
    this->p_velMsg->linear.x = MOVING_STABLE_SPEED; // force velocity to be maximum, bc robot speed might not be maximum
    this->p_velMsg->angular.z = 0; // force stop rotating bc the robot might still be rotating

    // cout << "\n\n\nmoving_stable:\n";
    // int i = 0;

    while (distance(this->p_cposEncoder, p_next_point) > keep_moving_distance) {
        this->p_RobotController_velPub->publish(*this->p_velMsg);
        this->p_ResolutionRate->sleep();
        // cout << i << " linear speed: " << actual_odom_velocity << " m/s\n";
        // i++;
        adjust_moving_direction(*this->p_odomAngleXAxis - Direction(this->p_cposEncoder, p_next_point));
    }
}


inline void Turtlebot3_MovingController::moving_decelerate(const automotive_robot::Point* p_next_point){  
    // cout << "\n\n\nmoving_decelerate:\n";

    for(int i = 0; i < MOVING_DECELERATE_TIME*RESOLUTION_RATE; i++){
        if (i%MOVING_DECELERATE_UPDATE_VELOCITY_RATE == 0) this->p_velMsg->linear.x -= MOVING_DECELERATE_VELOCITY_CHANGE;
        this->p_RobotController_velPub->publish(*this->p_velMsg);
        this->p_ResolutionRate->sleep();
        // cout << i << " linear speed: " << actual_odom_velocity << " m/s\n";
        adjust_moving_direction(*this->p_odomAngleXAxis - Direction(p_cposEncoder, p_next_point));
    }
}


void Turtlebot3_MovingController::moving_very_slow(const automotive_robot::Point* p_next_point){
    int i = 0;

    // cout << "\n\n\nmoving_very_slow:\n";

    while (distance(this->p_cposEncoder, p_next_point) > MOVING_VERY_SLOW_TOLERANCE) {
        this->p_RobotController_velPub->publish(*this->p_velMsg);
        this->p_ResolutionRate->sleep();
        // cout << i << " linear speed: " << actual_odom_velocity << " m/s\n";
        double direction = Direction(this->p_cposEncoder, p_next_point);
        adjust_moving_direction(*this->p_odomAngleXAxis - direction);
        i++;
        if (i >= RESOLUTION_RATE*10) {
            cout << "very slow moving:   stop because has runned  for 10s\n";
            return; // 10s
        }
        if (abs(*this->p_odomAngleXAxis - direction) > 100.00/180*M_PI)
        {
            cout << "very slow moving:   stop because has overrun\n";
            return;
        }
    }
    cout << "very slow moving:   successful distance below 1cm\n";
}


void Turtlebot3_MovingController::moving_to_next_point(const automotive_robot::Point* p_next_point) {
    moving_accelerate(p_next_point);
    moving_stable(p_next_point, MOVING_STABLE_TOLERANCE*1.2*1.2);
    moving_decelerate(p_next_point);
    moving_very_slow(p_next_point);
    stop_moving_stop_rotating();
}

void Turtlebot3_MovingController::setField(ros::Rate* p_ResolutionRate, ros::Publisher* p_RobotController_velPub, geometry_msgs::Twist* p_velMsg,
volatile automotive_robot::Point* p_cposEncoder, volatile double* p_odomAngleXAxis){

    this->p_ResolutionRate = p_ResolutionRate;
    this->p_RobotController_velPub = p_RobotController_velPub;
    this->p_velMsg = p_velMsg;
    this->p_cposEncoder = p_cposEncoder;
    this->p_odomAngleXAxis = p_odomAngleXAxis;
}