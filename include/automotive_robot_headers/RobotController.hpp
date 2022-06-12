#ifndef _TURTLEBOT3_ROBOT_CONTROLLER_
#define _TURTLEBOT3_ROBOT_CONTROLLER_

/******************************************************************************************************/
/* include section */
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "std_msgs/String.h"
#include "automotive_robot/Path.h"
#include "automotive_robot/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <cstdint>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <math.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <stdlib.h>
#include <fstream>
/******************************************************************************************************/


/******************************************************************************************************/
/* defines in Robot Controller*/

#define RESOLUTION_RATE                         unsigned(20)                               // Hz
#define CAMERA_VISION                           ((double(30) / 180) * M_PI)                // radian

/******************************************************************************************************/



/******************************************************************************************************/
/* defines in Robot Rotate Controller*/
#define ROTATE_STABLE_SPEED                     ((double(25) / 180) * M_PI)                // radian/s
#define ROTATE_CHANGE_SMOOTH_LEVEL              unsigned(20)                                // no unit
#define ROTATE_DECELERATE_TIME                  unsigned(2)                                // seconds
#define ROTATE_ACCELERATE_TIME                  unsigned(2)                                // seconds
#define ROTATE_ACCELERATE_TOLERANCE             (ROTATE_STABLE_SPEED*ROTATE_STABLE_SPEED*(1 - 0.2*0.2))\
                            /(2*0.8*ROTATE_STABLE_SPEED/ROTATE_DECELERATE_TIME)          // 36 degrees
#define ROTATE_ACCELERATE_UPDATE_VELOCITY_RATE  \
            (ROTATE_ACCELERATE_TIME * RESOLUTION_RATE) / ROTATE_CHANGE_SMOOTH_LEVEL        // no unit
#define ROTATE_STABLE_TOLERANCE                 ROTATE_ACCELERATE_TOLERANCE                // 36 degrees 
#define ROTATE_DECELERATE_UPDATE_VELOCITY_RATE  \
            (ROTATE_DECELERATE_TIME * RESOLUTION_RATE) / ROTATE_CHANGE_SMOOTH_LEVEL        // no unit
#define ROTATE_VERY_SLOW_TOLERANCE              (0.75 / 180 * M_PI)                        // 0.75 degree

/******************************************************************************************************/

/******************************************************************************************************/
/* defines in Robot Moving Controller */
#define MOVING_STABLE_SPEED                         double(0.15)                           // m/s
#define MOVING_CHANGE_SMOOTH_LEVEL                  unsigned(20)                            // no unit
#define MOVING_DECELERATE_TIME                      unsigned(2)                            // seconds
#define MOVING_ACCELERATE_TIME                      unsigned(2)                            // seconds
#define MOVING_ACCELERATE_TOLERANCE                 pow((MOVING_STABLE_SPEED*MOVING_STABLE_SPEED*(1 - 0.2*0.2))\
                                /(2*0.8*MOVING_STABLE_SPEED/MOVING_DECELERATE_TIME) ,2)    // m^2
#define MOVING_ACCELERATE_UPDATE_VELOCITY_RATE  \
            (MOVING_ACCELERATE_TIME * RESOLUTION_RATE) / MOVING_CHANGE_SMOOTH_LEVEL        // no unit
#define MOVING_ACCELERATE_VELOCITY_CHANGE           MOVING_STABLE_SPEED/ \
                                MOVING_CHANGE_SMOOTH_LEVEL                                 // m/s
#define MOVING_STABLE_TOLERANCE                     MOVING_ACCELERATE_TOLERANCE            // m^2
#define MOVING_DECELERATE_VELOCITY_CHANGE           0.8*MOVING_ACCELERATE_VELOCITY_CHANGE  // m/s
#define MOVING_DECELERATE_UPDATE_VELOCITY_RATE  \
            (MOVING_DECELERATE_TIME * RESOLUTION_RATE) / MOVING_CHANGE_SMOOTH_LEVEL        // no unit
#define MOVING_VERY_SLOW_TOLERANCE                  MOVING_STABLE_TOLERANCE/85             // m^2
#define MOVING_ADJUST_ANGULAR_SPEED_COEFFICENT      double(0.25)                           // no unit
#define MOVING_ADJUST_ANGLE_DIFFERENCE_TOLERANCE    ((double(1) / 180) * M_PI)             // degree 
/******************************************************************************************************/


/******************************************************************************************************/
/* namespace used section */
using namespace std;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;
/******************************************************************************************************/

/******************************************************************************************************/
/* global varibles */
extern volatile double z;
extern volatile double actual_odom_velocity;
/******************************************************************************************************/

/******************************************************************************************************/
/* global function */
inline double exactAngelDeviationFromOdomAngelXAxis(double odomAngleXaxis, double direction){
    if (abs(odomAngleXaxis - direction) > M_PI) 
        return 2*M_PI - abs(odomAngleXaxis - direction);
    
    return abs(odomAngleXaxis - direction);
}

inline float distance(const volatile automotive_robot::Point* p_pointA, const automotive_robot::Point* p_pointB){
    return (p_pointA->x - p_pointB->x)*(p_pointA->x - p_pointB->x) + (p_pointA->y - p_pointB->y)*(p_pointA->y - p_pointB->y);
}

inline double Direction(const volatile automotive_robot::Point* p_p1, const automotive_robot::Point* p_p2) {
    return atan2(p_p2->y - p_p1->y, p_p2->x - p_p1->x);
}
/******************************************************************************************************/


/******************************************************************************************************/
/* Robot Rotate Controller */ 
class Turtlebot3_RotateController{
private:    
    ros::Rate* p_ResolutionRate; 
    ros::Publisher* p_RobotController_velPub;
    geometry_msgs::Twist* p_velMsg; 
    volatile double* p_odomAngleXAxis;

    inline void rotate_stop();

    inline void clearVelocity();

    inline void rotate_accelerate(bool rotate_right, double direction);

    inline void rotate_stable(double direction);

    inline void rotate_decelerate(bool rotate_right, double direction);

    inline void rotate_very_slow(double direction);

public:
    void setField(ros::Rate* p_ResolutionRate, ros::Publisher* p_RobotController_velPub, geometry_msgs::Twist* p_velMsg, volatile double* p_odomAngleXAxis);

    void rotate(bool rotate_right, double direction);    // rotate clockwise
};
/******************************************************************************************************/


/******************************************************************************************************/
/* Robot Moving Controller */
class Turtlebot3_MovingController{
private:
    ros::Rate* p_ResolutionRate; 
    ros::Publisher* p_RobotController_velPub;
    geometry_msgs::Twist* p_velMsg;
    volatile automotive_robot::Point* p_cposEncoder;
    volatile double* p_odomAngleXAxis;

    inline void adjust_moving_direction(double current_angle_diff);

    inline void moving_accelerate(const automotive_robot::Point* p_next_point);

    inline void moving_stable(const automotive_robot::Point* p_next_point, double keep_moving_distance);

    inline void moving_decelerate(const automotive_robot::Point* p_next_point);


public:
    void setField(ros::Rate* p_ResolutionRate, ros::Publisher* p_RobotController_velPub, geometry_msgs::Twist* p_velMsg,
    volatile automotive_robot::Point* p_cposEncoder, volatile double* p_odomAngleXAxis);

    void stop_moving_stop_rotating();

    void moving_very_slow(const automotive_robot::Point* p_next_point);

    void moving_to_next_point(const automotive_robot::Point* p_next_point);
};

/******************************************************************************************************/
class Turtlebot3MotionController{
private:
    static volatile double odomAngleXAxis;
    static volatile bool updateVectorEncoderSlam;
    
    //volatile double z = 0;
    //volatile double actual_odom_velocity;
    
    static geometry_msgs::Twist vel_msg;
    static ros::Rate* p_ResolutionRate;

    static volatile automotive_robot::Point cposSlam;
    static volatile automotive_robot::Point cposEncoder;
    static volatile automotive_robot::Point encoderSlam;

    static ros::NodeHandle* p_RobotControllerNode;

    static ros::Publisher velocity_publisher;
    static ros::Publisher moveFinished_publisher;
    static ros::Publisher updateSlamPos_publisher;
    static ros::Publisher doneChangeView_publisher;

    static ros::Subscriber Odometry_sub;
    static ros::Subscriber cmd_moving_sub;
    static ros::Subscriber changeView_sub;

    static Turtlebot3_RotateController rotateController;
    static Turtlebot3_MovingController movingController;

    static void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    static void moveCallback(const automotive_robot::Path::ConstPtr &msg);
    static void changeViewCallback(const std_msgs::String::ConstPtr &msg);

    static inline bool motion_determine_direction(const automotive_robot::Point* next_point, double &direction);

public:
    static void setFields(int argc, char **argv);
    static void robotMotionControllerSpin();
};

#endif /* _TURTLEBOT3_ROBOT_MOTION_ */