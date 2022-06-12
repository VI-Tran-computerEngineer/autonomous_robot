#include "automotive_robot_headers/RobotController.hpp"

volatile double Turtlebot3MotionController::odomAngleXAxis;
volatile bool Turtlebot3MotionController::updateVectorEncoderSlam;

geometry_msgs::Twist Turtlebot3MotionController::vel_msg;
ros::Rate *Turtlebot3MotionController::p_ResolutionRate;

volatile automotive_robot::Point Turtlebot3MotionController::cposSlam;
volatile automotive_robot::Point Turtlebot3MotionController::cposEncoder;
volatile automotive_robot::Point Turtlebot3MotionController::encoderSlam;

ros::NodeHandle *Turtlebot3MotionController::p_RobotControllerNode;

ros::Publisher Turtlebot3MotionController::velocity_publisher;
ros::Publisher Turtlebot3MotionController::moveFinished_publisher;
ros::Publisher Turtlebot3MotionController::updateSlamPos_publisher;
ros::Publisher Turtlebot3MotionController::doneChangeView_publisher;

ros::Subscriber Turtlebot3MotionController::Odometry_sub;
ros::Subscriber Turtlebot3MotionController::cmd_moving_sub;
ros::Subscriber Turtlebot3MotionController::changeView_sub;

Turtlebot3_RotateController Turtlebot3MotionController::rotateController;
Turtlebot3_MovingController Turtlebot3MotionController::movingController;

volatile double z;                    // robot actual angular speed
volatile double actual_odom_velocity; // robot actual linear speed

// return true if rotate right, else return false if rotate left
inline bool Turtlebot3MotionController::motion_determine_direction(const automotive_robot::Point *next_point, double &direction)
{
    direction = Direction(&cposEncoder, next_point);
    double angleDiff = direction - odomAngleXAxis;
    if (odomAngleXAxis > 0 && M_PI < (odomAngleXAxis - direction))
        angleDiff = 2 * M_PI - odomAngleXAxis + direction;
    else if (odomAngleXAxis < 0 && M_PI < (-odomAngleXAxis + direction))
        angleDiff = -2 * M_PI + direction + odomAngleXAxis;
    return ((angleDiff < 0) ? true : false);
}

void Turtlebot3MotionController::setFields(int argc, char **argv)
{
    updateVectorEncoderSlam = false;
    p_RobotControllerNode = new ros::NodeHandle;
    p_ResolutionRate = new ros::Rate(RESOLUTION_RATE);

    velocity_publisher = p_RobotControllerNode->advertise<geometry_msgs::Twist>("cmd_vel", 20);
    moveFinished_publisher = p_RobotControllerNode->advertise<std_msgs::String>("update_vision", 20);
    updateSlamPos_publisher = p_RobotControllerNode->advertise<std_msgs::String>("updateSlamPosCpp", 20);
    doneChangeView_publisher = p_RobotControllerNode->advertise<std_msgs::String>("done_change_view", 20);

    rotateController.setField(p_ResolutionRate, &velocity_publisher, &vel_msg, &odomAngleXAxis);
    movingController.setField(p_ResolutionRate, &velocity_publisher, &vel_msg, &cposEncoder, &odomAngleXAxis);

    cout << "\nWaiting for all register publishes stablization (10 seconds)\n";
    for (int i = 0; i < 10; i++)
    {
        sleep(1);
        cout << i + 1 << endl;
    }

    Odometry_sub = p_RobotControllerNode->subscribe("odom", 20, odomCallback);
    cmd_moving_sub = p_RobotControllerNode->subscribe("cmd_moving", 20, moveCallback);
    changeView_sub = p_RobotControllerNode->subscribe("request_change_view", 20, changeViewCallback);
}

void Turtlebot3MotionController::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (updateVectorEncoderSlam)
    {
        encoderSlam.x = cposSlam.x - msg->pose.pose.position.x;
        encoderSlam.y = cposSlam.y - msg->pose.pose.position.y;
        updateVectorEncoderSlam = false;
    }
    odomAngleXAxis = yaw;
    cposEncoder.x = msg->pose.pose.position.x + encoderSlam.x;
    cposEncoder.y = msg->pose.pose.position.y + encoderSlam.y;
    // z = msg->twist.twist.angular.z;
    // actual_odom_velocity = msg->twist.twist.linear.x;
}

void Turtlebot3MotionController::changeViewCallback(const std_msgs::String::ConstPtr &msg)
{
    cout << "changeViewCallBack: message received = *" << msg->data << "*\n";

    /* uncomment 2 lines below to treat the received goal as a relative goal.*/
    // moveFinished_publisher.publish(*msg);
    // return;

    static bool receivedGoal = false;
    if (!receivedGoal)
    {
        receivedGoal = true;
        std_msgs::String done_msg;
        done_msg.data = "goal:" + msg->data + ";Angle:" + to_string(odomAngleXAxis);
        doneChangeView_publisher.publish(done_msg);
        return;
    }

    if (!strcmp(msg->data.c_str(), "Not found Paper!"))
    {
        cout << "changeViewCallback: Not found Paper!\n";
        float nextView = odomAngleXAxis - CAMERA_VISION;
        rotateController.rotate(true, nextView + ((nextView <= -M_PI) ? 2 * M_PI : 0));
        sleep(1);
        std_msgs::String done_msg;
        done_msg.data = "odom:" + to_string(odomAngleXAxis);
        doneChangeView_publisher.publish(done_msg);
    }
    else
        moveFinished_publisher.publish(*msg);
    cout << "OdomAngelXAxis:" << odomAngleXAxis / M_PI * 180 << "\n\n";
}

void Turtlebot3MotionController::moveCallback(const automotive_robot::Path::ConstPtr &msg)
{
    cposSlam.x = msg->points[0].x;
    cposSlam.y = msg->points[0].y;
    updateVectorEncoderSlam = true;
    usleep(250000);

    cout << "received trigger message, should showing path.\n";
    unsigned loops = msg->points.size();
    for (int i = 0; i < loops; i++)
    {
        cout << msg->points[i];
    }
    cout << endl;

    cout << ((loops > 2) ? "Move according to ASP path\n" : "Move 1 step ahead\n");
    double direction;
    for (int i = 1; i < loops; i++)
    {
        printf("\nCPOS: [%f : %f]", cposEncoder.x, cposEncoder.y);
        printf("   Next Point: [%f : %f]\n", msg->points[i].x, msg->points[i].y);

        bool rotate_right = motion_determine_direction(&msg->points[i], direction);
        rotateController.rotate(rotate_right, direction);
        if (distance(&cposEncoder, &msg->points[i]) < MOVING_STABLE_TOLERANCE)
        {
            vel_msg.linear.x = 0.10;
            movingController.moving_very_slow(&msg->points[i]);
            movingController.stop_moving_stop_rotating();
        }
        else
            movingController.moving_to_next_point(&msg->points[i]);

        if (i == loops - 1)
            break;

        sleep(6);
        std_msgs::String dum_msg;
        dum_msg.data = "done";
        updateSlamPos_publisher.publish(dum_msg);
        usleep(2000 * 1000);

        ifstream MyReadFile("~/catkin_ws/src/automotive_robot/src/automotive_robot/logSlamOdomCpp.log");
        cout << "fished writting\n";
        string myText;

        while (getline(MyReadFile, myText))
        {
            if (myText.find("Translation") != size_t(-1))
            {
                float temp;
                sscanf(myText.c_str(), "- Translation: [%f, %f, %f]", &cposSlam.x, &cposSlam.y, &temp);
                cout << "Odom Slam got: [" << cposSlam.x << "," << cposSlam.y << "]\n";
                break;
            }
        }
        MyReadFile.close();

        updateVectorEncoderSlam = true;
        usleep(250000);
    }

    cout << "Done moving to next point!!!\n\n";

    /********************************************************************************************************/
    /* functions for smooth asp movement

    else {
        cout<<"Co nhieu duong gap khuc"<<endl;

        // moving_accelerate(msg->points[1]);

        vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum bc velocity
                                        // might not be maximum
        vel_msg.angular.z = 0; // force stop rotating bc the robot might still be rotating
        velocity_publisher.publish(vel_msg);

        // stable moving
        loops--;    // pay attention please
        for(unsigned i = 1 ; i < loops; i++){
            automotive_robot::Point next_point = msg->points[i];
            float start_bending_distance = 0.15*distance(msg->points[i - 1], next_point);
            automotive_robot::Point start_bending_point;
            automotive_robot::Point stop_bending_point;
            float vector_nextPoint_to_start_bending_point[2];
            float vector_nextPoint_to_stop_bending_point[2];

            calculate_start_bending_vector_and_stop_bending_vector(msg, vector_nextPoint_to_start_bending_point, vector_nextPoint_to_stop_bending_point, start_bending_distance,i);
            translation_of_point(vector_nextPoint_to_start_bending_point, msg->points[i], start_bending_point);
            translation_of_point(vector_nextPoint_to_stop_bending_point, msg->points[i], stop_bending_point);

            vel_msg.linear.x = MAX_MV_SPEED; // force velocity to be maximum bc velocity
            // might not be maximum
            stop_rotating();

            stable_moving(next_point, start_bending_distance);

            stop_rotating();
            // be huong voi ban kinh cong R
            float alpha = acos((vector_nextPoint_to_start_bending_point[0]*vector_nextPoint_to_stop_bending_point[0]
            + vector_nextPoint_to_start_bending_point[1]*vector_nextPoint_to_stop_bending_point[1]) / (start_bending_distance*start_bending_distance));

            float R = tan(alpha/2)*start_bending_distance;

            automotive_robot::Point direction_forcing_point = msg->points[i+1];
            double direction;
            bool rotate_right = determine_rotate_direction(direction_forcing_point, direction);

            vel_msg.angular.z = ((rotate_right) ? -1 : 1)*MAX_MV_SPEED/R;     // v = R*W;
            velocity_publisher.publish(vel_msg);    // start bending
            direction = Direction(next_point, direction_forcing_point);
            while(abs(odomAngleXaxis - direction) > close_rotate_tolerance_while_moving){}

            stop_rotating();    // stop bending
        }

        automotive_robot::Point last_point = msg->points[loops];
        stable_moving(last_point, stable_moving_tolerance);
        stop_rotating();

        moving_decelerate(last_point);
        stop_rotating();

        very_slow_moving(last_point);

        stop_moving_stop_rotating();
        ROS_INFO("Moving done!");
    }
    /*******************************************************************************************************/

    std_msgs::String done_msg;
    done_msg.data = "done";
    moveFinished_publisher.publish(done_msg);
}

void Turtlebot3MotionController::robotMotionControllerSpin()
{
    ros::MultiThreadedSpinner Spinner(3);
    Spinner.spin();
    ros::spin();
    ros::waitForShutdown();
}
