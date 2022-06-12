//#include "robot_motion_node.hpp"
#include "automotive_robot/Path.h"
#include "automotive_robot/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <math.h>
#include <string.h>
#include <string>
#include <unistd.h>
#include <vector>

#define MAX_MV_SPEED 0.18 // square_unit/s
//#define CLOSE_MV_SPEED 0.05                              // square_unit/s
#define STABLE_ANGULAR_SPEED ((double(20) / 180) * M_PI) // radian/s
#define CLOSE_ANGULAR_SPEED ((double(10) / 180) * M_PI)  // radian/s
#define MV_ANGULAR_SPEED ((double(5) / 180) * M_PI)      // radian/s

using namespace std;

ros::Publisher velocity_publisher;
ros::Publisher moveFinished_publisher;
geometry_msgs::Twist vel_msg;
volatile automotive_robot::Point cpos;
double odomAngleXaxis = 0;
// double c_pos[2] = {0, 0};
const double stable_moving_tolerance = 0.25 * 0.25;
const double close_moving_tolerance = 0.01;
const double stable_rotate_tolerance = 0.2;
const double close_rotate_tolerance = 0.0025;
const int n = 6;

void move(double distance, automotive_robot::Point &nextPt);
void rotate(double relative_angle, double direction);
void moveCallback(const automotive_robot::Path::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

// bool volatile flag_check_angle = true;

int main(int argc, char **argv) {
  // Initiate new ROS node named "talker"
  ros::init(argc, argv, "robot_motion");
  ros::NodeHandle n;

  velocity_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  moveFinished_publisher = n.advertise<std_msgs::String>("update_vision", 1000);

  //	/turtle1/cmd_vel is the Topic name
  //	/geometry_msgs::Twist is the msg type
  ROS_INFO("\n\n\n ******** START *********\n");

  ros::Subscriber sub = n.subscribe("cmd_moving", 1000, moveCallback);
  ros::Subscriber Odometry_sub = n.subscribe("odom", 1000, odomCallback);
  std_msgs::String triggered_msg;
  triggered_msg.data = "done";
  // for(int i = 0; i < 10; i++){
  ROS_INFO("\n\n\n Before sleep\n");
  sleep(20);
  ROS_INFO("\n\n\n Now Starting!\n");
  moveFinished_publisher.publish(triggered_msg);
  //}
  ros::MultiThreadedSpinner Spinner(2);

  Spinner.spin();
  ros::waitForShutdown();

  return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  odomAngleXaxis = yaw;
  cpos.x = msg->pose.pose.position.x;
  cpos.y = msg->pose.pose.position.y;
}

// direction from p1 to p2
double Direction(const volatile automotive_robot::Point &p1,
                 const automotive_robot::Point &p2) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;

  return atan2(dy, dx);
}

void move(double distance, automotive_robot::Point &point) {

  ROS_INFO("Moving start!");

  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  // set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;

  double vel = MAX_MV_SPEED;

  int i = 0;
  // double temp = stable_moving_tolerance + sin(abs(direction -
  // odomAngleXaxis)) * distance; temp *= temp;

  while ((cpos.x - point.x) * (cpos.x - point.x) +
             (cpos.y - point.y) * (cpos.y - point.y) >
         stable_moving_tolerance) {
    vel_msg.linear.x = vel;
    velocity_publisher.publish(vel_msg);
    usleep(1250000);
    vel_msg.linear.x = vel / 2;
    velocity_publisher.publish(vel_msg);
    usleep(500000);
    double direction = Direction(cpos, point);
    double angleDiff = direction - odomAngleXaxis;

    if (odomAngleXaxis > 0 && M_PI < (odomAngleXaxis - direction))
      angleDiff = 2 * M_PI - odomAngleXaxis + direction;
    else if (odomAngleXaxis < 0 && M_PI < (-odomAngleXaxis + direction))
      angleDiff = -2 * M_PI + direction + odomAngleXaxis;
    rotate(angleDiff, direction);
    cout << "-----------------" << endl;
    ROS_INFO("Direction: %d, [%f]", i + 1, direction / M_PI * 180);
    ROS_INFO("Odom Angle Axis: %d, [%f]", i + 1, odomAngleXaxis / M_PI * 180);
    i++;
  }

  distance = sqrt(pow(cpos.x - point.x, 2) + pow(cpos.y - point.y, 2) * 1.0);

  vel_msg.linear.x = vel / 3;
  velocity_publisher.publish(vel_msg);
  usleep(distance / vel * 1000000);

  double direction = Direction(cpos, point);
  double angleDiff = direction - odomAngleXaxis;

  if (odomAngleXaxis > 0 && M_PI < (odomAngleXaxis - direction))
    angleDiff = 2 * M_PI - odomAngleXaxis + direction;
  else if (odomAngleXaxis < 0 && M_PI < (-odomAngleXaxis + direction))
    angleDiff = -2 * M_PI + direction + odomAngleXaxis;
  rotate(angleDiff, direction);

  distance = sqrt(pow(cpos.x - point.x, 2) + pow(cpos.y - point.y, 2) * 1.0);

  vel_msg.linear.x = vel / 3;
  velocity_publisher.publish(vel_msg);
  usleep(distance / vel * 1000000);

  direction = Direction(cpos, point);
  angleDiff = direction - odomAngleXaxis;

  if (odomAngleXaxis > 0 && M_PI < (odomAngleXaxis - direction))
    angleDiff = 2 * M_PI - odomAngleXaxis + direction;
  else if (odomAngleXaxis < 0 && M_PI < (-odomAngleXaxis + direction))
    angleDiff = -2 * M_PI + direction + odomAngleXaxis;
  rotate(angleDiff, direction);

  cout << "-----------------" << endl;
  ROS_INFO("Direction finally: [%f]", direction / M_PI * 180);
  ROS_INFO("Odom Angle Axis finally: [%f]", odomAngleXaxis / M_PI * 180);

  // vel_msg.linear.x = CLOSE_MV_SPEED;
  // velocity_publisher.publish(vel_msg);
  // double temp1 = close_moving_tolerance + sin(abs(direction -
  // odomAngleXaxis)) * distance; temp1 *= temp1; while ((cpos.x - point.x) *
  // (cpos.x - point.x) + (cpos.y - point.y) * (cpos.y - point.y) > temp1) {
  //     continue;
  // }

  vel_msg.linear.x = 0.0000;
  // sai so do viec tang toc (0->MV speed) va sai so ve vi tri thuc su den (quan
  // tinh giam toc MV speed ->0) sai so ve toc do khi on dinh (XAP XI MV speed)
  velocity_publisher.publish(vel_msg);
  ROS_INFO("Moving done!");
}

// rotate clockwise
void rotate(double relative_angle, double direction) {
  // set a random linear velocity in the x-axis
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  // set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = ((relative_angle < 0) ? -1 : 1) * STABLE_ANGULAR_SPEED;

  velocity_publisher.publish(vel_msg);
  while (abs(odomAngleXaxis - direction) > stable_rotate_tolerance) {
    continue;
  }

  vel_msg.angular.z = ((relative_angle < 0) ? -1 : 1) * CLOSE_ANGULAR_SPEED;
  velocity_publisher.publish(vel_msg);
  double before = abs(odomAngleXaxis - direction);
  while (before > close_rotate_tolerance) {
    double after = abs(direction - odomAngleXaxis);
    if (after > before) {
      vel_msg.angular.z = -vel_msg.angular.z;
      velocity_publisher.publish(vel_msg);
    }
    before = after;
    continue;
  }

  vel_msg.angular.z = 0;
  // ROS_INFO("Odom Angle Axis After Rotate: [%f]", odomAngleXaxis / M_PI *
  // 180);
  velocity_publisher.publish(vel_msg);
  usleep(500000);
}

void moveCallback(const automotive_robot::Path::ConstPtr
                      &msg) // robot_motion::Path::ConstPtr& msg
{
  // ROS_INFO("I heard: [%s]", msg->data.c_str());
  unsigned loops = msg->points.size();
  ROS_INFO("I heard: [%f,%f]", msg->points[loops - 1].x,
           msg->points[loops - 1].y);

  for (unsigned i = 1; i < loops; i++) {
    //
    // cpos = msg->points[i - 1];
    automotive_robot::Point nextPt = msg->points[i];
    ROS_INFO("CPOS: [%f : %f]", cpos.x, cpos.y);
    ROS_INFO("Next Point: [%f : %f]", nextPt.x, nextPt.y);
    //
    double direction = Direction(cpos, nextPt);
    double angleDiff = direction - odomAngleXaxis;

    if (odomAngleXaxis > 0 && M_PI < (odomAngleXaxis - direction))
      angleDiff = 2 * M_PI - odomAngleXaxis + direction;
    else if (odomAngleXaxis < 0 && M_PI < (-odomAngleXaxis + direction))
      angleDiff = -2 * M_PI + direction + odomAngleXaxis;
    double distance =
        sqrt(pow(cpos.x - nextPt.x, 2) + pow(cpos.y - nextPt.y, 2) * 1.0);

    // ROS_INFO("Distance: [%f]", distance);
    ROS_INFO("Direction: %f", direction / M_PI * 180);

    //
    rotate(angleDiff, direction);
    ROS_INFO("Odom Angle Axis After Rotate: [%f]", odomAngleXaxis / M_PI * 180);
    move(distance, nextPt);
    sleep(1);
    ROS_INFO("Odom CPOS After Moving: [%f : %f]", cpos.x, cpos.y);
    ROS_INFO("Odom Angle Axis After Moving: [%f]", odomAngleXaxis / M_PI * 180);
    cout << endl << endl;
  }
  // cpos = msg->points[loops - 1];

  // cout << "Done Moving:" << endl;
  // system("Pause");
  // string temp;
  // cin >> temp;
  std_msgs::String done_msg;
  done_msg.data = "done";
  moveFinished_publisher.publish(done_msg);
}
