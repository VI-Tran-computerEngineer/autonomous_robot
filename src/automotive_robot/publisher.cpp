#include "ros/ros.h"
#include "std_msgs/String.h"

#include "automotive_robot/Path.h"
#include "automotive_robot/Point.h"

#include <iostream>
#include <sstream>
#include <vector>

using namespace std;
using namespace automotive_robot;

Point s_p;
ros::Publisher chatter_pub;

void callback(const std_msgs::String::ConstPtr &msg) {
  cout << "I heard: " << msg->data << endl;
  Point p;
  cout << "Input please:" << endl;
  cin >> p.x >> p.y;
  vector<Point> v;
  v.push_back(s_p);
  v.push_back(p);
  chatter_pub.publish(v);
  s_p.x = p.x;
  s_p.y = p.y;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "publisher");

  ros::NodeHandle n;
  chatter_pub = n.advertise<automotive_robot::Path>("cmd_moving", 1000);

  // In Hz
  s_p.x = -2;
  s_p.y = -0.5;
  ros::Subscriber sub = n.subscribe("update_vision", 1000, callback);
    ros::spin();
  return 0;
}
