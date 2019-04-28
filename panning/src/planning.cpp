#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include "std_msgs/String.h"

using namespace std;

move_base_msgs::MoveBaseGoal position;

ros::init(argc, argv, "planning");
ros::NodeHandle n;
// publisher definition
ros::Publisher next_leader_position_pub = n.advertise<std_msgs::String>("next_leader_position", 1000);
ros::Rate loop_rate(1);


void nextLeaderPositionCallback(const std_msgs::String::ConstPtr& msg)
{ 
  string message= msg->data.c_str();
  std::stringstream ss;
  switch (message)
  {
    case "home":
      ss << "pose1";
      msg->data = ss.str();
      next_leader_position_pub.publish(msg);
    break;
    case "pose1":
      ss << "pose2";
      msg->data = ss.str();
      next_leader_position_pub.publish(msg);
    break;
    case "pose2":
      ss << "pose3";
      msg->data = ss.str();
      next_leader_position_pub.publish(msg);
    break;
    case "pose3":
      ss << "home";
      msg->data = ss.str();
      next_leader_position_pub.publish(msg);
    break;
    default:
    break;
  }
}

int main(int argc, char** argv)
{
  // subscriber definition
  ros::Subscriber leader_position_sub = n.subscribe("leader_position", 1000, nextLeaderPositionCallback);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
