#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>

using namespace std;

move_base_msgs::MoveBaseGoal position;
std_msgs::String msg2;



void nextLeaderPositionCallback(const std_msgs::String::ConstPtr& msg)
{
  string message = msg->data.c_str();
  std::stringstream ss;
  
  if(message == "home")
  {
    ss << "pose1";
    msg2.data = ss.str();
	}
  else if(message == "pose1")
  {
    ss << "pose2";
    msg2.data = ss.str();
  }
  else if(message == "pose2")
  {
    ss << "pose3";
    msg2.data = ss.str();
  }
  else if(message == "pose3")
  {
    ss << "pose4";
    msg2.data = ss.str();
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planning");
    ros::NodeHandle n;
    // publisher definition
    ros::Publisher next_leader_position_pub = n.advertise<std_msgs::String>("next_leader_position", 1000);
    ros::Rate loop_rate(1);
    // subscriber definition
    ros::Subscriber leader_position_sub = n.subscribe("leader_position", 1000, nextLeaderPositionCallback);
    while (ros::ok())
    {
        ros::spinOnce();
        next_leader_position_pub.publish(msg2);
        loop_rate.sleep();
    }
}

