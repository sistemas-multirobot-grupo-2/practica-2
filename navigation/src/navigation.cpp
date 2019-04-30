#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include "std_msgs/String.h"

using namespace std;
ros::NodeHandle n;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal position;
// publisher definition
ros::Publisher leader_position_pub = n.advertise<std_msgs::String>("leader_position", 1000);
ros::Rate loop_rate(1);

/*
	This contains the position coordinates for the navigation positions
*/
struct positions{
	//-HOME-//
  double homeX = 1.35;
  double homeY = -0.15;
	//-position cargo-//
  double pose1X = -2.87;
  double pose1Y = 4.62;
	//-position green-//
	double pose2X = 11.09;
	double pose2Y = 1.97;
	//-position red-//
	double pose3X = 15.47;
	double pose3Y = -1.89;
} pos;


void move2posCallBack(const std_msgs::String::ConstPtr& msg){

  string message= msg->data.c_str();
  MoveBaseClient ac("move_base", true);
  if(message == "home")
  { //Changes position coordinates to match
    //We move to  home base
		position.target_pose.pose.position.x = pos.homeX;
		position.target_pose.pose.position.y = pos.homeY;
		cout << "Moving back to home." << endl;
    //Sends movement command
  	ac.sendGoal(position);
    ac.waitForResult();

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      cout << "Target position achieved." << endl << endl;
      leader_position_pub.publish(msg);
  	}
    else
    cout << "Position could not be reached, attempting to rescue TurtleBot... sending it back home..." << endl;
	}
  else if(message == "pose1")
  {
    position.target_pose.pose.position.x = pos.pose1X;
    position.target_pose.pose.position.y = pos.pose1Y;
    cout << "Moving to pose1." << endl;
    //Sends movement command
    ac.sendGoal(position);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      cout << "Target position achieved." << endl << endl;
      leader_position_pub.publish(msg);
    }
    else
      cout << "Position could not be reached, attempting to rescue TurtleBot... sending it back home..." << endl;
  }
  else if(message == "pose2")
  {
    position.target_pose.pose.position.x = pos.pose2X;
    position.target_pose.pose.position.y = pos.pose2Y;
    cout << "Moving to pose2." << endl;
    //Sends movement command
    ac.sendGoal(position);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      cout << "Target position achieved." << endl << endl;
      leader_position_pub.publish(msg);
    }
    else
      cout << "Position could not be reached, attempting to rescue TurtleBot... sending it back home..." << endl;
  }
  else if(message == "pose3")
  {
    position.target_pose.pose.position.x = pos.pose3X;
    position.target_pose.pose.position.y = pos.pose3Y;
    cout << "Moving to pose1." << endl;
    //Sends movement command
    ac.sendGoal(position);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      cout << "Target position achieved." << endl << endl;
      leader_position_pub.publish(msg);
    }
    else
      cout << "Position could not be reached, attempting to rescue TurtleBot... sending it back home..." << endl;
  }
}




int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::init(argc, argv, "navigation");


  //tell the action client that we want to spin a thread pose2Y default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

	//configuration of main position values
  position.target_pose.header.frame_id = "map";
  position.target_pose.header.stamp = ros::Time::now();
  position.target_pose.pose.orientation.w = 1.0;
	//Automated program
  //Subscribe to the topic "next_leader_position"
  ros::Subscriber next_leader_position_sub = n.subscribe("next_leader_position", 1000, move2posCallBack);

  //publish on the topic "leader_position" which is the actual pose of the robot
  std_msgs::String msg;
  std::stringstream ss;
  ss << "home";
  msg.data = ss.str();
  leader_position_pub.publish(msg);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}

