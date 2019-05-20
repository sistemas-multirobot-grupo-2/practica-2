#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
move_base_msgs::MoveBaseGoal position;
/*
	This contains the position coordinates for the navigation positions
*/
struct positions
{
    //-HOME-//
    double initPosX = 1.35;
    double initPosY = -0.15;

    //-position 1-//
    double pose1X = -2.65;
    double pose1Y = 4.5;
    double or1w = 0.97;
    double or1z = -0.24;
    //-position 2-//
    double pose2X = 1.78;
    double pose2Y = 0.09;
    double or2w = 0.99;
    double or2z = -0.32;
    //-position 3-//
    double pose3X = -1.93;
    double pose3Y = 0.97;
    double or3w = -0.07;
    double or3z = 1.0;
    //-position 4-//
    double pose4X = 1.84;
    double pose4Y = -0.01;
    double or4w = -0.45;
    double or4z = 0.9;
} pos;

/*
	This function recieves a number for position selection and moves the robot to such position
	if battery is suficient to acomplish the mission according to battery safe levels. It
	returns true if reached, false if it cannot reach such position due to an anomaly in the navigation
	If it can't reach target goal, it returns home by default.
*/
bool move2pos(int target){
	MoveBaseClient ac("move_base", true);
  switch(target) { //Changes position coordinates to match
    case 0 : //We move to  home base
      position.target_pose.pose.position.x = pos.initPosX;
      position.target_pose.pose.position.y = pos.initPosY;
      position.target_pose.pose.orientation.w = 1.0;
      position.target_pose.pose.orientation.z = 0.0;

      cout << "No valid position selected, returning home." << endl;
    break;
		case 1 :
			position.target_pose.pose.position.x = pos.pose1X;
			position.target_pose.pose.position.y = pos.pose1Y;
      position.target_pose.pose.orientation.w = pos.or1w;
      position.target_pose.pose.orientation.z = pos.or1z;

    	ac.sendGoal(position);
      ac.waitForResult();
      position.target_pose.pose.position.x = pos.pose2X;
			position.target_pose.pose.position.y = pos.pose2Y;
      position.target_pose.pose.orientation.w = pos.or2w;
      position.target_pose.pose.orientation.z = pos.or2z;
		break;

		case 2 :

			position.target_pose.pose.position.x = pos.pose3X;
			position.target_pose.pose.position.y = pos.pose3Y;
      position.target_pose.pose.orientation.w = pos.or3w;
      position.target_pose.pose.orientation.z = pos.or3z;
      ac.sendGoal(position);
      ac.waitForResult();
      position.target_pose.pose.position.x = pos.pose4X;
			position.target_pose.pose.position.y = pos.pose4Y;
      position.target_pose.pose.orientation.w = pos.or4w;
      position.target_pose.pose.orientation.z = pos.or4z;
		break;
    case 9 :
    break;
		default:
			position.target_pose.pose.position.x = pos.initPosX;
			position.target_pose.pose.position.y = pos.initPosY;
      position.target_pose.pose.orientation.w = 1.0;
      position.target_pose.pose.orientation.z = 0.0;
			cout << "No valid position selected, returning home." << endl;
		break;
	}
  ac.sendGoal(position);
  ac.waitForResult();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //tell the action client that we want to spin a thread greenY default
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  int target = 0;
	//configuration of main position values
  position.target_pose.header.frame_id = "map";
  position.target_pose.header.stamp = ros::Time::now();
	//Automated program
  while (target!=9){
    cout << "Movement desired: home(0) p2p(1) t2p(2) shutdown(9) ";
		cin >> target;
    move2pos(target);
  }
  return 0;
}
