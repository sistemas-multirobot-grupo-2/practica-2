#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include "std_msgs/String.h"

using namespace std;

/*
	This contains the position coordinates for the navigation positions
*/
struct positions
{
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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class leader_position
{
private:
  // membres of the class
  ros::Subscriber nextLeaderPositionSub;
  move_base_msgs::MoveBaseGoal position;
public:
  ros::Publisher leaderPositionPub;
  // member functions of the class
  // contructor
  leader_position(ros::NodeHandle& nh)
  {
    //configuration of main position values
    position.target_pose.header.frame_id = "map";
    position.target_pose.header.stamp = ros::Time::now();
    position.target_pose.pose.orientation.w = 1.0;
    // publisher
    leaderPositionPub = nh.advertise<std_msgs::String>("leader_position", 1000);
    // subscriber
    nextLeaderPositionSub = nh.subscribe("next_leader_position", 1000, &leader_position::move2posCallBack, this);
  }

  // subscriber callback
  void move2posCallBack(const std_msgs::String::ConstPtr& msg)
  {
    //cout << msg->data << endl;
    cout << "entra" << endl;
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    //wait for the action server to come up
    string message = msg->data;

    cout << message << endl;
    std::stringstream ss;
    std_msgs::String msg2;

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
        ss << "home";
        msg2.data = "home";
        do {
          leaderPositionPub.publish(msg2);
        } while(!leaderPositionPub.getNumSubscribers() > 0);
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
        ss << "pose1";
        msg2.data = "pose1";
        do {
          leaderPositionPub.publish(msg2);
        } while(!leaderPositionPub.getNumSubscribers() > 0);
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
        ss << "pose2";
        msg2.data = "pose2";
        do {
          leaderPositionPub.publish(msg2);
        } while(!leaderPositionPub.getNumSubscribers() > 0);
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
        ss << "pose3";
        msg2.data = "pose3";
        do {
          leaderPositionPub.publish(msg2);
        } while(!leaderPositionPub.getNumSubscribers() > 0);
      }
      else
        cout << "Position could not be reached, attempting to rescue TurtleBot... sending it back home..." << endl;
    }
  }
};//End of class leaderPosition


int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation");
  ros::NodeHandle nh;
  leader_position leadPos(nh);
  ros::Rate loop_rate(1);

  std_msgs::String msg; // create the message to send
  std::stringstream ss;
  ss << "home";
  msg.data = ss.str();
  ROS_INFO("%s", msg.data.c_str());
  do {
    leadPos.leaderPositionPub.publish(msg);
  } while(!leadPos.leaderPositionPub.getNumSubscribers() > 0);


  cout << "cff" << endl;
  ros::spinOnce();
  cout << "sended" << endl;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}
