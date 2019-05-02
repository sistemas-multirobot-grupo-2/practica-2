#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>

#include "std_msgs/String.h"

using namespace std;


class Planning
{
    private:
    ros::Publisher next_leader_position_pub_;
    ros::Subscriber leader_position_sub_;
    
    move_base_msgs::MoveBaseGoal position_;
    std_msgs::String next_pose_;
    
    public:
    Planning(ros::NodeHandle& nh)
    {
        next_leader_position_pub_ = nh.advertise<std_msgs::String>("/next_leader_position", 1000);
        leader_position_sub_ = nh.subscribe("/leader_position", 1000, &Planning::nextLeaderPositionCallback,this);
    }
    
    void nextLeaderPositionCallback(const std_msgs::String::ConstPtr& msg)
    {
        string message = msg->data.c_str();
        std::stringstream ss;

        if(message == "home")
        {
            ss << "pose1";
            next_pose_.data = ss.str();
        }
        else if(message == "pose1")
        {
            ss << "pose2";
            next_pose_.data = ss.str();
        }
        else if(message == "pose2")
        {
            ss << "pose3";
            next_pose_.data = ss.str();
        }
        else if(message == "pose3")
        {
            ss << "pose4";
            next_pose_.data = ss.str();
        }
    };
    
    void loop()
    {
        ros::Rate rate(1);
        while (ros::ok())
        {
            next_leader_position_pub_.publish(next_pose_);
            
            ros::spinOnce();
            rate.sleep();
        }
    };  
         
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planning");
	ros::NodeHandle n;
	Planning* plan = new Planning(n);
	plan->loop();
	
	delete(plan);
	return 0;
};

