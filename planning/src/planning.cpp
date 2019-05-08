#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include "std_msgs/String.h"

using namespace std;


/*
 * Planning: This class uses the information about current posisition
 *
 */
class Planning
{
    private:
    ros::Publisher next_leader_position_pub_;
    ros::Subscriber leader_position_sub_;

    move_base_msgs::MoveBaseGoal position_;
    std_msgs::String next_pose_;

    public:
    /*
     * Class constructor
     * param nh: Is the node handle created with the init node
     */
    Planning(ros::NodeHandle& nh)
    {
        next_leader_position_pub_ = nh.advertise<std_msgs::String>("next_leader_position", 1000);
        leader_position_sub_ = nh.subscribe("leader_position", 1000, &Planning::nextLeaderPositionCallback,this);
        cout << "subscribed" << endl;
    }
    /*
     * Class Method and Topic Callback: /leader_position
     * param msg: Is the data, published by other node, of our current position
     */
    void nextLeaderPositionCallback(const std_msgs::String::ConstPtr& msg)
    {
        cout << "entra" << endl;
        string message = msg->data;
        cout << message << endl;
        std::stringstream ss;
        std::string s;

        if(message == "home")
        {
            ss << "pose1";
            s = ss.str();
            next_pose_.data = s;
        }
        else if(message == "pose1")
        {
            ss << "pose2";
            s = ss.str();
            cout << s << endl;
            next_pose_.data = s;
        }
        else if(message == "pose2")
        {
            ss << "pose3";
            s = ss.str();
            cout << s << endl;
            next_pose_.data = s;
        }
        else if(message == "pose3")
        {
            ss << "home";
            s = ss.str();
            cout << s << endl;
            next_pose_.data = s;
        }
        do {
          next_leader_position_pub_.publish(next_pose_);
        } while(!next_leader_position_pub_.getNumSubscribers() > 0);


    }
    /*
     * Class Method and ros loop: This loop listen the callbacks and publish the information about
     * the next leader position with some rate
     */
    void loop()
    {
        ros::Rate rate(1);
        while (ros::ok())
        {
            //cout << "working" << endl;
            ros::spinOnce();
            rate.sleep();
        }
    }

};

/*
 * Main Function: Initialize the node and creates an object of the class.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "planning");
	ros::NodeHandle n;
	Planning* plan = new Planning(n);
	plan->loop();

	delete(plan);
	return 0;
};
