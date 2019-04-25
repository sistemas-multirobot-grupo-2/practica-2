#include "ros/ros.h"
//ROS messages
#include "std_msgs/Float32.h"
#include "perception/perception_data.h"

void perceptionCallback(const perception::perception_data::ConstPtr& msg)
{
	std::cout << "[*] I've received information!" << std::endl;
	
	std::cout << "  + Centroid coordinates: [" << msg->centroid_x << "; " << msg->centroid_y << "]" << std::endl;
	std::cout << "  + Green area detected: " << msg->area << std::endl;
	std::cout << "  + Bounding box coordinates: [" << msg->bb_x << "; " << msg->bb_y << "]" << " [" << msg->bb_x+msg->bb_width << "; " << msg->bb_y+msg->bb_height << "]" << std::endl;
	std::cout << "  + Confidence in this data: " << msg->confidence << std::endl << std::endl << std::endl;
}


int main(int argc, char **argv)
{
	//ROS setup
	ros::init(argc, argv, "perception_tester");
	ros::NodeHandle n;

	//Perception data subscriber
	ros::Subscriber perception_sub = n.subscribe("perception_data", 10, perceptionCallback);

	ros::spin();

	return 0;
}

