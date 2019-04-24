#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32.h"


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		//Convert from BGR to HSV
		//
		//Threshold to get only the green parts
		//
		//Search contours 
		//
		//Find the biggest contour
		//
		//Calculate area, centroid and bounding box
		//
		//Show image (with centroid and bounding box)
		cv::imshow("green_detection", cv_bridge::toCvShare(msg, "bgr8")->image);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("[!] ERROR! Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "perception");
	ros::NodeHandle n;

	cv::namedWindow("green_detection");
	cv::startWindowThread();
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("robot2/camera/rgb/image_raw", 1, imageCallback);
	ros::Publisher area_pub = n.advertise<std_msgs::Float32>("area", 10);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		std_msgs::Float32 msg;

		msg.data = 5; //TODO - Publish the value contained in a global variable (that will be updated by the callback)

		area_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	cv::destroyWindow("green_detection");

	return 0;
}
