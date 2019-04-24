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
		//Convert from ROS image message to OpenCV image
		cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

		//Convert from BGR to HSV
		cv::Mat img_hsv;
		cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

		//Threshold to get only the green parts
		cv::Mat thresholded_img;
		cv::Scalar min_green = cv::Scalar(20, 0, 0); //Minimum values considered "green"
		cv::Scalar max_green = cv::Scalar(80, 255, 255); //Maximum values considered "green"
		inRange(img_hsv, min_green, max_green, thresholded_img);

		
		//Search contours 
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		findContours(thresholded_img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		//Find the biggest contour (by measuring their bounded areas)
		int biggest_idx = 0;
		double biggest_area = 0.0;

		for (int i=0; i<contours.size(); i++)
		{
			double current_area = cv::contourArea(contours[i]);

			if (current_area > biggest_area)
			{
				biggest_idx = i;
				biggest_area = current_area;
			}
		}

		std::vector<cv::Point> biggest_cnt = contours[biggest_idx];

		//Calculate bounding rectangle for the biggest area
		cv::Rect bounding_box = cv::boundingRect(biggest_cnt);

		//Calculate the centroid of the biggest area
		cv::Moments mu = cv::moments(biggest_cnt);
		cv::Point2f centroid = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00 ); //[m10/m00; m01/m00]
		
		//Draw centroid and bounding box
		cv::circle(img, centroid, 2, cv::Scalar(0, 0, 255), -1, 8, 0 ); //Centroid
		cv::rectangle(img,
				cv::Point(bounding_box.x, bounding_box.y),
				cv::Point(bounding_box.x + bounding_box.width, bounding_box.y + bounding_box.height),
				cv::Scalar(0, 0, 255)); //Bounding box
		
		//Show image (with centroid and bounding box)
		cv::imshow("green_detection", img);
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
