#include "ros/ros.h"
//ROS messages
#include "std_msgs/Float32.h"
#include "perception/perception_data.h"
//OpenCV
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp> //To calculate optical flow (using calcOpticalFlowPyrLK())

//Optical flow:
//	Use example: https://docs.opencv.org/3.4/d7/d8b/tutorial_py_lucas_kanade.html
//	Documentation: https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323

cv::Point centroid_to_pub = cv::Point(0, 0);
float area_to_pub = 0.0;
cv::Rect bounding_box;
int confidence = 0;

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
		//Permissive: (20, 0, 0)-(80, 255, 255); Strict: (50, 100, 100)-(60, 255, 125)
		cv::Mat thresholded_img;
		cv::Scalar min_green = cv::Scalar(50, 100, 100); //Minimum values considered "green" 		
		cv::Scalar max_green = cv::Scalar(60, 255, 125); //Maximum values considered "green" 
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

		if (contours.size() > 0)
		{
			//Store biggest contour in a variable in order not to access an index several times
			std::vector<cv::Point> biggest_cnt = contours[biggest_idx];
	
			//Calculate bounding rectangle for the biggest area
			bounding_box = cv::boundingRect(biggest_cnt);
	
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
	
			//Update global variables (to publish them in the main loop)
			centroid_to_pub = centroid;
			area_to_pub = biggest_area;
			confidence = 100;
		}
		else //IF NO CONTOURS WERE FOUND
		{
			//Update global variables (to publish them in the main loop)
			bounding_box = cv::Rect(0,0,0,0);
			centroid_to_pub = cv::Point(0,0);
			area_to_pub = 0;
			confidence = 0;
		}
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("[!] ERROR! Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}



int main(int argc, char **argv)
{
	//ROS setup
	ros::init(argc, argv, "perception");
	ros::NodeHandle n;

	//OpenCV setup
	cv::namedWindow("green_detection");
	cv::startWindowThread();

	//Image subscriber
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

	//Centroid (x and y) and area publisher (z)
	ros::Publisher perception_pub = n.advertise<perception::perception_data>("perception_data", 10);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		//Create message
		perception::perception_data msg;
		
		msg.centroid_x = centroid_to_pub.x;
		msg.centroid_y = centroid_to_pub.y;
		
		msg.area = area_to_pub;

		msg.bb_x = bounding_box.x;
		msg.bb_y = bounding_box.y;
		msg.bb_width = bounding_box.width;
		msg.bb_height = bounding_box.height;

		msg.confidence = confidence;

		//Publish
		perception_pub.publish(msg);

		//Spin and wait
		ros::spinOnce();
		loop_rate.sleep();
	}

	cv::destroyWindow("green_detection");

	return 0;
}

