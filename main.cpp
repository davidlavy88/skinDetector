/*
 * main.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: david
 */


//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>

namespace enc = sensor_msgs::image_encodings;

class skinDetector {
protected:
	//Initialize ROS node
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	sensor_msgs::CvBridge bridge_;
	cv_bridge::CvImagePtr cv_ptr_;
	//cv_bridge::CvImagePtr cv_out_;
	cv_bridge::CvImage cv_out_;
	cv::Mat img_hsv_, img_out, img_bin, img_morph;

public:
	skinDetector(ros::NodeHandle &nh): nh_(nh), it_(nh_)
	{
		// Advertise image messages to a topic
		image_pub_ = it_.advertise("/camera/image_hsv", 1);
		// Listen for image messages on a topic and setup callback
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &skinDetector::imageCallback, this);
		// Open HighGUI Window
		cv::namedWindow ("hsv", 1);
	}

	void imageCallback (const sensor_msgs::ImageConstPtr& msg_ptr) {
		try{
			cv_ptr_ = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);
		}
		catch (sensor_msgs::CvBridgeException error) {
			ROS_ERROR("CvBridge input error");
		}

		//cv_out_ = cv_ptr_;
		img_out = cv_ptr_->image.clone();
		cv::cvtColor(cv_ptr_->image,img_hsv_, CV_BGR2HSV);
		std::vector<cv::Mat> channels;
		img_bin = cv::Mat::zeros(img_hsv_.rows,img_hsv_.cols,CV_8U);
		cv::split(img_hsv_,channels);

		for(int j = 0; j < img_out.rows; j++) {

			uchar* data_h = channels[0].ptr<uchar>(j);
			uchar* data_s = channels[1].ptr<uchar>(j);
			uchar* data_v = channels[2].ptr<uchar>(j);
			for(int i = 0; i < img_out.cols; i++) {

				if (data_h[i] > 6 && data_h[i] < 38 &&
						data_s[i] > 58 && data_s[i] < 173 &&
						data_v[i] > 89 && data_v[i] < 229) img_bin.at<uchar>(j,i) = 255;
				else {
					img_bin.at<uchar>(j,i) = 0;
					img_out.at<uchar>(j,3*i + 0) = 0;
					img_out.at<uchar>(j,3*i + 1) = 0;
					img_out.at<uchar>(j,3*i + 2) = 0;
				}
			}
		}

		cv::Size strel_size;
		strel_size.width = 3;
		strel_size.height = 3;
		cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,strel_size);
		cv::morphologyEx(img_bin,img_morph,cv::MORPH_OPEN,strel,cv::Point(-1, -1),3);

		cv_out_.image = img_out;
		cv_out_.encoding = enc::BGR8;

		cv::imshow("hsv",cv_out_.image);
		cv::imshow("binary",img_bin);
		cv::imshow("morphed output",img_morph);
		cv::waitKey(3);

		/*cv_out_.reset(new cv_bridge::CvImage);
		//cv_out_.header = cv_ptr_->header;
		//cv_out_.encoding = enc::BGR8;
		cv_out_->image = img_hsv_;*/
		image_pub_.publish(cv_out_.toImageMsg());
	}
};

int main(int argc, char** argv) {
	//Initialize ROS node
	ros::init(argc,argv,"skinDetection");
	ros::NodeHandle nh;
	skinDetector sk(nh);
	ros::spin();

	return 0;
}
