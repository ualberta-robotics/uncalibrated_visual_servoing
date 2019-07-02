/* 
 * lpetrich 07/06/19
 */

#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <vector>
#include "ros/ros.h"
#include <Eigen/Core>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking.hpp>
#include "uncalibrated_visual_servoing/Point2D.h"
#include "uncalibrated_visual_servoing/PointVector2D.h"


static const std::string OPENCV_WINDOW = "UVS";

class ImageConverter 
{
public:
	bool initialize_tracker;
	bool initialized_1;
	bool initialized_2;
	std::vector<std::string> tracker_types;
	std::vector<cv::Point> clicked_points;
	cv::Ptr<cv::MultiTracker> multiTracker;
	cv_bridge::CvImagePtr cv_ptr1;
	cv_bridge::CvImagePtr cv_ptr2;
	ros::Publisher error_pub;
	ros::Publisher eef_pub;

	ImageConverter(ros::NodeHandle nh_);
	~ImageConverter();
	cv::Ptr<cv::Tracker> createTrackerByName(std::string trackerType);
	void image_error(std::vector<cv::Rect2d> v);
	void spin();

	static void mouseHandleStatic(int event, int x, int y, int flags, void* that);
	void mouseHandle(int event, int x, int y, int flags);


private:
	image_transport::Subscriber cam1_sub;
	image_transport::Subscriber cam2_sub;

	void cam1Callback(const sensor_msgs::ImageConstPtr& msg)
	{
		try {
			cv_ptr1 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		if (cv_ptr1->image.rows > 60 && cv_ptr1->image.cols > 60) { 
			initialized_1 = true;
		}
		return;
	}

	void cam2Callback(const sensor_msgs::ImageConstPtr& msg)
	{
		try {
			cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		if (cv_ptr2->image.rows > 60 && cv_ptr2->image.cols > 60) {
			initialized_2 = true;
		}
		return;
	}
};

#endif // USER_INTERFACE_H