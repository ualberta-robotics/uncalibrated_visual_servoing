/* 
 * lpetrich 07/06/19
 */

#include "uncalibrated_visual_servoing/user_interface.h"

ImageConverter::ImageConverter(ros::NodeHandle nh_)
{ 
	image_transport::ImageTransport it_(nh_);
	tracker_types = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE"}; 
	multiTracker = cv::MultiTracker::create();
	initialize_tracker = false;
	initialized_1 = false;
	initialized_2 = false;
	clicked_points.clear();
	cv::namedWindow(OPENCV_WINDOW);
	cv::setMouseCallback(OPENCV_WINDOW, &ImageConverter::mouseHandleStatic, (void*)this);
	error_pub = nh_.advertise<uncalibrated_visual_servoing::PointVector2D>("image_error", 10);
	eef_pub = nh_.advertise<uncalibrated_visual_servoing::PointVector2D>("end_effector", 10);
	cam1_sub = it_.subscribe("/cam1/image_raw", 1, &ImageConverter::cam1Callback, this);
	cam2_sub = it_.subscribe("/cam2/image_raw", 1, &ImageConverter::cam2Callback, this);
}

ImageConverter::~ImageConverter()
{ 
	cam1_sub.shutdown();
	cam2_sub.shutdown();
}

void ImageConverter::mouseHandleStatic(int event, int u, int v, int flags, void* param )
{
	ImageConverter* thiz = static_cast<ImageConverter*>(param);
	thiz->mouseHandle(event, u, v, flags);
}


void ImageConverter::mouseHandle( int event, int u, int v, int flags)
{
	switch (event) {
		case cv::EVENT_LBUTTONDOWN:
			// std::cout << "Left Button Down at: " << u << " " << v << std::endl;
			clicked_points.push_back(cv::Point(u, v));
			if (clicked_points.size() == 2) {
				initialize_tracker = true;
			}
			break;
		case cv::EVENT_RBUTTONDOWN: 
			clicked_points.clear();
			// std::cout << "Right Button Down at: " << x << " " << y << std::endl;
			break;
	}
}

cv::Ptr<cv::Tracker> ImageConverter::createTrackerByName(std::string trackerType) 
{
	cv::Ptr<cv::Tracker> tracker;
	if (trackerType ==  tracker_types[0])
		tracker = cv::TrackerBoosting::create();
	else if (trackerType == tracker_types[1])
		tracker = cv::TrackerMIL::create();
	else if (trackerType == tracker_types[2])
		tracker = cv::TrackerKCF::create();
	else if (trackerType == tracker_types[3])
		tracker = cv::TrackerTLD::create();
	else if (trackerType == tracker_types[4])
		tracker = cv::TrackerMedianFlow::create();
	else if (trackerType == tracker_types[5])
		tracker = cv::TrackerGOTURN::create();
	else if (trackerType == tracker_types[6])
		tracker = cv::TrackerMOSSE::create();
	else {
		std::cout << "Incorrect tracker name" << std::endl;
		std::cout << "Available trackers are: " << std::endl;
		for (std::vector<std::string>::iterator it = tracker_types.begin() ; it != tracker_types.end(); ++it) {
			std::cout << " " << *it << std::endl;
		}
	}
	return tracker;
}

void ImageConverter::image_error(std::vector<cv::Rect2d> v)
{ // TARGET - END EFFECTOR
	uncalibrated_visual_servoing::PointVector2D pv;
	uncalibrated_visual_servoing::Point2D pt;
	// publish end effector position
	pt.x = (v[1].tl().x + v[1].br().x) / 2;
	pt.y = (v[1].tl().y + v[1].br().y) / 2;
	pv.points.push_back(pt);
	pt.x = (v[3].tl().x + v[3].br().x) / 2;
	pt.y = (v[3].tl().y + v[3].br().y) / 2;
	pv.points.push_back(pt);
	eef_pub.publish(pv);
	// publish current image error
	pv.points.clear();
	pt.x = ((v[0].tl().x + v[0].br().x) / 2) - ((v[1].tl().x + v[1].br().x) / 2);
	pt.y = ((v[0].tl().y + v[0].br().y) / 2) - ((v[1].tl().y + v[1].br().y) / 2);
	pv.points.push_back(pt);
	pt.x = ((v[2].tl().x + v[2].br().x) / 2) - ((v[3].tl().x + v[3].br().x) / 2);
	pt.y = ((v[2].tl().y + v[2].br().y) / 2) - ((v[3].tl().y + v[3].br().y) / 2);
	pv.points.push_back(pt);
	error_pub.publish(pv);
	return;
}

void ImageConverter::spin()
{
	int k;
	ros::Rate r(60); 
	std::vector<cv::Rect2d> v;
	std::cout << "***********************************************************************************************" << std::endl;
	std::cout << "* User Interface Node" << std::endl;
	std::cout << "***********************************************************************************************" << std::endl;
	std::cout << "* First click the top left and bottom right corners to set the goal position" << std::endl;
	std::cout << "* Then click the top left and bottom right corners to set the end effector position" << std::endl;
	std::cout << "* Do the left camera frame first, then the right camera frame" << std::endl;
	std::cout << "***********************************************************************************************" << std::endl;
	while (ros::ok()) {
		if (initialized_1 && initialized_2) {
			cv::Mat frame(cv::Size(cv_ptr1->image.cols*2, cv_ptr1->image.rows), cv_ptr1->image.type(), cv::Scalar::all(0));
			cv::Mat roi = frame(cv::Rect(0, 0, cv_ptr1->image.cols, cv_ptr1->image.rows));
			cv_ptr1->image.copyTo(roi);
			roi = frame(cv::Rect(cv_ptr1->image.cols, 0, cv_ptr1->image.cols, cv_ptr1->image.rows));
			cv_ptr2->image.copyTo(roi);
			if (initialize_tracker) {
				multiTracker->add(createTrackerByName("KCF"), frame, cv::Rect2d(clicked_points[0], clicked_points[1]));
				clicked_points.clear();
				initialize_tracker = false;
			}
			multiTracker->update(frame);
			v = multiTracker->getObjects();
			for (int i = 0; i < v.size(); i++) {
				cv::rectangle(frame, v[i], CV_RGB(0,255,255), 2, 1);
				// std::cout << v[i].tl().x << " " << v[i].br().x << std::endl;
			}
			if (v.size() == 4) {
				image_error(v);
			}
			cv::imshow(OPENCV_WINDOW, frame);
			k = cv::waitKey(1);
			if (k == 27) { // esc key
				cv::destroyAllWindows();
				break;
			} else if (k == 114) { // 'r' key
				std::cout << "creating new multitracker object" << std::endl;
				multiTracker = cv::MultiTracker::create();
			}
		}
		ros::spinOnce();
		r.sleep();
	}
	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "user_interface");
	ros::NodeHandle nh_("~");
	ImageConverter ic(nh_);
	ic.spin();
	return 0;
}