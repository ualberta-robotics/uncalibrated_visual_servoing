#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
 
using namespace cv;
using namespace std;
 
vector<string> trackerTypes = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"}; 
 
// create tracker by name
Ptr<Tracker> createTrackerByName(string trackerType) 
{
	Ptr<Tracker> tracker;
	if (trackerType ==  trackerTypes[0])
		tracker = TrackerBoosting::create();
	else if (trackerType == trackerTypes[1])
		tracker = TrackerMIL::create();
	else if (trackerType == trackerTypes[2])
		tracker = TrackerKCF::create();
	else if (trackerType == trackerTypes[3])
		tracker = TrackerTLD::create();
	else if (trackerType == trackerTypes[4])
		tracker = TrackerMedianFlow::create();
	else if (trackerType == trackerTypes[5])
		tracker = TrackerGOTURN::create();
	else if (trackerType == trackerTypes[6])
		tracker = TrackerMOSSE::create();
	else if (trackerType == trackerTypes[7])
		tracker = TrackerCSRT::create();
	else {
		cout << "Incorrect tracker name" << endl;
		cout << "Available trackers are: " << endl;
		for (vector<string>::iterator it = trackerTypes.begin() ; it != trackerTypes.end(); ++it)
			std::cout << " " << *it << endl;
	}
	return tracker;
}
// Fill the vector with random colors
void getRandomColors(vector<Scalar>& colors, int numColors)
{
	RNG rng(0);
	for(int i=0; i < numColors; i++)
		colors.push_back(Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255))); 
}


// set default values for tracking algorithm and video
string videoPath = "videos/run.mp4";

// Initialize MultiTracker with tracking algo
vector<Rect> bboxes;

// create a video capture object to read videos
cv::VideoCapture cap(videoPath);
Mat frame;

// quit if unabke to read video file
if(!cap.isOpened()) 
{
	cout << "Error opening video file " << videoPath << endl;
	return -1;
}

// read first frame
cap >> frame;
	
// Get bounding boxes for first frame
// selectROI's default behaviour is to draw box starting from the center
// when fromCenter is set to false, you can draw box starting from top left corner
bool showCrosshair = true;
bool fromCenter = false;
cout << "\n==========================================================\n";
cout << "OpenCV says press c to cancel objects selection process" << endl;
cout << "It doesn't work. Press Escape to exit selection process" << endl;
cout << "\n==========================================================\n";
cv::selectROIs("MultiTracker", frame, bboxes, showCrosshair, fromCenter);
 
// quit if there are no objects to track
if(bboxes.size() < 1)
	return 0;
 
vector<Scalar> colors;  
getRandomColors(colors, bboxes.size()); 


// Specify the tracker type
string trackerType = "CSRT";
// Create multitracker
Ptr<MultiTracker> multiTracker = cv::MultiTracker::create();
 
// Initialize multitracker
for(int i=0; i < bboxes.size(); i++)
	multiTracker->add(createTrackerByName(trackerType), frame, Rect2d(bboxes[i]));  

while(cap.isOpened()) 
{
	// get frame from the video
	cap >> frame;
 
	// Stop the program if reached end of video
	if (frame.empty()) break;
 
	//Update the tracking result with new frame
	multiTracker->update(frame);
 
	// Draw tracked objects
	for(unsigned i=0; i<multiTracker->getObjects().size(); i++)
	{
		rectangle(frame, multiTracker->getObjects()[i], colors[i], 2, 1);
	}
 
	// Show frame
	imshow("MultiTracker", frame);
	 
	// quit on x button
	if  (waitKey(1) == 27) break;
	 
 }