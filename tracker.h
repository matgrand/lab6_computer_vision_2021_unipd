#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class MyTracker {

public:

	// constructor 
	//accepts a vector of vectors of points, one vector of points for each obj
	//see https://docs.opencv.org/3.4/d4/dee/tutorial_optical_flow.html
	MyTracker(vector<vector<Point2f>> to_track, Mat initial_frame);

	// // methods
	
	//tracks the points and estimate transaltions and rotatins for each obj
	void track(Mat frame);

	//draw_rect : 
	void draw_rect();



	// //variables
	
	//keep the points to_track, for each frame analyzed 
	//frames < objects < points < point > > >
	vector<vector<vector<Point2f>>> objs_points;  // features points
	vector<vector<vector<Point2f>>> rect_points;  // rectangles points

	vector<Mat> recent_frames; //collection of the recent frames
	vector<Scalar> colors; //colors for the rect 

	int num_objs;

	vector<vector<Mat>> H_vec; //4 obj for each frame, one Mat for each obj

};