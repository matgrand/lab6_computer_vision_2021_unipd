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


private:
	// //variables
	
	//keep the points to_track, for each frame analyzed 
	//frames < objects < points < point > > >
	vector<vector<Point2f>> last_objs_pts;  // features points
	vector<vector<Point2f>> last_rects;  // rectangles points

	Mat last_frame; //last frame
	vector<Scalar> colors; //colors for the rect 

	int num_objs;

	vector<Mat> Hs; // one Mat for each obj

	vector<bool> has_changed;

	Mat last_out;
	Mat track_mask;
	Mat show_frame;
};