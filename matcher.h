// matcher class

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>

using namespace cv;
using namespace std;

const double lowe_ratio = 0.8;

class MyMatcher {
public:

	// constructor
	MyMatcher();

	// // methods
	// load_obj: takes an image, calculates features (defines a new object)
	// then adds the object to the list of objects to match
	void load_obj(Mat img_of_obj);

	// match: return a vector of vectors of points : positions of the good matches, one vector for each obj
	// See https://docs.opencv.org/3.4/d7/dff/tutorial_feature_homography.html
	vector<vector<Point2f>> match(Mat frame);


	// //variables
	vector<Mat> obj_imgs;
	vector<Mat> obj_descr;
	vector<vector<KeyPoint>> obj_keypts;

};
