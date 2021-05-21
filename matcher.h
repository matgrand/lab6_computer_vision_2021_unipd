// matcher class

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>


using namespace cv;
using namespace std;

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
	vector<Mat> match(Mat frame);


	// //variables
	vector<Mat> obj_imgs;

};
