// matcher class

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>


using namespace cv;
using namespace std;

class Matcher {
public:

	// constructor
	Matcher();

	// // methods
	//load_obj, takes an image, calculates features (defines a new object)
	// then adds the object to the list of objects to track
	void load_obj(Mat img_of_obj);

	// //variables
	vector<Mat> obj_imgs;


};

