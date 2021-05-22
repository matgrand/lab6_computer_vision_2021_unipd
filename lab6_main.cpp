#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <stdlib.h>
#include "matcher.h"
#include "tracker.h"

using namespace cv;
using namespace std;

const String img_folder_path = "../Data/objects/";

int main(int argc, char* argv[]) {
	//load the images and create the objects associated with them
	vector<String> img_filenames;
	utils::fs::glob(img_folder_path, "*.bmp", img_filenames);
	utils::fs::glob(img_folder_path, "*.png", img_filenames);

	//initialize the matcher
	MyMatcher mtchr;

	for (int i = 0; i < img_filenames.size(); i++) { //for every obj image
		Mat tmp_img = imread(img_filenames[i]);
		//imshow(to_string(i), tmp_img); //show the image

		//loads the objects in the matcher
		//and finds its features
		mtchr.load_obj(tmp_img);
	}

	VideoCapture cap("../Data/video.mov");
	Mat frame;

	if (cap.isOpened()) {
		// get the first frame
		cap >> frame;
		//match the first frame
		vector<vector<Point2f>> good_points_vec = mtchr.match(frame);

		//create tracker
		MyTracker trckr = MyTracker(good_points_vec, frame);

		//draw first rectangle
		trckr.draw_rect();

		waitKey(0);

		while (1) {
			cap >> frame; //update frame
			if (frame.empty()) //exit condition
				break;
			else {
				//track the points in the new frame and estimates rotations and translations
				trckr.track(frame);

				//draw the updated rectangle
				trckr.draw_rect();
			}
		}
	}

	waitKey(0);
}
