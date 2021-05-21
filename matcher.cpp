#include "matcher.h"

#define SLOW_MODE false 

using namespace cv;
using namespace std;

MyMatcher::MyMatcher(){

}

void MyMatcher::load_obj(Mat img_of_obj){

	Mat descr;
	vector<KeyPoint> keypoints;

	Ptr<SIFT> sift = SIFT::create();
	sift->SIFT::detect(img_of_obj, keypoints);   //detect keypoints
	sift->SIFT::compute(img_of_obj, keypoints, descr);  //compute descriptor
	obj_imgs.push_back(img_of_obj);
	obj_descr.push_back(descr);
	obj_keypts.push_back(keypoints);

}

vector<vector<Point2f>> MyMatcher::match(Mat frame){

	Mat descr;
	vector<KeyPoint> keypoints;

	Ptr<SIFT> sift = SIFT::create();
	sift->SIFT::detect(frame, keypoints);   //detect keypoints
	sift->SIFT::compute(frame, keypoints, descr);  //compute descriptor

	Ptr<BFMatcher> matcher = BFMatcher::create(cv::NORM_L2);   //create a matcher of type BFMatcher to match descriptors

	vector<vector<DMatch>> matches;  //one vector of metches for each example


	for(int i = 0; i < obj_imgs.size(); ++i){
		//for loop to match the frame with the example images

		vector<DMatch> v;
		matcher->BFMatcher::match(obj_descr[i], descr, v);
		matches.push_back(v);
	}

  //thresholding to pick only good matches, for every example compute the minimum match distance.
  //Then accept as good match if its distance is below min_distance times threshold

	double ratio_thresh = 2;
	vector<double> min_dists(matches.size());   //vector of minimum distances of matches
	fill(min_dists.begin(), min_dists.end(), numeric_limits<double>::infinity());
	vector<vector<DMatch>> good_matches(matches.size());

	for (int i = 0; i < matches.size(); ++i) {
		for (int j = 0; j < matches[i].size(); ++j) {
			//for loop to compute the minimum distances between matches of the frame and the examples

			if (matches[i][j].distance < min_dists[i]) {
				min_dists[i] = matches[i][j].distance;
			}
		}
	}

	for (int i = 0; i < matches.size(); ++i) {
		for (int j = 0; j < matches[i].size(); ++j) {
			//for loop to take the best matches given the threshold
			if (matches[i][j].distance < ratio_thresh*min_dists[i]) {
				good_matches[i].push_back(matches[i][j]);
			}
		}
		std::cout << "Number of good matches: " << good_matches[i].size() << '\n';
	}

	vector<vector<Point2f>> obj;
	vector<vector<Point2f>> scene;
	vector<Point2f> obj_corners(4);
	vector<Point2f> scene_corners(4);

	for(int i = 0; i < obj_imgs.size(); ++i){

		obj_corners[0] = Point2f(0, 0);
		obj_corners[1] = Point2f( (float)obj_imgs[i].cols, 0 );
		obj_corners[2] = Point2f( (float)obj_imgs[i].cols, (float)obj_imgs[i].rows );
		obj_corners[3] = Point2f( 0, (float)obj_imgs[i].rows );

		cout << "corners " << i << " = " << obj_corners[0] << " " << obj_corners[1];
		cout << " " << obj_corners[2] << " " << obj_corners[3] << endl;
		vector<Point2f> obj_to_add;
		vector<Point2f> scene_to_add;
		for(int j = 0; j < good_matches[i].size(); ++j){
			//-- Get the keypoints from the good matches
			obj_to_add.push_back(obj_keypts[i][good_matches[i][j].queryIdx].pt);
			scene_to_add.push_back(keypoints[good_matches[i][j].trainIdx].pt);
		}
		obj.push_back(obj_to_add);
		//add the projected points
		scene.push_back(scene_to_add);

		//find the projected corners
		Mat H = findHomography(obj[i], scene[i], RANSAC);
		perspectiveTransform( obj_corners, scene_corners, H);
    
		//add the corners at the end of the vector
		for (int k = 0; k < 4; k++) {
			Point2f corner = scene_corners[k];
			scene[i].push_back(corner);
		}
	
		if (SLOW_MODE) {
			cv::Mat out;
			cv::drawMatches(obj_imgs[i], obj_keypts[i], frame, keypoints, good_matches[i], out);
			cv::imshow("Matches", out);
			waitKey(0);
		}
	}

  
	std::cout << "match() ret size = " <<scene.size() << '\n';
	cout << "match() ret[0] size = " << scene[0].size() << endl;
	cout << "match() ret[last] size = " << scene[scene.size()-1].size() << endl;

	cout << "corners 0:" << endl;
	cout << scene[0][scene[0].size() - 1] << " " << scene[0][scene[0].size() - 2];
	cout << scene[0][scene[0].size() - 3] << " " << scene[0][scene[0].size() - 4] << endl;

	//cv::Mat out;
	//cv::drawMatches(obj_imgs[0], obj_keypts[0], frame, keypoints, good_matches[0], out);
	//cv::imshow("Matches", out);
	//waitKey(0);

	return scene;

}
