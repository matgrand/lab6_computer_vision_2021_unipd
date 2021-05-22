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

	Mat frame_descr;
	vector<KeyPoint> frame_keypoints;

	Ptr<SIFT> sift = SIFT::create();
	sift->SIFT::detect(frame, frame_keypoints);   //detect keypoints
	sift->SIFT::compute(frame, frame_keypoints, frame_descr);  //compute descriptor

	BFMatcher matcher = BFMatcher(cv::NORM_L2, false);   //create a matcher of type BFMatcher to match descriptors

	vector<vector<Point2f>> ret;

	//robust features selection using combination of lowe method and homography
	for (int i = 0; i < obj_imgs.size(); ++i) {
		
		vector<vector<DMatch>> matches;
		matcher.knnMatch(obj_descr[i], frame_descr, matches, 2);

		//keep only good matches, nearest neighbor
		vector<DMatch> good_matches;
		vector<DMatch> very_good_matches;
		vector<Point2f> good_obj_pts; //good obj points
		vector<Point2f> good_scene_pts; //frame points

		//lowe ratio test
		for (int j = 0; j < matches.size() - 1; j++) {
			if (matches[j][0].distance < lowe_ratio * matches[j][1].distance) {
				DMatch good_match = matches[j][0];
				good_matches.push_back(good_match); // add this match to the good matches
				Point2f p1 = obj_keypts[i][good_match.queryIdx].pt;
				Point2f p2 = frame_keypoints[good_match.trainIdx].pt;
				good_obj_pts.push_back(p1); //add the first image point from the current good match 
				good_scene_pts.push_back(p2); //add the second image point from the current good match
			}
		}

		cout << "Initial Matches = " << matches.size();
		cout << ",   After Lowe ratio test = " << good_matches.size() << endl;

		// select among good matches with RANSAC
		Mat inliers_mask;
		Mat H = findHomography(good_obj_pts, good_scene_pts, inliers_mask, RANSAC);

		//keep only good points
		vector<Point2f> very_good_obj_pts; //obj[i] points
		vector<Point2f> very_good_scene_pts; //frame points
		for (int j = 0; j < inliers_mask.rows; j++) {
			if (inliers_mask.at<char>(j, 0) > 0) { //check with inliers mask if the projected point is correct
				very_good_matches.push_back(good_matches[j]);
				very_good_obj_pts.push_back(good_obj_pts[j]);
				very_good_scene_pts.push_back(good_scene_pts[j]);
			}
		}

		cout << "After homography test = " << very_good_scene_pts.size() << endl;

		//add rectangle points 
		vector<Point2f> obj_corners(4);
		vector<Point2f> scene_corners(4);

		obj_corners[0] = Point2f(0, 0);
		obj_corners[1] = Point2f((float)obj_imgs[i].cols, 0);
		obj_corners[2] = Point2f((float)obj_imgs[i].cols, (float)obj_imgs[i].rows);
		obj_corners[3] = Point2f(0, (float)obj_imgs[i].rows);

		perspectiveTransform(obj_corners, scene_corners, H);

		//add the corners at the end of the vector
		for (int k = 0; k < 4; k++)
			very_good_scene_pts.push_back(scene_corners[k]);
		
		ret.push_back(very_good_scene_pts);

		if (SLOW_MODE) {
			Mat out1, out2, out3;
			namedWindow("Original Matches", WINDOW_NORMAL);
			cv::drawMatches(obj_imgs[i], obj_keypts[i], frame, frame_keypoints, matches, out1);
			cv::imshow("Original Matches", out1);
			namedWindow("After Lowe", WINDOW_NORMAL);
			cv::drawMatches(obj_imgs[i], obj_keypts[i], frame, frame_keypoints, good_matches, out2);
			cv::imshow("After Lowe", out2);
			namedWindow("After Homography", WINDOW_AUTOSIZE);
			cv::drawMatches(obj_imgs[i], obj_keypts[i], frame, frame_keypoints, very_good_matches, out3);
			cv::imshow("After Homography", out3);
			
			cout << "H = " << endl << " " << H << endl << endl;

			cv::waitKey(0);
		}
	}
  
	std::cout << "match() ret size = " << ret.size() << '\n';
	cout << "match() ret[0] size = " << ret[0].size() << endl;
	cout << "match() ret[last] size = " << ret[ret.size()-1].size() << endl;

	cout << "corners 0:" << endl;
	cout << ret[0][ret[0].size() - 1] << " " << ret[0][ret[0].size() - 2];
	cout << ret[0][ret[0].size() - 3] << " " << ret[0][ret[0].size() - 4] << endl;


	return ret;

}
