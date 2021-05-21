#include "matcher.h"


using namespace cv;
using namespace std;

MyMatcher(){

}

void MyMatcher::load_obj(Mat img_of_obj){

  Mat descr;
	vector<KeyPoint> keypoints;

	Ptr<SIFT> sift = SIFT::create();
	sift->SIFT::detect(img_of_obj, keypoints);   //detect keypoints
	sift->SIFT::compute(img_of_obj, keypoints, descr);  //compute descriptor
  obj_imgs.push_back(descr);

}

vector<Mat> MyMatcher::match(Mat frame){

  Mat descr;
  vector<KeyPoint> keypoints;

  Ptr<SIFT> sift = SIFT::create();
  sift->SIFT::detect(frame, keypoints);   //detect keypoints
  sift->SIFT::compute(frame, keypoints, descr);  //compute descriptor

  Ptr<BFMatcher> matcher = BFMatcher::create(cv::NORM_L2);   //create a matcher of type BFMatcher to match descriptors

  vector<vector<DMatch>> matches;

  for(int i = 0; i < obj_imgs_img.size(); ++i){
    //for loop to match the frame with the example images

    vector<DMatch> v;
    matcher->BFMatcher::match(frame, obj_imgs[i], v);
    matches.push_back(v);
  }

  //thresholding to pick only good matches, for every example compute the minimum match distance.
  //Then accept as good match if its distance is below min_distance times threshold

	double ratio_thresh = 2.5;
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
    std::cout << "Number of good matches: " << good_matches[i] << '\n';
	}

  vector<Mat> homographies;
  vector<Point2f> obj;
  vector<Point2f> scene;
  for(int j = 0; j < obj_imgs.size(); ++j){

    for(int i = 0; i < good_matches.size(); ++i){
    //-- Get the keypoints from the good matches
      obj.push_back( obj_imgs[ good_matches[i].queryIdx ].pt );
      scene.push_back( keypoints[ good_matches[i].trainIdx ].pt );
    }
    Mat H = findHomography(obj, scene, RANSAC);
    homographies.push_back(H);
  }
  return homographies
}
