#include "tracker.h"

#define MAX_KEPT 10
#define SLOW_MODE true

const int pyramid_levels = 2;

template <class T>
void remove_first(vector<T> v) {
	v.erase(v.begin());
}

MyTracker::MyTracker(vector<vector<Point2f>> to_track, Mat initial_frame) {
	
	//separate corners from features
	vector<vector<Point2f>> features_to_track = to_track; //change this
	vector<vector<Point2f>> rect_corners = to_track; //change this
	
	num_objs = to_track.size(); //number of objects to track
	
	// adds MAX_KEPT to the things, to create the "previous MAX_KEPT things"
	for (int frame_i = 0; frame_i < MAX_KEPT; frame_i++) {
		recent_frames.push_back(initial_frame); 
		objs_points.push_back(features_to_track);
		H_vec.push_back(findHomography(features_to_track, features_to_track, RANSAC));
		rect_points.push_back(rect_corners);
	}

	//assign a random color to each obj
	for (int obj_i = 0; obj_i < num_objs; obj_i++) { //cycle the objects
		RNG rng;
		int r = rng.uniform(0, 256);
		int g = rng.uniform(0, 256);
		int b = rng.uniform(0, 256);
		colors.push_back(Scalar(r, g, b));
	}
}

void MyTracker::track(Mat frame) {
	Mat last_frame;
	
	//last_frame is the last added
	last_frame = recent_frames.back();
	//add the new frame
	recent_frames.push_back(frame);
	remove_first(recent_frames); //remove the first element, keep the last MAX_KEPT elements

	//convert in gray scale
	Mat frame_gray, last_frame_gray;
	cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	cvtColor(last_frame, last_frame_gray, COLOR_BGR2GRAY);

	//vector of vectors of points, one vector for each obj
	vector<vector<Point2f>> points_to_add;
	vector<Mat> H_to_add;

	for (int obj_i = 0; obj_i < num_objs; obj_i++) { // cycle through the objects
		
		//p_old is the most recent element in objs_points
		vector<Point2f> p_old = objs_points.back()[obj_i];

		// Create a mask image for drawing purposes
		Mat mask = Mat::zeros(last_frame.size(), last_frame.type());

		// calculate optical flow
		vector<uchar> status;
		vector<float> err;
		TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
		//create p_new
		vector<Point2f> p_new;
		calcOpticalFlowPyrLK(last_frame_gray, frame_gray, p_old, p_new, status, err, Size(15, 15), pyramid_levels, criteria);
	
		//select good points only
		vector<Point2f> p_new_good;
		for (uint i = 0; i < p_old.size(); i++) //cycle through the old points
		{
			if (status[i] == 1) {
				p_new_good.push_back(p_new[i]);
				// draw the tracks
				line(mask, p_new[i], p_old[i], colors[i], 2);
				circle(frame, p_new[i], 5, colors[i], -1);
			}
		}

		//add the new good point to the vector of points
		points_to_add.push_back(p_new_good);

		if (SLOW_MODE) {
			Mat img_out; 
			add(frame, mask, img_out);
			imshow("Frame", img_out);
		}


		//ESTIMATE TRANSLATION 
		H_to_add.push_back(findHomography(p_old, p_new_good, RANSAC));
	
	}

	//add the current set of vectors to the list of sets of vectors
	objs_points.push_back(points_to_add);
	remove_first(objs_points);

	//add the current set of homographies
	H_vec.push_back(H_to_add);
	remove_first(H_vec);

}


void MyTracker::draw_rect() {
	vector<vector<Point2f>> rect_to_add;
	for (int obj_i = 0; obj_i < num_objs; obj_i++) {
		vector<Point2f> curr_rect, new_rect;
		curr_rect = rect_points.back()[obj_i];
		//translate the rect
		perspectiveTransform(curr_rect, new_rect, H_vec.back());
		//add the rect
		rect_to_add.push_back(new_rect);
	}
	//add the rects
	rect_points.push_back(rect_to_add);
	remove_first(rect_points);
}


