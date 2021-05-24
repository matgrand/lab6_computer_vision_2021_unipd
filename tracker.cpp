#include "tracker.h"
#include <time.h>       

#define SLOW_MODE true //change this to false to only see the video with the rectangles

const int pyramid_levels = 2;
const float max_movement = 0.1;

// estimate variation between frames
float get_average_movement(vector<Point2f> pts1, vector<Point2f> pts2);

MyTracker::MyTracker(vector<vector<Point2f>> to_track, Mat initial_frame) {
	
	cout << endl << "TRACKER" << endl << endl;

	num_objs = to_track.size(); //number of objects to track
	last_out = initial_frame.clone();
	last_frame = initial_frame.clone();
	show_frame = initial_frame.clone();


	// Create a mask image for drawing purposes
	track_mask = Mat::zeros(last_frame.size(), last_frame.type());

	//separate corners from features
	for (int obj_i = 0; obj_i < num_objs; obj_i++) {
		vector<Point2f> i_th_vec = to_track[obj_i];
		//extract rect points
		vector<Point2f> rect_to_add(4);
		std::copy(i_th_vec.begin() + i_th_vec.size()-4, i_th_vec.begin() + i_th_vec.size(), rect_to_add.begin());
		//add rect points
		last_rects.push_back(rect_to_add);
		//extract features
		vector<Point2f> feat_to_add(i_th_vec.size() - 4);
		std::copy(i_th_vec.begin(), i_th_vec.begin() + i_th_vec.size()-4, feat_to_add.begin());
		//add features
		last_objs_pts.push_back(feat_to_add);

		Mat H = findHomography(last_objs_pts[obj_i], last_objs_pts[obj_i], RANSAC);
		
		has_changed.push_back(true);
		Hs.push_back(H);
	}

	//assign a random color to each obj
	RNG rng(time(NULL));
	for (int obj_i = 0; obj_i < num_objs; obj_i++) { //cycle the objects
		int r = rng.uniform(0, 256);
		int g = rng.uniform(0, 256);
		int b = rng.uniform(0, 256);
		colors.push_back(Scalar(r, g, b));
	}

	if (SLOW_MODE) {
		cout << "num_objs= " << num_objs << endl;
		//imshow("Initial Frame", last_frame);
		waitKey(0);
	}

}

void MyTracker::track(Mat new_frame) {
	
	Mat frame = new_frame.clone();
	//convert in gray scale
	Mat frame_gray, last_frame_gray;
	cvtColor(frame.clone(), frame_gray, COLOR_BGR2GRAY);
	cvtColor(last_frame, last_frame_gray, COLOR_BGR2GRAY);

	for (int obj_i = 0; obj_i < num_objs; obj_i++) { // cycle through the objects

		//old_pts are the points in the most recent element in objs_points_vec
		vector<Point2f> old_pts = last_objs_pts[obj_i];

		// // CALCULATE OPTICAL FLOW
		vector<uchar> status;
		vector<float> err;
		TermCriteria criteria = TermCriteria((TermCriteria::COUNT)+(TermCriteria::EPS), 10, 0.03);
		//create new_pts
		vector<Point2f> new_pts;
		calcOpticalFlowPyrLK(last_frame_gray, frame_gray, old_pts, new_pts, status, err, Size(15, 15), pyramid_levels, criteria);
	
		//select only the good points
		vector<Point2f> new_good_pts, old_good_points;
		for (int i = 0; i < old_pts.size(); i++) //cycle through the old points
		{
			if (status[i] == 1) {
				new_good_pts.push_back(new_pts[i]);
				old_good_points.push_back(old_pts[i]);
			}
		}

		//ESTIMATE TRANSLATION 
		Mat H = findHomography(old_good_points, new_good_pts, RANSAC);

		has_changed[obj_i] = (get_average_movement(old_good_points, new_good_pts) > max_movement);

		if (has_changed[obj_i]) {// position is changed

			// update homography
			Hs[obj_i] = H.clone();
			//update last_objs_pts
			last_objs_pts[obj_i] = new_good_pts;
			
			if (SLOW_MODE) // draw track lines
				for (int i = 0; i < new_pts.size(); ++i) 
					line(track_mask, new_pts[i], old_pts[i], colors[obj_i], 1);
		}
	}
	
	last_frame = frame.clone();
	if (SLOW_MODE) {
		imshow("Mask", track_mask);
		waitKey(1);
		for (int j = 0; j < 20; j++)
			subtract(frame, track_mask, frame);
		add(frame, track_mask, frame);
		show_frame = frame;
	}
}

void MyTracker::draw_rect() {
	Mat out = show_frame.clone();
	for (int obj_i = 0; obj_i < num_objs; obj_i++) {
		vector<Point2f> curr_rect, new_rect;
		curr_rect = last_rects[obj_i];
		//translate the rect
		Mat H = Hs[obj_i];
		perspectiveTransform(curr_rect, new_rect, H);
		Hs[obj_i] = Mat::eye(H.size(), H.type());

		//add the rect
		last_rects[obj_i] = new_rect;

		//draw
		line(out, new_rect[0], new_rect[1], colors[obj_i], 5);
		line(out, new_rect[1], new_rect[2], colors[obj_i], 5);
		line(out, new_rect[2], new_rect[3], colors[obj_i], 5);
		line(out, new_rect[3], new_rect[0], colors[obj_i], 5);

		last_out = out.clone();
	}

	namedWindow("Tracking", WINDOW_NORMAL);
	imshow("Tracking", last_out);
	waitKey(1);
}

float get_average_movement(vector<Point2f> pts1, vector<Point2f> pts2) {
	vector<float> vx;
	vector<float> vy;
	float sumx = 0.0;
	float sumy = 0.0;
	for (int i = 0; i < pts1.size(); ++i) {
		float fact = pts1[i].x - pts2[i].x;
		sumx += fact * fact;
		fact = pts1[i].y - pts2[i].y;
		sumy += fact * fact;
	}
	float ret = (sumx + sumy) / pts1.size();
	if (SLOW_MODE)
		cout << ret << endl;
	return ret;
}
