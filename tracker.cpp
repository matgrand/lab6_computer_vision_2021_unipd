#include "tracker.h"

MyTracker::MyTracker(vector<vector<Point2f>> to_track, Mat initial_frame) {
	frames_objs_points.push_back(to_track);
	recent_frames.push_back(initial_frame);
}

void MyTracker::track(Mat frame) {

}


void MyTracker::draw_rect() {

}


