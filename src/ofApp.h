#pragma once

#include "ofMain.h"
#include "opencv2/core/affine.hpp"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
	public:
		void drawFrustum(float _f, float _cx, float _cy, float clip_near, float clip_far);
		string tracks_filename;
		double f, cx, cy;

	protected:
		bool camera_pov;
		bool is_projective;
		std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
		std::vector<cv::Vec3f> point_cloud_est;
		std::vector<cv::Affine3d> path_est;
		int idx, forw, n;

		ofEasyCam cam;
		ofVec3f last_cam_pos, last_cam_dir, last_cam_up;
		float last_fov;
};
