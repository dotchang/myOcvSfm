#include "ofApp.h"
#include "opencv_lib.hpp"

#define CERES_FOUND 1

#include <opencv2/core.hpp>
#include <opencv2/sfm.hpp>
//#include <opencv2/viz.hpp>
#include <opencv2/core/affine.hpp>

#include <iostream>
#include <fstream>
#include <string>

#ifdef _DEBUG
#pragma comment(lib, "ceres_d.lib")
#pragma comment(lib, "libglog_d.lib")
#pragma comment(lib, "gflags_d.lib")
#pragma comment(lib, "correspondence_d.lib")
#pragma comment(lib, "multiview_d.lib")
#pragma comment(lib, "numeric_d.lib")
#pragma comment(lib, "simple_pipeline_d.lib")
#else
#pragma comment(lib, "ceres.lib")
#pragma comment(lib, "libglog.lib")
#pragma comment(lib, "gflags.lib")
#pragma comment(lib, "correspondence.lib")
#pragma comment(lib, "multiview.lib")
#pragma comment(lib, "numeric.lib")
#pragma comment(lib, "simple_pipeline.lib")
#endif

static void help() {
	cout
		<< "\n------------------------------------------------------------------\n"
		<< " This program shows the camera trajectory reconstruction capabilities\n"
		<< " in the OpenCV Structure From Motion (SFM) module.\n"
		<< " \n"
		<< " Usage:\n"
		<< "        example_sfm_trajectory_reconstruction <path_to_tracks_file> <f> <cx> <cy>\n"
		<< " where: is the tracks file absolute path into your system. \n"
		<< " \n"
		<< "        The file must have the following format: \n"
		<< "        row1 : x1 y1 x2 y2 ... x36 y36 for track 1\n"
		<< "        row2 : x1 y1 x2 y2 ... x36 y36 for track 2\n"
		<< "        etc\n"
		<< " \n"
		<< "        i.e. a row gives the 2D measured position of a point as it is tracked\n"
		<< "        through frames 1 to 36.  If there is no match found in a view then x\n"
		<< "        and y are -1.\n"
		<< " \n"
		<< "        Each row corresponds to a different point.\n"
		<< " \n"
		<< "        f  is the focal lenght in pixels. \n"
		<< "        cx is the image principal point x coordinates in pixels. \n"
		<< "        cy is the image principal point y coordinates in pixels. \n"
		<< "------------------------------------------------------------------\n\n"
		<< endl;
}

/* Build the following structure data
*
*            frame1           frame2           frameN
*  track1 | (x11,y11) | -> | (x12,y12) | -> | (x1N,y1N) |
*  track2 | (x21,y11) | -> | (x22,y22) | -> | (x2N,y2N) |
*  trackN | (xN1,yN1) | -> | (xN2,yN2) | -> | (xNN,yNN) |
*
*
*  In case a marker (x,y) does not appear in a frame its
*  values will be (-1,-1).
*/


void
parser_2D_tracks(const string &_filename, std::vector<cv::Mat> &points2d)
{
	ifstream myfile(_filename.c_str());

	if (!myfile.is_open())
	{
		cout << "Unable to read file: " << _filename << endl;
		exit(0);

	}
	else {

		double x, y;
		string line_str;
		int n_frames = 0, n_tracks = 0;

		// extract data from text file

		vector<vector<cv::Vec2d> > tracks;
		for (; getline(myfile, line_str); ++n_tracks)
		{
			istringstream line(line_str);

			vector<cv::Vec2d> track;
			for (n_frames = 0; line >> x >> y; ++n_frames)
			{
				if (x > 0 && y > 0)
					track.push_back(cv::Vec2d(x, y));
				else
					track.push_back(cv::Vec2d(-1));
			}
			tracks.push_back(track);
		}

		// embed data in reconstruction api format

		for (int i = 0; i < n_frames; ++i)
		{
			cv::Mat_<double> frame(2, n_tracks);

			for (int j = 0; j < n_tracks; ++j)
			{
				frame(0, j) = tracks[j][i][0];
				frame(1, j) = tracks[j][i][1];
			}
			points2d.push_back(cv::Mat(frame));
		}

		myfile.close();
	}

}


//--------------------------------------------------------------
void ofApp::setup(){
	camera_pov = false;

	// Read input parameters
	
	// Read 2D points from text file
	tracks_filename = "data/desktop_tracks.txt";
	f = 1914.0;
	cx = 640;
	cy = 360;
	std::vector<cv::Mat> points2d;
	parser_2D_tracks(tracks_filename, points2d);

	// Set the camera calibration matrix
	cv::Matx33d K = cv::Matx33d(f, 0, cx,
								0, f, cy,
								0, 0, 1);

	/// Reconstruct the scene using the 2d correspondences

	is_projective = true;
	cv::sfm::reconstruct(points2d, Rs_est, ts_est, K, points3d_estimated, is_projective);

	// Print output

	cout << "\n----------------------------\n" << endl;
	cout << "Reconstruction: " << endl;
	cout << "============================" << endl;
	cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
	cout << "Estimated cameras: " << Rs_est.size() << endl;
	cout << "Refined intrinsics: " << endl << K << endl << endl;

	cout << "3D Visualization: " << endl;
	cout << "============================" << endl;

	/// Create 3D windows
	ofSetWindowTitle("Estimation Coordinate Frame");
	ofSetBackgroundColor(ofColor::black);

	// Create the pointcloud
	cout << "Recovering points  ... ";

	// recover estimated points3d
	for (int i = 0; i < points3d_estimated.size(); ++i)
		point_cloud_est.push_back(cv::Vec3f(points3d_estimated[i]));

	cout << "[DONE]" << endl;

	/// Recovering cameras
	cout << "Recovering cameras ... ";

	for (size_t i = 0; i < Rs_est.size(); ++i)
		path_est.push_back(cv::Affine3d(Rs_est[i], ts_est[i]));

	cout << "[DONE]" << endl;

	/// Add cameras
	cout << "Rendering Trajectory  ... ";

	/// Wait for key 'q' to close the window
	cout << endl << "Press:                       " << endl;
	cout << " 's' to switch the camera pov" << endl;
	cout << " 'q' to close the windows    " << endl;

	if (path_est.size() > 0)
	{
		// animated trajectory
		idx = 0;
		forw = -1;
		n = static_cast<int>(path_est.size());
	}

	cam.setNearClip(0.001f);
	cam.setDistance(5.0f);
}

//--------------------------------------------------------------
void ofApp::update(){

}

void ofApp::drawFrustum(float _f, float _cx, float _cy, float clip_near, float clip_far)
{
	float aw = 2 * atan2(2*_cx, 2 * _f);
	float ah = 2 * atan2(2*_cy, 2 * _f);
	
	float px0 = clip_near * tan(aw);
	float px1 = clip_far * tan(aw);
	float py0 = clip_near * tan(ah);
	float py1 = clip_far * tan(ah);

	ofDrawLine(px0, py0, clip_near, px1, py1, clip_far);
	ofDrawLine(-px0, py0, clip_near,  -px1, py1, clip_far);
	ofDrawLine(-px0, -py0, clip_near, -px1, -py1, clip_far);
	ofDrawLine(px0, -py0, clip_near, px1, -py1, clip_far);

	ofDrawLine(px0, py0, clip_near, -px0, py0, clip_near);
	ofDrawLine(-px0, py0, clip_near, -px0, -py0, clip_near);
	ofDrawLine(-px0, -py0, clip_near, px0, -py0, clip_near);
	ofDrawLine(px0, -py0, clip_near, px0, py0, clip_near);

	ofDrawLine(px1, py1, clip_far, -px1, py1, clip_far);
	ofDrawLine(-px1, py1, clip_far, -px1, -py1, clip_far);
	ofDrawLine(-px1, -py1, clip_far, px1, -py1, clip_far);
	ofDrawLine(px1, -py1, clip_far, px1, py1, clip_far);
}

//--------------------------------------------------------------
void ofApp::draw(){
	if (path_est.size() > 0)
	{
		cam.begin();
		ofPushStyle();
		/// Render points as 3D cubes
		for (size_t i = 0; i < point_cloud_est.size(); ++i)
		{
			cv::Vec3d point = point_cloud_est[i];
			cv::Affine3d point_pose(cv::Mat::eye(3, 3, CV_64F), point);
			
			char buffer[50];
			sprintf(buffer, "%d", static_cast<int>(i));

			ofBoxPrimitive box;
			ofSetLineWidth(2.0f);
			ofSetColor(ofColor::blue);
			box.set(0.1, 0.1, -0.1);
			box.setPosition(point[0], point[1], point[2]);
			box.drawWireframe();	
		}
		ofPopStyle();
		cam.end();

		cv::Affine3d cam_pose = path_est[idx];
		
		cv::Matx44d mat44 = cam_pose.matrix;
			ofMatrix4x4 m44(mat44(0, 0), mat44(1, 0), mat44(2, 0), mat44(3, 0),
				mat44(0, 1), mat44(1, 1), mat44(2, 1), mat44(3, 1),
				mat44(0, 2), mat44(1, 2), mat44(2, 2), mat44(3, 2),
				mat44(0, 3), mat44(1, 3), mat44(2, 3), mat44(3, 3));

		if (camera_pov) {	
			cam.setPosition(m44.getTranslation());
			cam.lookAt(ofVec3f(0,0,1)*m44, ofVec3f(mat44(1,0),mat44(1,1), mat44(1,2)));
		}
		else
		{
			std::vector<ofPoint> path;
			for (int i = 0; i < path_est.size()-1; i++) {
				cv::Vec3d point = path_est[i].translation();
				path.push_back(ofPoint(point[0], point[1], point[2]));
			}
			ofPolyline trajectory(path);
			
			// render complete trajectory
			cam.begin();
			ofSetColor(ofColor::green);
			trajectory.draw();
			ofSetColor(ofColor::yellow);
			ofPushMatrix();
			ofMultMatrix(m44);
			ofDrawAxis(0.25);
			drawFrustum(f, cx, cy, 0.025, 0.4);
			ofPopMatrix();
			cam.end();
		}

		// update trajectory index (spring effect)
		forw *= (idx == n-1 || idx == 0) ? -1 : 1; idx += forw;
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 's') {
		if (!camera_pov) {
			last_cam_pos = cam.getPosition();
			last_cam_dir = cam.getLookAtDir();
			last_cam_up = cam.getUpDir();
			last_fov = cam.getFov();
			cam.setFov(2 * atan2(2 * cx, 2 * f) * 180.0f / PI);
		}
		else {
			cam.setPosition(last_cam_pos);
			cam.lookAt(last_cam_dir, last_cam_up);
			cam.setFov(last_fov);
		}
		camera_pov = !camera_pov;
	}
	else if (key == 'h') {
		help();
	}
	else if (key == 'q') {
		ofExit();
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
