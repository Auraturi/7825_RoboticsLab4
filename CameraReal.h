#pragma once

#include <opencv2/opencv.hpp>
#include "CameraVirtual.h"

using namespace std;
using namespace cv;

class CCameraReal
{
public:
	CCameraReal();
	~CCameraReal();

private:
	// Webcam
	int _webcam_id;
	cv::VideoCapture _vid_webcam;

	// CVUI setting variables
	bool _draw_on_board;
	bool _draw_markers;

	// Webcam model
	Mat _cam_webcam_intrinsic;
	Mat _cam_webcam_extrinsic;
	Mat _cam_webcam_dist_coeff;

	Mat _rvec, _tvec; //Eth

	// for ui use
	double _board_x_val;
	double _board_y_val;
	double _board_z_val;
	int _board_x_ui;
	int _board_y_ui;
	int _board_z_ui;

public:
	const Mat& get_rvec() const { return _rvec; }
	const Mat& get_tvec() const { return _tvec; }

	void start_webcam(int webcam_id);
	void get_image(Mat& im);

	bool save_camparam(string filename, Mat& cam, Mat& dist);
	bool load_camparam(string filename, Mat& cam, Mat& dist);

	void createChArUcoBoard();
	void detect_markers(int cam_id); //based on calibrate_board to start

	void calibrate_board(int cam_id);


	void transform_to_image(Mat pt3d_mat, Point2f& pt2d);
	void transform_to_image(vector<Mat> pts3d_mat, vector<Point2f>& pts2d);

	void update_settings(Mat &im);


};

