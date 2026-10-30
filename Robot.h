#pragma once

#include <opencv2/opencv.hpp>

#include "CameraVirtual.h"
#include "CameraReal.h"

using namespace std;
using namespace cv;
using namespace dnn;

class CRobot
{
public:
	CRobot();
	~CRobot();

private:
	Size _image_size;
	Mat _canvas;

	void init();

	////////////////////////////////////
  // LAB 3
	
	vector<vector<Mat>> _simple_robot;
	bool _use_virtual_camera;

	CCameraVirtual _virtualcam;
	CCameraReal _realcam;

	std::vector<Mat> createBox(float w, float h, float d);
	std::vector<Mat> createCoord();

	void transformPoints(std::vector<Mat>& points, Mat T);
	
	void drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour);
	void drawCoord(Mat& im, std::vector<Mat> coord3d);

	////////////////////////////////////
	// LAB 4

	////////////////////////////////////
	// LAB 5
	
	int _do_animate; // Animation state machine

	void update_settings(Mat& im);

	// Part A-A: SCARA model. Each box is stored in its own joint frame,
	// and is moved into place by the joint transforms every frame.
	vector<vector<Mat>> _scara_links;	// pedestal, link 1, link 2, link 3 (quill)
	vector<Scalar> _scara_colors;		// one colour per box above

	// Part A-C: slider values. _joint[0..2] in degrees, _joint[3] (prismatic) in mm
	int _joint[4];

	// Part B-A: how far joint 2 has turned in the 0 -> 360 animation step (degrees)
	int _anim_count;

	// Part B-B: true = draw on the ArUco board with the webcam, false = virtual camera
	bool _draw_on_board;

	std::vector<Mat> scara_transforms(Vec4d joints);	// Part A-A: world pose of every joint frame
	Vec4d get_joints();									// Part A-C: slider values -> (deg, deg, deg, m)
	void update_scara_settings(Mat& im);				// Part A-C: SCARA sliders, Reset, Animate, pose readout
	void animate_scara();								// Part B-A: animation state machine

public:
	////////////////////////////////////
	// Lab 3

	Mat createHT(Vec3d t, Vec3d r);
	void create_simple_robot();
	void draw_simple_robot();

	////////////////////////////////////
	// Lab 4

	void draw();
	void create_simple_robot_lab4();
	void draw_simple_robot_lab4();
	void rotate_simple_robot_lab4();

	////////////////////////////////////
	// Lab 5

	void create_scara();		// Part A-A: build the SCARA boxes
	void draw_scara();			// Part A-A / Part B-B: draw on the virtual camera or the ArUco board
	Vec4d fkin(Vec4d joints);	// Part A-B: Input joint variables, output end effector pose (x, y, z, theta)

	////////////////////////////////////
	// Lab 6

	// bool ikine(); // Input end effector pose, output joint angles
};

