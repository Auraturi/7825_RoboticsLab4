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

	////////////////////////////////////
	// LAB 6

	// Part A-C: true = joint sliders drive the robot (fkin), false = X Y Z Th sliders drive it (ikine)
	bool _joint_control;

	// Part A-B: which of the two 2-link solutions ikine() returns. false = j1 >= 0, true = j1 <= 0
	bool _elbow_flip;

	// Part A-C: target end effector pose sliders (x mm, y mm, z mm, theta deg)
	double _pose[4];

	// Part A-C: the joints the robot is drawn with (j0 deg, j1 deg, j2 deg, j3 metres).
	// Kept as doubles because the int joint sliders would round the ikine() answer.
	Vec4d _joints_lab6;

	// Part A-C: false when the last ikine() call could not reach the target (robot holds its last pose)
	bool _ik_ok;

	// Part A-D: pose at the start of the current straight line move, and the path drawn so far
	Vec4d _anim_start;
	vector<Mat> _ik_trail;

	// Part B-B: true = look for ArUco marker ID 50 and lock the robot onto it (Part B-C)
	bool _track_marker;
	bool _marker_found;		// marker seen in this frame
	Mat _marker_bTm;		// marker pose in the board (drawing) frame, used to draw its box
	Vec6d _marker_pose;		// marker pose in the robot world frame (x mm, y mm, z mm, R deg, P deg, Y deg)

	void init_lab6();									// Part A-C: joints home + Lab 6 variables to defaults
	Mat createHT_rvec(Mat rvec, Mat tvec);				// Part B-B: OpenCV rvec / tvec -> 4x4 transform (Rodrigues)
	Point2f to_image(Mat pt3d);							// project one 3D point with whichever camera is in use
	void set_joint_sliders(Vec4d joints);				// Part A-C: show (deg, deg, deg, m) on the int joint sliders
	void update_scara_settings_lab6(Mat& im);			// Part A-C: Lab 6 version of update_scara_settings()
	void animate_scara_ikine();							// Part A-D: straight line end effector animation
	void track_marker(Mat& im, Mat base);				// Part B-B / B-C: marker pose -> target pose sliders

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

	void create_scara_lab6();	// Part A: create_scara() + Lab 6 defaults
	void draw_scara_lab6();		// Part A / Part B: draw the SCARA with joint or end effector control

	// Part A-B: Input end effector pose (x, y, z, theta), output joint variables (j0, j1, j2, j3).
	// Returns false (joints untouched) when the pose cannot be reached.
	bool ikine(Vec4d pose, Vec4d& joints, bool elbow_flip = false);
};

