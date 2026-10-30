#include "stdafx.h"

#include "Robot.h"
#include <cmath>

#include "cvui.h"

CRobot::CRobot()
{
	//////////////////////////////////////
	// Create image and window for drawing
	_image_size = Size(1280, 720);


	_canvas = cv::Mat::zeros(_image_size, CV_8UC3);
	cv::namedWindow(CANVAS_NAME);
	cvui::init(CANVAS_NAME);

	_virtualcam.init(_image_size); //matches canvas size to the virtual camera size
	_use_virtual_camera = true;
	_draw_on_board = false; // Lab 5 Part B-B: start on the virtual camera

	init();
}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	// reset variables
	_do_animate = 0;

	// Lab 5: SCARA joints back to home (all 0) and clear the animation counter
	for (int i = 0; i < 4; i++)
		_joint[i] = 0;
	_anim_count = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB3
////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Create Homogeneous Transformation Matrix
Mat CRobot::createHT(Vec3d t, Vec3d r) //t is the translation vector; r is the rotation vector, each are 3 element vectors
	//rotation r passes (gamma y, beta B, alpha a) in doubles
	// translation t passes (x, y, z) in doubles
	//wRb_xyz (Y,B,A) = Rz(A)*Ry(B)*Rx(Y) 
{

	float Y = static_cast<float>(r[0] * CV_PI / 180.0); // gamma
	float B = static_cast<float>(r[1] * CV_PI / 180.0); // beta
	float A = static_cast<float>(r[2] * CV_PI / 180.0); // alpha

	float _x = static_cast<float>(t[0]); // x_Boriginal
	float _y = static_cast<float>(t[1]); // y_Boriginal
	float _z = static_cast<float>(t[2]); // z_Boriginal

	float cA = cos(A);
	float cB = cos(B);
	float cY = cos(Y);
	float sA = sin(A);
	float sB = sin(B);
	float sY = sin(Y);

	return (Mat1f(4, 4) << 
		cA*cB,	cA*sB*sY - sA*cY,	cA * sB * cY + sA * sY,		_x, 
		sA*cB,	sA*sB*sY + cA*cY,	sA * sB * cY - cA * sY,		_y,  
		-sB,	cB*sY,				cB*cY,						_z, 
		0,		0,					0,							1);	


}

std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the center of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1))); //push_back adds an element to the end of the vector
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	return box;
}

std::vector<Mat> CRobot::createCoord()
{
	std::vector <Mat> coord;

	float axis_length = 0.05f;

	coord.push_back((Mat1f(4, 1) << 0, 0, 0, 1)); // O
	coord.push_back((Mat1f(4, 1) << axis_length, 0, 0, 1)); // X
	coord.push_back((Mat1f(4, 1) << 0, axis_length, 0, 1)); // Y
	coord.push_back((Mat1f(4, 1) << 0, 0, axis_length, 1)); // Z

	return coord;
}


void CRobot::transformPoints(std::vector<Mat>& points, Mat T)
{
	for (int i = 0; i < points.size(); i++)
	{
		points.at(i) = T * points.at(i);
	}
}

void CRobot::drawBox(Mat& im, std::vector<Mat> box3d, Scalar colour)
{
	std::vector<Point2f> box2d;

	// The 12 lines connecting all vertexes 
	std::size_t draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	std::size_t draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	if (_use_virtual_camera)
		_virtualcam.transform_to_image(box3d, box2d);
	else
		_realcam.transform_to_image(box3d, box2d);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = box2d.at(draw_box1[i]);
		Point pt2 = box2d.at(draw_box2[i]);

		line(im, pt1, pt2, colour, 1);
	}
}

void CRobot::drawCoord(Mat& im, std::vector<Mat> coord3d) //gets called elsewhere, draws the coordinate axes
{
	Point2f O, X, Y, Z;

	if (_use_virtual_camera)
		{
		_virtualcam.transform_to_image(coord3d.at(0), O);
		_virtualcam.transform_to_image(coord3d.at(1), X);
		_virtualcam.transform_to_image(coord3d.at(2), Y);
		_virtualcam.transform_to_image(coord3d.at(3), Z);
		}
	else
		{
		_realcam.transform_to_image(coord3d.at(0), O);
		_realcam.transform_to_image(coord3d.at(1), X);
		_realcam.transform_to_image(coord3d.at(2), Y);
		_realcam.transform_to_image(coord3d.at(3), Z);
		}

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

void CRobot::create_simple_robot()
{
	//below is added by Ted
	_simple_robot.clear();
	_use_virtual_camera = true;

	//Making all the transforms I need
	Mat base_transform = createHT(
		Vec3d(0.0, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));

	Mat wTb = createHT( //box B on top of W
		Vec3d(0.0, 0.0, 0.05f),
		Vec3d(0.0, 0.0, 0.0));

	Mat bTc = createHT( //box C on top of B
		Vec3d(0.0, 0.0, 0.05f),
		Vec3d(0.0, 0.0, 0.0));

	Mat cTd = createHT( //box D on top of C
		Vec3d(0.0, 0.0, 0.05f),
		Vec3d(0.0, 0.0, 0.0));

	Mat cTe = createHT( //box E positive x axis of C
		Vec3d(0.05f, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));

	Mat cTf = createHT( //box F negative x axis of C
		Vec3d(-0.05f, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));


	Mat wTc = wTb * bTc;
	Mat wTd = wTc * cTd;
	Mat wTe = wTc * cTe;
	Mat wTf = wTc * cTf;

	std::vector<Mat> bodyTransforms = { base_transform, wTb, wTc, wTd, wTe, wTf };

	for (int i = 0; i < 6; i++) {
		std::vector<Mat> box = createBox(0.05f, 0.05f, 0.05f);
		transformPoints(box, bodyTransforms.at(i));
		_simple_robot.push_back(box);
	}

}

void CRobot::create_simple_robot_lab4()
	{
	//below is added by Ted
	_simple_robot.clear();
	_use_virtual_camera = false;

	Mat board_align = createHT(
		Vec3d(0.0, 0.0, 0.0),
		Vec3d(180.0, 0.0, 90.0)
	);

	//Making all the transforms I need
	Mat base_transform = createHT(
		Vec3d(0.0, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));

	Mat wTb = createHT( //box B on top of W
		Vec3d(0.0, 0.0, 0.05f),
		Vec3d(0.0, 0.0, 0.0));

	Mat bTc = createHT( //box C on top of B
		Vec3d(0.0, 0.0, 0.05f),
		Vec3d(0.0, 0.0, 0.0));

	Mat cTd = createHT( //box D on top of C
		Vec3d(0.0, 0.0, 0.05f),
		Vec3d(0.0, 0.0, 0.0));

	Mat cTe = createHT( //box E positive x axis of C
		Vec3d(0.05f, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));

	Mat cTf = createHT( //box F negative x axis of C
		Vec3d(-0.05f, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));


	Mat wTc = wTb * bTc;
	Mat wTd = wTc * cTd;
	Mat wTe = wTc * cTe;
	Mat wTf = wTc * cTf;

	std::vector<Mat> bodyTransforms = { base_transform, wTb, wTc, wTd, wTe, wTf };

	for (int i = 0; i < 6; i++) {
		std::vector<Mat> box = createBox(0.05f, 0.05f, 0.05f);
		transformPoints(box, bodyTransforms.at(i));

		// Align the robot model into the board coordinate system
		for (int j = 0; j < box.size(); j++) {
			box.at(j) = board_align * box.at(j);
			}

		_simple_robot.push_back(box);
		}

	}




void CRobot::draw_simple_robot()
{
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	_virtualcam.update_settings(_canvas);

	std::vector<Scalar> colors = {
		CV_RGB(100, 0, 0),
		CV_RGB(170, 0, 0),
		CV_RGB(0, 0, 0),
		CV_RGB(255, 0, 0),
		CV_RGB(0, 255, 0),
		CV_RGB(0, 0, 255),
		};

	for (size_t i = 0; i < _simple_robot.size(); i++)
		{
		drawBox(_canvas, _simple_robot.at(i), colors.at(i));
		}

	// 6) Draw coordinate axes
	std::vector<Mat> O = createCoord();
	drawCoord(_canvas, O);

	// 7) Draw the UI to the right side
	_realcam.update_settings(_canvas);
	update_settings(_canvas);
	

	cv::imshow(CANVAS_NAME, _canvas);


	/* //Old draw_simple_robot()

	Mat im;

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	std::vector<Mat> O = createCoord();
	

	_realcam.get_image(im); //this was uncommented out, gets an image, that can then be drawn upon later
	//get rvec and tvec;

	//_realcam.detect_markers(0);
	
	_realcam.detect_markers(0);
	


	im.copyTo(_canvas);


	//get current settings for x y and z in mm, and 


	_virtualcam.update_settings(_canvas);

	//need to set those values to what i get... probably inside the function
	_realcam.update_settings(_canvas);

	//for drawing robot
	std::vector<Scalar> colors = {

		CV_RGB(100, 0, 0), //darker red
		CV_RGB(170, 0, 0), //red
		CV_RGB(0, 0, 0), //invisible?
		CV_RGB(255, 0, 0), //red
		CV_RGB(0, 255, 0), //green
		CV_RGB(0, 0, 255), //blue
	};

	//for (const std::vector<Mat>& object : _simple_robot)
	//{
	//	drawBox(_canvas, object, CV_RGB(255, 255, 255));
	//}
	

	for (size_t i = 0; i < _simple_robot.size(); i++)
		{
	
		drawBox(_canvas, _simple_robot.at(i), colors.at(i));
	}


	drawCoord(_canvas, O); // added to draw coordinates

	_realcam.update_settings(_canvas); //uncommented out for detction.
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
	*/
}

void CRobot::draw_simple_robot_lab4()
	{
	Mat im_raw;

	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	// 1) Get the webcam image
	_realcam.get_image(im_raw);

	// 2) Detect the board pose before drawing
	_realcam.detect_markers(0);

	// 3) If no valid pose, still draw the image and return
	if (_realcam.get_rvec().empty() || _realcam.get_tvec().empty())
		{
		if (!im_raw.empty())
			{
			// Put the webcam image at the original location
			im_raw.copyTo(_canvas(Rect(0, 0, im_raw.cols, im_raw.rows)));
			}

		_realcam.update_settings(_canvas);
		update_settings(_canvas);

		cv::imshow(CANVAS_NAME, _canvas);
		return;
		}

	// 4) Put the webcam image at its original location
	if (!im_raw.empty())
		{
		im_raw.copyTo(_canvas(Rect(0, 0, im_raw.cols, im_raw.rows)));
		}

	// 5) Draw robot on the image
	std::vector<Scalar> colors = {
		CV_RGB(100, 0, 0),
		CV_RGB(170, 0, 0),
		CV_RGB(0, 0, 0),
		CV_RGB(255, 0, 0),
		CV_RGB(0, 255, 0),
		CV_RGB(0, 0, 255),
		};

	for (size_t i = 0; i < _simple_robot.size(); i++)
		{
		drawBox(_canvas, _simple_robot.at(i), colors.at(i));
		}

	// 6) Draw coordinate axes
	std::vector<Mat> O = createCoord();
	drawCoord(_canvas, O);

	// 7) Draw the UI to the right side
	_realcam.update_settings(_canvas);
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);


	}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB4
////////////////////////////////////////////////////////////////////////////////////////////////////////
void CRobot::rotate_simple_robot_lab4()
	{
	//create rotating HT
	Mat rotation_HT = createHT(
		Vec3d(0.0, 0.0, 0.0),
		Vec3d(0.0, 0.0, 5.0) //rotate about the z axis of the world frame.
	);

	//go through each of the boxes and rotate them about the world frame's z axis.
	for (size_t i = 0; i < _simple_robot.size(); i++)
		{
		transformPoints(_simple_robot.at(i), rotation_HT);
		}
	}


////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB5
////////////////////////////////////////////////////////////////////////////////////////////////////////

void CRobot::update_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 220;
	_setting_window.y = 220;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 20;

	if (cvui::button(im, _setting_window.x, _setting_window.y, 100, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	if (_do_animate != 0)
	{
		int step_size = 5;
		if (_do_animate == 1)
		{
			// state 1
			if (1) { _do_animate = 2; }
		}
		else if (_do_animate == 2)
		{
			// state 2
			if (1) { _do_animate = 3; }
		}
		else if (_do_animate == 3) {
			if (1) { _do_animate = 0; init(); }
		}
	}

	cvui::update();
}

void CRobot::draw()
{
	_virtualcam.update_settings(_canvas);
	update_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB5 - SCARA robot
////////////////////////////////////////////////////////////////////////////////////////////////////////

// SCARA dimensions, all in metres (same units as the Lab 3 boxes)
static const float SCARA_LINK_LENGTH = 0.15f;		// every link is 15 cm long
static const float SCARA_LINK_SIZE = 0.03f;			// every link is 3 cm in height and depth
static const float SCARA_PEDESTAL_HEIGHT = 0.135f;	// 15 cm minus half the 3 cm link height
static const float SCARA_JOINT1_HEIGHT = 0.15f;		// joint 1 (link centre line) sits 15 cm above the world origin

// Part B-A: move value one step toward target. Returns true once it has arrived.
static bool step_toward(int& value, int target, int step)
{
	if (value < target)
		value = std::min(value + step, target);
	else if (value > target)
		value = std::max(value - step, target);

	return value == target;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Part A-A: Create the SCARA robot
// Like create_simple_robot(), but each box is left in its OWN joint frame instead of the world frame.
// draw_scara() moves them into the world every frame using the current joint values.
void CRobot::create_scara()
{
	_scara_links.clear();
	_draw_on_board = false;
	_use_virtual_camera = true;
	init(); // joints to home

	// Pedestal: 3 x 3 cm, 13.5 cm tall, standing on the world origin (stored in world frame)
	std::vector<Mat> pedestal = createBox(SCARA_LINK_SIZE, SCARA_LINK_SIZE, SCARA_PEDESTAL_HEIGHT);
	transformPoints(pedestal, createHT(
		Vec3d(0.0, 0.0, SCARA_PEDESTAL_HEIGHT / 2), // lift so the bottom face is on z = 0
		Vec3d(0.0, 0.0, 0.0)));

	// Link 1: 15 cm along its x axis. The box origin is the centre of the face at joint 1,
	// so shift the (centred) box +L/2 along x.
	std::vector<Mat> link1 = createBox(SCARA_LINK_LENGTH, SCARA_LINK_SIZE, SCARA_LINK_SIZE);
	transformPoints(link1, createHT(
		Vec3d(SCARA_LINK_LENGTH / 2, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0)));

	// Link 2: same shape as link 1, origin at the centre of the face at joint 2
	std::vector<Mat> link2 = createBox(SCARA_LINK_LENGTH, SCARA_LINK_SIZE, SCARA_LINK_SIZE);
	transformPoints(link2, createHT(
		Vec3d(SCARA_LINK_LENGTH / 2, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0)));

	// Link 3 (quill): 15 cm tall, stored in the end effector frame.
	// The origin is the centre of its bottom face (the tool tip). Tool z points DOWN,
	// so the box extends L/2 along -z to stick up from the tip.
	std::vector<Mat> quill = createBox(SCARA_LINK_SIZE, SCARA_LINK_SIZE, SCARA_LINK_LENGTH);
	transformPoints(quill, createHT(
		Vec3d(0.0, 0.0, -SCARA_LINK_LENGTH / 2),
		Vec3d(0.0, 0.0, 0.0)));

	_scara_links = { pedestal, link1, link2, quill };

	_scara_colors = {
		CV_RGB(255, 255, 255),	// pedestal = white
		CV_RGB(255, 0, 0),		// link 1 = red
		CV_RGB(0, 255, 0),		// link 2 = green
		CV_RGB(255, 0, 255),	// link 3 (quill) = magenta
		};

	// Virtual camera view that fits the whole SCARA (from the lab PDF screenshot, moved a bit
	// to clear the UI on the right). The virtual camera Reset button returns to this view.
	_virtualcam.set_default_view(3, 250, -480, 203, -101, 0, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Part A-A: Pose of every joint frame in world coordinates
// joints = (j0 deg, j1 deg, j2 deg, j3 metres)
// Returns { wT0 (world/pedestal), wT1 (link 1), wT2 (link 2), wT3 (wrist), wT4 (end effector) }
std::vector<Mat> CRobot::scara_transforms(Vec4d joints)
{
	Mat wT0 = createHT( // world frame, pedestal base
		Vec3d(0.0, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));

	Mat T01 = createHT( // up to the top of the pedestal, then rotate j0 about z
		Vec3d(0.0, 0.0, SCARA_JOINT1_HEIGHT),
		Vec3d(0.0, 0.0, joints[0]));

	Mat T12 = createHT( // out to the end of link 1, then rotate j1 about z
		Vec3d(SCARA_LINK_LENGTH, 0.0, 0.0),
		Vec3d(0.0, 0.0, joints[1]));

	Mat T23 = createHT( // out to the end of link 2, then rotate j2 about z (wrist)
		Vec3d(SCARA_LINK_LENGTH, 0.0, 0.0),
		Vec3d(0.0, 0.0, joints[2]));

	Mat T34 = createHT( // slide down by j3 (prismatic), flip 180 about x so tool z points down
		Vec3d(0.0, 0.0, -joints[3]),
		Vec3d(180.0, 0.0, 0.0));

	// Chain them together, keeping every intermediate pose for drawing
	Mat wT1 = wT0 * T01;
	Mat wT2 = wT1 * T12;
	Mat wT3 = wT2 * T23;
	Mat wT4 = wT3 * T34;

	return { wT0, wT1, wT2, wT3, wT4 };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Part A-B: Forward kinematics
// Input: joints = (j0 deg, j1 deg, j2 deg, j3 metres)
// Output: end effector pose (x m, y m, z m, theta deg). theta = heading of the tool x axis about world z.
// Same chain as scara_transforms(), but multiplied straight through without keeping intermediate poses.
Vec4d CRobot::fkin(Vec4d joints)
{
	Mat wT4 =
		createHT(Vec3d(0.0, 0.0, SCARA_JOINT1_HEIGHT), Vec3d(0.0, 0.0, joints[0])) *
		createHT(Vec3d(SCARA_LINK_LENGTH, 0.0, 0.0), Vec3d(0.0, 0.0, joints[1])) *
		createHT(Vec3d(SCARA_LINK_LENGTH, 0.0, 0.0), Vec3d(0.0, 0.0, joints[2])) *
		createHT(Vec3d(0.0, 0.0, -joints[3]), Vec3d(180.0, 0.0, 0.0));

	double x = wT4.at<float>(0, 3);
	double y = wT4.at<float>(1, 3);
	double z = wT4.at<float>(2, 3);
	double theta = atan2(wT4.at<float>(1, 0), wT4.at<float>(0, 0)) * 180.0 / CV_PI;

	return Vec4d(x, y, z, theta);
}

// Part A-C: slider values converted to the units fkin() / scara_transforms() use
Vec4d CRobot::get_joints()
{
	return Vec4d(_joint[0], _joint[1], _joint[2], _joint[3] / 1000.0); // j3 mm -> m
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Part A-C (+ Part B-A button, Part B-B checkbox): SCARA "Robot Settings" window on the right side
void CRobot::update_scara_settings(Mat& im)
{
	Point _setting_window;

	_setting_window.x = im.size().width - 220;
	_setting_window.y = 220;
	cvui::window(im, _setting_window.x, _setting_window.y, 200, 450, "Robot Settings");

	_setting_window.x += 5;
	_setting_window.y += 20;

	// Part A-C: one slider per joint
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_joint[0], -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j0");

	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_joint[1], -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j1");

	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_joint[2], -180, 180);
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j2");

	_setting_window.y += 45;
	cvui::trackbar(im, _setting_window.x, _setting_window.y, 180, &_joint[3], 0, 150); // mm
	cvui::text(im, _setting_window.x + 180, _setting_window.y + 20, "j3");

	// Part A-C: Reset button puts every joint back to 0 and stops any animation
	_setting_window.y += 50;
	if (cvui::button(im, _setting_window.x, _setting_window.y, 90, 30, "Reset"))
	{
		init();
	}

	// Part B-A: Animate button starts the state machine in animate_scara()
	if (cvui::button(im, _setting_window.x + 95, _setting_window.y, 90, 30, "Animate"))
	{
		init();
		_do_animate = 1;
	}

	// Part B-B: choose virtual camera or ArUco board
	_setting_window.y += 40;
	cvui::checkbox(im, _setting_window.x, _setting_window.y, "Draw on ArUco board", &_draw_on_board);

	// Part A-C: end effector pose from fkin()
	Vec4d pose = fkin(get_joints());

	_setting_window.y += 30;
	cvui::text(im, _setting_window.x, _setting_window.y, "End effector (fkin)");
	_setting_window.y += 20;
	cvui::printf(im, _setting_window.x, _setting_window.y, "X:  %7.1f mm", pose[0] * 1000.0);
	_setting_window.y += 18;
	cvui::printf(im, _setting_window.x, _setting_window.y, "Y:  %7.1f mm", pose[1] * 1000.0);
	_setting_window.y += 18;
	cvui::printf(im, _setting_window.x, _setting_window.y, "Z:  %7.1f mm", pose[2] * 1000.0);
	_setting_window.y += 18;
	cvui::printf(im, _setting_window.x, _setting_window.y, "Th: %7.1f deg", pose[3]);

	if (_do_animate != 0)
	{
		_setting_window.y += 25;
		cvui::printf(im, _setting_window.x, _setting_window.y, 0.4, 0xFFFF00, "Animating (step %d)", _do_animate);
	}

	// Part B-A: advance the animation one step per frame
	animate_scara();

	// Only ONE cvui::update() per frame, otherwise the buttons do not work
	cvui::update();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Part B-A: Animation state machine, one step per frame (called from update_scara_settings)
// j0: 0 -> +180 -> -180 -> 0, then j1: 0 -> +180 -> -180 -> 0, then j2: 0 -> 360, then j3: 0 -> 15 cm
void CRobot::animate_scara()
{
	int step_size = 5; // degrees (or mm for j3) per frame

	switch (_do_animate)
	{
	case 0: // not animating
		break;

	// joint 0
	case 1: if (step_toward(_joint[0], 180, step_size)) { _do_animate = 2; } break;
	case 2: if (step_toward(_joint[0], -180, step_size)) { _do_animate = 3; } break;
	case 3: if (step_toward(_joint[0], 0, step_size)) { _do_animate = 4; } break;

	// joint 1
	case 4: if (step_toward(_joint[1], 180, step_size)) { _do_animate = 5; } break;
	case 5: if (step_toward(_joint[1], -180, step_size)) { _do_animate = 6; } break;
	case 6: if (step_toward(_joint[1], 0, step_size)) { _do_animate = 7; } break;

	// joint 2: one full turn 0 -> 360. The slider only goes -180..180, so wrap past 180.
	case 7:
		_anim_count += step_size;
		_joint[2] = (_anim_count > 180) ? _anim_count - 360 : _anim_count;
		if (_anim_count >= 360) { _joint[2] = 0; _do_animate = 8; }
		break;

	// joint 3 (prismatic): 0 -> 150 mm
	case 8: if (step_toward(_joint[3], 150, step_size)) { _do_animate = 9; } break;

	// done: back to home, like the original template's last state
	default:
		_do_animate = 0;
		init();
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Part A-A / Part B-B: Draw the SCARA robot
// Virtual camera (default): grey background, Virtual Camera Settings window.
// ArUco board (checkbox ticked): webcam image background, robot sits on the board origin.
void CRobot::draw_scara()
{
	_canvas = cv::Mat::zeros(_image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	// Part B-B: drawBox() / drawCoord() pick the camera from this flag
	_use_virtual_camera = !_draw_on_board;

	bool have_pose = true;
	Mat base = createHT( // virtual camera: robot world = drawing world
		Vec3d(0.0, 0.0, 0.0),
		Vec3d(0.0, 0.0, 0.0));

	if (_draw_on_board)
	{
		Mat im_raw;

		// Part B-B 1) Get the webcam image and the board pose (same steps as draw_simple_robot_lab4)
		_realcam.get_image(im_raw);
		_realcam.detect_markers(0);

		// Part B-B 2) Webcam image at the top left, where its pixel coordinates line up
		if (!im_raw.empty() && im_raw.cols <= _canvas.cols && im_raw.rows <= _canvas.rows)
		{
			im_raw.copyTo(_canvas(Rect(0, 0, im_raw.cols, im_raw.rows)));
		}
		else
		{
			cvui::text(_canvas, 20, 20, "No webcam image", 0.6, 0xFF6060);
		}

		// Part B-B 3) Without a board pose there is nothing to project onto
		have_pose = !(_realcam.get_rvec().empty() || _realcam.get_tvec().empty());
		if (!have_pose)
		{
			cvui::text(_canvas, 20, 50, "No ArUco board pose yet - robot not drawn", 0.6, 0xFF6060);
		}

		// Part B-B 4) Same board alignment as create_simple_robot_lab4 so robot z points up off the board
		base = createHT(
			Vec3d(0.0, 0.0, 0.0),
			Vec3d(180.0, 0.0, 90.0));
	}

	if (have_pose)
	{
		// Part A-A: joint frames from the current slider values
		std::vector<Mat> T = scara_transforms(get_joints());

		// Which frame each box lives in: pedestal->wT0, link1->wT1, link2->wT2, quill->wT4 (end effector)
		int link_frame[] = { 0, 1, 2, 4 };

		for (size_t i = 0; i < _scara_links.size(); i++)
		{
			// Deep copy (clone) so the model stays in its own frame. A plain vector copy shares
			// the Mat data, and transformPoints() would then move the stored model every frame.
			std::vector<Mat> box;
			for (const Mat& pt : _scara_links.at(i))
				box.push_back(pt.clone());

			transformPoints(box, base * T.at(link_frame[i]));
			drawBox(_canvas, box, _scara_colors.at(i));
		}

		// Coordinate axes at the world origin and at every joint frame
		for (size_t i = 0; i < T.size(); i++)
		{
			std::vector<Mat> coord = createCoord();
			transformPoints(coord, base * T.at(i));
			drawCoord(_canvas, coord);
		}

		// Part A-B check: yellow circle at the fkin() position. It should sit on the end effector axes.
		Vec4d pose = fkin(get_joints());
		Mat tip = base * (Mat1f(4, 1) << (float)pose[0], (float)pose[1], (float)pose[2], 1.0f);
		Point2f tip2d;
		if (_use_virtual_camera)
			_virtualcam.transform_to_image(tip, tip2d);
		else
			_realcam.transform_to_image(tip, tip2d);
		circle(_canvas, tip2d, 6, CV_RGB(255, 255, 0), 1);
	}

	// Draw the UI on the right side (camera settings for whichever camera is in use)
	if (_draw_on_board)
		_realcam.update_settings(_canvas);
	else
		_virtualcam.update_settings(_canvas);

	update_scara_settings(_canvas);

	cv::imshow(CANVAS_NAME, _canvas);
}
