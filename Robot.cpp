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

	init();
}

CRobot::~CRobot()
{
}

void CRobot::init()
{
	// reset variables
	_do_animate = 0;
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

	// If Virtual Camera
	//_virtualcam.transform_to_image(box3d, box2d); //Commented out.
	// If Real Camera
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

	// If Virtual Camera
	//_virtualcam.transform_to_image(coord3d.at(0), O);
	//_virtualcam.transform_to_image(coord3d.at(1), X);
	//_virtualcam.transform_to_image(coord3d.at(2), Y);
	//_virtualcam.transform_to_image(coord3d.at(3), Z);

	// If Real Camera
	//_realcam.transform_to_image();
	_realcam.transform_to_image(coord3d.at(0), O);
	_realcam.transform_to_image(coord3d.at(1), X);
	_realcam.transform_to_image(coord3d.at(2), Y);
	_realcam.transform_to_image(coord3d.at(3), Z);

	line(im, O, X, CV_RGB(255, 0, 0), 1); // X=RED
	line(im, O, Y, CV_RGB(0, 255, 0), 1); // Y=GREEN
	line(im, O, Z, CV_RGB(0, 0, 255), 1); // Z=BLUE
}

void CRobot::create_simple_robot()
{
	//below is added by Ted
	_simple_robot.clear();

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

void CRobot::create_simple_robot_lab4()
	{
	//below is added by Ted
	_simple_robot.clear();

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
