#include "stdafx.h"

#include "CameraVirtual.h"
#include <cmath>

#include "cvui.h"

CCameraVirtual::CCameraVirtual()
{
	// Initialize with a default camera image size 
	init(Size(1000, 600));
}

CCameraVirtual::~CCameraVirtual()
{
}

void CCameraVirtual::init (Size image_size)
{
	//////////////////////////////////////
	// CVUI interface default variables

	_cam_setting_f = 0;

	_cam_setting_x = 0; // units in mm
	_cam_setting_y = 0; // units in mm
	_cam_setting_z = 0; // units in mm

	_cam_setting_roll = 0; // units in degrees, gamma Y
	_cam_setting_pitch = 0; // units in degrees, beta B
	_cam_setting_yaw = 0; // units in degrees, yaw A

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046; // Units of m
	_principal_point = Point2f(image_size / 2);
	
	calculate_intrinsic();

	//////////////////////////////////////
	// Virtual Camera Extrinsic

	calculate_extrinsic();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// LAB3
////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: Setup camera intrinsic matrix, lec05 pg6
void CCameraVirtual::calculate_intrinsic()
{
	double u_o = _principal_point.x;
	double v_o = _principal_point.y;
	double p_u = _pixel_size;
	double p_v = _pixel_size;
	double f = _cam_setting_f / 1000; // convert to m
	_cam_virtual_intrinsic = (Mat1f(3, 4) << 
		1/p_u*f,		0,			u_o,	0, 
		0,			1/p_v*f,	v_o,	0, 
		0,			0,			1,		0);
}

// TODO: Setup camera extrinsic matrix
void CCameraVirtual::calculate_extrinsic()
{
	double Y = _cam_setting_roll; // gamma
	double B = _cam_setting_pitch; // beta
	double A = _cam_setting_yaw; // alpha

	double _x = _cam_setting_x; // x_Boriginal
	double _y = _cam_setting_y; // y_Boriginal
	double _z = _cam_setting_z; // z_Boriginal

	double cA = cos(A);
	double cB = cos(B);
	double cY = cos(Y);
	double sA = sin(A);
	double sB = sin(B);
	double sY = sin(Y);

	_cam_virtual_extrinsic = (Mat1f(4, 4) <<
		cA * cB, cA * sB * sY - sA * cY, cA * sB * cY + sA * sY, _x,
		sA * cB, sA * sB * sY + cA * cY, sA * sB * cY - cA * sY, _y,
		-sB, cB * sY, cB * cY, _z,
		0, 0, 0, 1);


	//_cam_virtual_extrinsic = (Mat1f(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);


}

// TODO: transform single 3D point to 2D image point
void CCameraVirtual::transform_to_image(Mat pt3d_mat, Point2f& pt)
{
}

// TODO: transform vector of 3D points to vector of 2D image points
void CCameraVirtual::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
}

void CCameraVirtual::update_settings(Mat& im)
{
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 375, "Virtual Camera Settings");

	_camera_setting_window.x = 5;
	_camera_setting_window.y = 20;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_f, 1, 20);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "F");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_x, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "X");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_y, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_z, -500, 500);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Z");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_roll, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "R");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_pitch, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "P");

	_camera_setting_window.y += 45;
	cvui::trackbar(im, _camera_setting_window.x, _camera_setting_window.y, 180, &_cam_setting_yaw, -180, 180);
	cvui::text(im, _camera_setting_window.x + 180, _camera_setting_window.y + 20, "Y");

	_camera_setting_window.y += 45;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Reset"))
	{
		init(im.size());
	}

	// Use this line if only this settings window in use
	// cvui::update();

	//////////////////////////////
	// Update camera model

	calculate_intrinsic();
	calculate_extrinsic();
}
