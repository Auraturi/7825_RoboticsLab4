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
	float u_o = _principal_point.x;
	float v_o = _principal_point.y;
	float p_u = _pixel_size;
	float p_v = _pixel_size;
	float f = (float)_cam_setting_f / 1000.f; // convert to m
	_cam_virtual_intrinsic = (Mat1f(3, 4) << 
		1/p_u*f,		0,			u_o,	0, 
		0,			1/p_v*f,	v_o,	0, 
		0,			0,			1,		0);
}


// TODO: Setup camera extrinsic matrix
void CCameraVirtual::calculate_extrinsic()
{
	float Y = _cam_setting_roll * CV_PI/ 180.0; // gamma
	float B = _cam_setting_pitch * CV_PI/ 180.0; // beta
	float A = _cam_setting_yaw * CV_PI/ 180.0; // alpha

	float _x = _cam_setting_x / 1000.f; // x_Boriginal
	float _y = _cam_setting_y / 1000.f; // y_Boriginal
	float _z = _cam_setting_z / 1000.f; // z_Boriginal

	float cA = cos(A);
	float cB = cos(B);
	float cY = cos(Y);
	float sA = sin(A);
	float sB = sin(B);
	float sY = sin(Y);

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
	/*
	So it's simple.
	make a new matrix, of intrinsic x extrinsic.
	Then take the point, and transform it by the new matrix.
	easy.
	
	*/
	Mat extInv = _cam_virtual_extrinsic.inv();
	Mat transformMatrix = _cam_virtual_intrinsic * extInv;
	Mat uvw = transformMatrix * pt3d_mat; //the transformation into 3x1 matrix

	float u_h = uvw.at<float>(0, 0);
	float v_h = uvw.at<float>(1, 0);
	float w_h = uvw.at<float>(2, 0);

	float u = u_h / w_h;
	float v = v_h / w_h;
	pt = Point2f(u, v);
	
}

// TODO: transform vector of 3D points to vector of 2D image points
void CCameraVirtual::transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d)
{
	/*
Make a big matrix of all points, then transform it by the transformation matrix, then divide by w to get u and v.
*/
	pts2d.clear();

	Mat extInv = _cam_virtual_extrinsic.inv();
	Mat transformMatrix = _cam_virtual_intrinsic * extInv;
	Mat bigMatrix(4, pts3d_mat.size(), CV_32F);

	for (size_t i = 0; i < pts3d_mat.size(); i++) {
		bigMatrix.at<float>(0, i) = pts3d_mat.at(i).at<float>(0, 0);
		bigMatrix.at<float>(1, i) = pts3d_mat.at(i).at<float>(1, 0);
		bigMatrix.at<float>(2, i) = pts3d_mat.at(i).at<float>(2, 0);
		bigMatrix.at<float>(3, i) = 1.0f;
	}

	Mat uvw = transformMatrix * bigMatrix; //the transformation into 3xN matrix
	for (size_t i = 0; i < pts3d_mat.size(); i++) {
		float u_h = uvw.at<float>(0, i);
		float v_h = uvw.at<float>(1, i);
		float w_h = uvw.at<float>(2, i);
		float u = u_h / w_h;
		float v = v_h / w_h;
		pts2d.push_back(Point2f(u, v));
	}

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

//a copy of the robot 's createHT function, but for the virtual camera
Mat CCameraVirtual::createHT_VCam(Vec3d t, Vec3d r) //t is the translation vector; r is the rotation vector, each are 3 element vectors
//rotation r passes (gamma y, beta B, alpha a) in doubles
// translation t passes (x, y, z) in doubles
//wRb_xyz (Y,B,A) = Rz(A)*Ry(B)*Rx(Y) 
{

	float Y = r[0] * CV_PI/ 180.0; // gamma
	float B = r[1] * CV_PI/ 180.0; // beta
	float A = r[2] * CV_PI/ 180.0; // alpha

	float _x = t[0]; // x_Boriginal
	float _y = t[1]; // y_Boriginal
	float _z = t[2]; // z_Boriginal

	float cA = cos(A);
	float cB = cos(B);
	float cY = cos(Y);
	float sA = sin(A);
	float sB = sin(B);
	float sY = sin(Y);

	return (Mat1f(4, 4) <<
		cA * cB, cA * sB * sY - sA * cY, cA * sB * cY + sA * sY, _x,
		sA * cB, sA * sB * sY + cA * cY, sA * sB * cY - cA * sY, _y,
		-sB, cB * sY, cB * cY, _z,
		0, 0, 0, 1);


}