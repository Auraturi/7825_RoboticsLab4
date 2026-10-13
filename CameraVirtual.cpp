#include "stdafx.h"

#include "CameraVirtual.h"
#include <cmath>
#include <limits>

#include "cvui.h"

CCameraVirtual::CCameraVirtual()
{
	// Initialize with a default camera image size 
	init(Size(1280, 720));
}

CCameraVirtual::~CCameraVirtual()
{
}

void CCameraVirtual::init (Size image_size)
{
	//////////////////////////////////////
	// CVUI interface default variables

	_cam_setting_f = 0;

	_cam_setting_x = 184; // units in mm
	_cam_setting_y = -368; // units in mm
	_cam_setting_z = -32; // units in mm

	_cam_setting_roll = -78; // units in degrees, gamma Y
	_cam_setting_pitch = 0; // units in degrees, beta B
	_cam_setting_yaw = 18; // units in degrees, yaw A

	lookAt = false; //lookAt functionality, if true, the camera will look at the origin (0,0,0) when moving around. impacts the extrinsic matrix calculation.

	//////////////////////////////////////
	// Virtual Camera intrinsic

	_cam_setting_f = 3; // Units are mm, convert to m by dividing 1000

	_pixel_size = 0.0000046f; // Units of m
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
	if (lookAt) {
		// If lookAt mode is enabled, let lookAtOrigin compute and set the
		// camera-to-world extrinsic directly and skip the angle-based rebuild.
		//lookAtOrigin();
		lookAtOrigin2();
		return;
	}

	float Y = static_cast<float>(_cam_setting_roll * CV_PI / 180.0); // gamma
	float B = static_cast<float>(_cam_setting_pitch * CV_PI / 180.0); // beta
	float A = static_cast<float>(_cam_setting_yaw * CV_PI / 180.0); // alpha

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
	CV_Assert(pts3d_mat.size() <= static_cast<size_t>(std::numeric_limits<int>::max())); //safety check, ensures its not too big for an int
	Mat bigMatrix(4, static_cast<int>(pts3d_mat.size()), CV_32F);

	for (size_t i = 0; i < pts3d_mat.size(); i++) {
		bigMatrix.at<float>(0, static_cast<int>(i)) = pts3d_mat.at(i).at<float>(0, 0);
		bigMatrix.at<float>(1, static_cast<int>(i)) = pts3d_mat.at(i).at<float>(1, 0);
		bigMatrix.at<float>(2, static_cast<int>(i)) = pts3d_mat.at(i).at<float>(2, 0);
		bigMatrix.at<float>(3, static_cast<int>(i)) = 1.0f;
	}

	Mat uvw = transformMatrix * bigMatrix; //the transformation into 3xN matrix
	for (size_t i = 0; i < pts3d_mat.size(); i++) {
		float u_h = uvw.at<float>(0, static_cast<int>(i));
		float v_h = uvw.at<float>(1, static_cast<int>(i));
		float w_h = uvw.at<float>(2, static_cast<int>(i));
		float u = u_h / w_h;
		float v = v_h / w_h;
		pts2d.push_back(Point2f(u, v));
	}

}

void CCameraVirtual::update_settings(Mat& im)
{
	bool track_board = false;
	Point _camera_setting_window;

	cvui::window(im, _camera_setting_window.x, _camera_setting_window.y, 200, 425, "Virtual Camera Settings");

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

	_camera_setting_window.y += 50;
	if (cvui::button(im, _camera_setting_window.x, _camera_setting_window.y, 100, 30, "Look At"))
	{
		//The code, requires debounce, to ensure it only runs once per click, and not every frame while the button is held down.
		lookAt = !lookAt;
	}

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

void CCameraVirtual::lookAtOrigin()
{
	// Robust look-at implementation: compute rotation so the camera's forward axis
	// points toward the world origin, then extract Euler angles using the same
	// R = Rz(A) * Ry(B) * Rx(Y) convention used by calculate_extrinsic().
	// Camera position in meters
	Vec3d camPos(_cam_setting_x / 1000.0, _cam_setting_y / 1000.0, _cam_setting_z / 1000.0);

	// Forward vector from camera to origin (world coords)
	Vec3d forward = -camPos;
	double flen = cv::norm(forward);
	if (flen < 1e-9) {
		// Camera is essentially at the origin; leave angles unchanged
		return;
	}
	forward /= flen;

	// World-up (choose Z up). If forward is nearly parallel to up, pick an alternate up
	Vec3d worldUp(0.0, 0.0, -1.0);
	if (std::abs(forward.dot(worldUp)) > 0.9999) {
		worldUp = Vec3d(0.0, 1.0, 0.0);
	}

	// Build orthonormal basis: right, up, forward (columns of Rcw)
	Vec3d right = worldUp.cross(forward);
	double rlen = cv::norm(right);
	if (rlen < 1e-9) {
		// Fallback: cannot build basis
		return;
	}
	right /= rlen;
	Vec3d up = forward.cross(right);
	// Flip up to match image coordinate convention (prevents upside-down projection)
	//up = -up;

	// Rotation matrix from camera to world (columns are camera axes in world coords)
	// Rcw = [ right | up | forward ]
	double R00 = right[0], R10 = right[1], R20 = right[2];
	double R01 = up[0],    R11 = up[1],    R21 = up[2];
	double R02 = forward[0],R12 = forward[1],R22 = forward[2];

	// Extract Euler angles matching calculate_extrinsic() construction:
	// R = Rz(A) * Ry(B) * Rx(Y)
	// From that, R20 = -sin(B)
	double B_rad = std::atan2(-R20, std::sqrt(R00*R00 + R10*R10));
	double cosB = std::cos(B_rad);
	double A_rad = 0.0;
	double Y_rad = 0.0;
	if (std::abs(cosB) > 1e-6) {
		A_rad = std::atan2(R10 / cosB, R00 / cosB);
		Y_rad = std::atan2(R21 / cosB, R22 / cosB);
	} else {
		// Gimbal lock: set yaw from R01/R11 and roll = 0
		A_rad = std::atan2(-R01, R11);
		Y_rad = 0.0;
	}

	double A_deg = A_rad * 180.0 / CV_PI;
	double B_deg = B_rad * 180.0 / CV_PI;
	double Y_deg = Y_rad * 180.0 / CV_PI;

	// Update camera integer settings (round to nearest int)
	_cam_setting_yaw = static_cast<int>(std::round(A_deg));
	_cam_setting_pitch = static_cast<int>(std::round(B_deg));
	_cam_setting_roll = static_cast<int>(std::round(Y_deg));

	// Also set the camera-to-world extrinsic directly from the computed basis
	// so calculate_extrinsic() can return early in lookAt mode.
	float tx = static_cast<float>(camPos[0]);
	float ty = static_cast<float>(camPos[1]);
	float tz = static_cast<float>(camPos[2]);

	_cam_virtual_extrinsic = (Mat1f(4, 4) <<
		static_cast<float>(R00), static_cast<float>(R01), static_cast<float>(R02), tx,
		static_cast<float>(R10), static_cast<float>(R11), static_cast<float>(R12), ty,
		static_cast<float>(R20), static_cast<float>(R21), static_cast<float>(R22), tz,
		0.0f, 0.0f, 0.0f, 1.0f);
}


void CCameraVirtual::lookAtOrigin2()
{
	// Camera position in meters. The target is the world origin.
	const Vec3d cameraPosition(_cam_setting_x / 1000.0,
		_cam_setting_y / 1000.0,
		_cam_setting_z / 1000.0);
	Vec3d forward = -cameraPosition;
	const double forwardLength = cv::norm(forward);
	if (forwardLength < 1e-9)
		return;
	forward /= forwardLength;

	// Build the camera axes. These become the columns of the rotation matrix.
	Vec3d referenceUp(0.0, 0.0, -1.0);
	if (std::abs(forward.dot(referenceUp)) > 0.9999)
		referenceUp = Vec3d(0.0, 1.0, 0.0);

	Vec3d right = referenceUp.cross(forward);
	const double rightLength = cv::norm(right);
	if (rightLength < 1e-9)
		return;
	right /= rightLength;
	const Vec3d up = forward.cross(right);

	// Rcw maps camera coordinates to world coordinates:
	// column 0 = right, column 1 = up, column 2 = forward.
	const Matx33d Rcw(
		right[0], up[0], forward[0],
		right[1], up[1], forward[1],
		right[2], up[2], forward[2]);

	// Extract the same yaw, pitch, and roll convention used elsewhere.
	const double pitch = std::atan2(-Rcw(2, 0),
		std::sqrt(Rcw(0, 0) * Rcw(0, 0) + Rcw(1, 0) * Rcw(1, 0)));
	const double cosPitch = std::cos(pitch);
	const double yaw = std::abs(cosPitch) > 1e-6
		? std::atan2(Rcw(1, 0) / cosPitch, Rcw(0, 0) / cosPitch)
		: std::atan2(-Rcw(0, 1), Rcw(1, 1));
	const double roll = std::abs(cosPitch) > 1e-6
		? std::atan2(Rcw(2, 1) / cosPitch, Rcw(2, 2) / cosPitch)
		: 0.0;

	_cam_setting_yaw = static_cast<int>(std::round(yaw * 180.0 / CV_PI));
	_cam_setting_pitch = static_cast<int>(std::round(pitch * 180.0 / CV_PI));
	_cam_setting_roll = static_cast<int>(std::round(roll * 180.0 / CV_PI));

	// Homogeneous camera-to-world transform: [ Rcw | cameraPosition ].
	_cam_virtual_extrinsic = (Mat1f(4, 4) <<
		static_cast<float>(Rcw(0, 0)), static_cast<float>(Rcw(0, 1)), static_cast<float>(Rcw(0, 2)), static_cast<float>(cameraPosition[0]),
		static_cast<float>(Rcw(1, 0)), static_cast<float>(Rcw(1, 1)), static_cast<float>(Rcw(1, 2)), static_cast<float>(cameraPosition[1]),
		static_cast<float>(Rcw(2, 0)), static_cast<float>(Rcw(2, 1)), static_cast<float>(Rcw(2, 2)), static_cast<float>(cameraPosition[2]),
		0.0f, 0.0f, 0.0f, 1.0f);

}





//a copy of the robot 's createHT function, but for the virtual camera
Mat CCameraVirtual::createHT_VCam(Vec3d t, Vec3d r) //t is the translation vector; r is the rotation vector, each are 3 element vectors
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
		cA * cB, cA * sB * sY - sA * cY, cA * sB * cY + sA * sY, _x,
		sA * cB, sA * sB * sY + cA * cY, sA * sB * cY - cA * sY, _y,
		-sB, cB * sY, cB * cY, _z,
		0, 0, 0, 1);


}