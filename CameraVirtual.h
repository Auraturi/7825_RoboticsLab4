#pragma once

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class CCameraVirtual
{
public:
	CCameraVirtual();
	~CCameraVirtual();

private:

	// Virtual Camera
	float _pixel_size;
	Point2f _principal_point;
	Mat _cam_virtual_intrinsic;
	Mat _cam_virtual_extrinsic;

	void calculate_intrinsic();
	void calculate_extrinsic();
	void lookAtOrigin(); // for lookAt functionality, if true, the camera will look at the origin (0,0,0) when moving around.
	void lookAtOrigin2(); // same as above, but hopefully thinner, leaner.

	// CVUI setting variables
	int _cam_setting_f;
	int _cam_setting_x;
	int _cam_setting_y;
	int _cam_setting_z;
	int _cam_setting_roll;
	int _cam_setting_pitch;
	int _cam_setting_yaw;

	bool lookAt; // for lookAt functionality, if true, the camera will look at the origin (0,0,0) when moving around.

	// Lab 5: view that init() / the Reset button return to (f in mm, x y z in mm, angles in degrees)
	int _default_f;
	int _default_x;
	int _default_y;
	int _default_z;
	int _default_roll;
	int _default_pitch;
	int _default_yaw;

public:
	void init(Size image_size);

	void transform_to_image(Mat pt3d_mat, Point2f& pt);
	void transform_to_image(std::vector<Mat> pts3d_mat, std::vector<Point2f>& pts2d);

	void update_settings(Mat& im);
	Mat createHT_VCam(Vec3d t, Vec3d r); //added from CRobot to be able to use here if needed.

	// Lab 5: change the default view (used by create_scara so the whole SCARA is in frame) and apply it now
	void set_default_view(int f, int x, int y, int z, int roll, int pitch, int yaw);
};

