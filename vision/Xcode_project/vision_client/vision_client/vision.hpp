//
//  vision.hpp
//  vision_client
//
//  Created by 童博涵 on 2021/2/27.
//

#ifndef vision_hpp
#define vision_hpp

#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

const int CAMERA_MODE = 0;
const int VIDEO_MODE = 1;

class Vision{
public:
	Vision(int index);
	Vision(const string& filename);
	~Vision();
	VideoCapture cap;
	void estimatePose_1(Mat& image, double length, Mat& light_roi);
	void estimatePose_2(Mat& image, double length, Mat& light_roi);
	void estimatePose_3(Mat& image, Mat& light_roi);
	int lightJudge(Mat& image, Mat& light_roi);
private:
	int vision_mode;
	int width, height;
	Mat camMatrix, distCoeffs;
	Ptr<aruco::Dictionary> aruco_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	void writeCameraParam(string filename);
	bool readCameraParam(string filename);
	void readArucoMap(string filename, vector<int>& ids, vector<vector<Point3f>>& pts);
	void detectLight(Mat& image, int markerId, vector<Point2f> markerCorner, Point2f& left_corner, Point2f& right_corner, Mat& light_roi);
};

#endif /* vision_hpp */
