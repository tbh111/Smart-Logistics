//
//  main.cpp
//  vision_client
//
//  Created by 童博涵 on 2021/2/18.
//

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

#define MODE_CAMERA 0
#define MODE_VIDEO 1

using namespace std;
using namespace cv;

void writeCameraParam(string filename, Mat& camMatrix, Mat& distCoeffs){
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "camera_matrix" << camMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs.release();
}

bool readCameraParam(string filename, Mat& camMatrix, Mat& distCoeffs){
	FileStorage fs(filename, FileStorage::READ);
	if(!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

void readArucoMap(string filename, vector<int>& ids, vector<vector<Point3f>>& pts){
	ifstream file(filename, ifstream::in);
	if (!file) {
		cout << "open file failed" << endl;
		return;
	}
	string line, id, loc_x, loc_y;
	vector<Point3f> pt_v;
	Point3f pt;
	
	while (getline(file, line)) {
		istringstream sin(line);
		vector<string> datas;
		string data;
		while (getline(sin, data, ';')) {
			datas.push_back(data);
		}
		id = datas[0];
		for(int i = 1; i < 6; i++) {
			string str = datas[i];
			auto pos = str.find(",");
			if (pos != string::npos) {
//				cout << str.substr(0, pos) << endl;
//				cout << str.substr(pos+1, str.length()) << endl;
				loc_x = str.substr(0, pos);
				loc_y = str.substr(pos+1, str.length());
				pt.x = (atof(loc_x.c_str()));
				pt.y = (atof(loc_y.c_str()));
				pt.z = 0.0;
				pt_v.push_back(pt);
			}
			
		}
		ids.push_back(atoi(id.c_str()));
		pts.push_back(pt_v);
		pt_v.clear();
	}
//	for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++) {
//		cout << *it << endl;
//	}
}

void estimatePose_1(Mat& image, double length, Mat camMat, Mat dist, Ptr<aruco::Dictionary> dictionary){
	// 使用aruco内置函数估计相机位姿
	
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
	vector<Vec3d> rvecs, tvecs;
	Mat tvecs_m(3, 1, CV_64FC1);
	Mat rotM(3,3,CV_64FC1);// 转换为矩阵
	Mat rotT(3,3,CV_64FC1);
	aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
	
	if (markerIds.size() == 0) {
		cout << "no aruco marker detected" << endl;
	} else {
		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
		aruco::estimatePoseSingleMarkers(markerCorners, length, camMat, dist, rvecs, tvecs);
		for (int i = 0; i < markerIds.size(); i++) {
			aruco::drawAxis(image, camMat, dist, rvecs, tvecs, 0.1);
			tvecs_m.at<double>(0,0) = tvecs[i][0];
			tvecs_m.at<double>(1,0) = tvecs[i][1];
			tvecs_m.at<double>(2,0) = tvecs[i][2];
			Rodrigues(rvecs, rotM);
			Rodrigues(tvecs, rotT);
			
			// 计算旋转角度
			double theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
			double theta_y = atan2(-rotM.at<double>(2, 0),
			sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
			double theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));
			theta_x = theta_x * (180 / M_PI);
			theta_y = theta_y * (180 / M_PI);
			theta_z = theta_z * (180 / M_PI);
			putText(image, "angle: " + to_string(theta_x) + " " + to_string(theta_y) + " " + to_string(theta_z), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,0,0),1);
			
			// 计算坐标 参考:https://blog.csdn.net/cocoaqin/article/details/77848588
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
			cv2eigen(rotM, R_n);
			cv2eigen(tvecs_m, T_n);
			Eigen::Vector3f P_oc;
			P_oc = -R_n.inverse() * T_n;
			string str1 = to_string(P_oc(0));
			string str2 = to_string(P_oc(1));
			string str3 = to_string(P_oc(2));
			putText(image, "pos: " + str1 + " " + str2 + " " + str3, Point(10, 45), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
		}
	}
}

void estimatePose_2(Mat& image, double length, Mat camMat, Mat dist, Ptr<aruco::Dictionary> dictionary, Mat& light_roi){
	// 使用solvePnP估计相机位姿
	
	vector<Point3f> markerCorners3d;
	vector<Point2f> markerCorners2d;
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
	Mat rvecs;
	Mat_<float> tvecs;
	Mat raux, taux;
	Mat_<float> rotM(3, 3);
	markerCorners3d.push_back(Point3f(-length/2,  length/2, 0));
	markerCorners3d.push_back(Point3f( length/2,  length/2, 0));
	markerCorners3d.push_back(Point3f( length/2, -length/2, 0));
	markerCorners3d.push_back(Point3f(-length/2, -length/2, 0));
	
	aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
	if (markerIds.size() == 0) {
		cout << "no aruco marker detected" << endl;
	} else {
		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
		for (int i = 0; i < markerIds.size(); i++) {
			markerCorners2d = markerCorners[i];
			solvePnP(markerCorners3d, markerCorners2d, camMat, dist, raux, taux);
			aruco::drawAxis(image, camMat, dist, rvecs, tvecs, 0.1);
			raux.convertTo(rvecs, CV_32F);
			taux.convertTo(tvecs, CV_32F);
			Rodrigues(rvecs, rotM);
			
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
			cv2eigen(rotM, R_n);
			cv2eigen(tvecs, T_n);
			Eigen::Vector3f P_oc;
			P_oc = -R_n.inverse()*T_n;
			string str1 = to_string(P_oc(0));
			string str2 = to_string(P_oc(1));
			string str3 = to_string(P_oc(2));
			putText(image, "pos: " + str1 + " " + str2 + " " + str3, Point(10, 45), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
		}
	}
}

void estimatePose_3(Mat& image, Mat camMat, Mat dist, Ptr<aruco::Dictionary> dictionary){
	// 多个aruco，给定map
	
	// 读取aruco地图
	vector<int> totalIds;
	vector<vector<Point3f>> totalPts;
	readArucoMap("aruco_video.txt", totalIds, totalPts); // aruco_video为视频中的map   aruco_test为打印的map
	if (totalIds.size() != totalPts.size()) {
		cout << "aruco file error" << endl;
		return;
	}
	else {
		cout << "read aruco file success" << endl;
	}
//	for (int i = 0; i < totalIds.size(); i++) {
//		cout << totalIds[i] << endl;
//		for (int j = 0; j < totalPts[i].size(); j++) {
//			cout << totalPts[i][j];
//		}
//		cout << endl;
//	}
	
	vector<Point3f> markerCorners3d;
	vector<Point2f> markerCorners2d;
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
	Mat rvecs;
	Mat_<float> tvecs;
	Mat raux, taux;
	Mat_<float> rotM(3, 3);
	int marker_id_current;
	int marker_index = -1;
	double length = 0.0;
	
	aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
	if (markerIds.size() == 0) {
		cout << "no aruco marker detected" << endl;
	} else {
		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
		for (int i = 0; i < markerIds.size(); i++) {
			marker_id_current = markerIds[i];
			markerCorners2d = markerCorners[i];
			for (int j = 0; j < totalIds.size(); j++) {
				if (totalIds[j] == marker_id_current) {
					marker_index = j;
				}
			}
			if (marker_index < 0) {
				cout << "detect error" << endl;
				continue;
			}
			else {
				length = abs((totalPts[marker_index][0].x-totalPts[marker_index][1].x)*2)*0.01;
				for (int k = 1; k < 5; k++) {
					markerCorners3d.push_back(totalPts[marker_index][k]);
				}
				
			}
			solvePnP(markerCorners3d, markerCorners2d, camMat, dist, raux, taux);

			//aruco::drawAxis(image, camMat, dist, rvecs, tvecs, 0.1);
			raux.convertTo(rvecs, CV_32F);
			taux.convertTo(tvecs, CV_32F);
			Rodrigues(rvecs, rotM);

			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
			cv2eigen(rotM, R_n);
			cv2eigen(tvecs, T_n);
			Eigen::Vector3f P_oc;
			P_oc = -R_n.inverse()*T_n;
			string str1 = to_string(P_oc(0)-totalPts[marker_index][0].x);
			string str2 = to_string(P_oc(1)-totalPts[marker_index][0].y);
			string str3 = to_string(P_oc(2)-totalPts[marker_index][0].z);
			putText(image, "pos_x: " + str1, Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
			putText(image, "pos_y: " + str2, Point(10, 45), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
			putText(image, "pos_z: " + str3, Point(10, 65), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
			cout << str3 << endl;
			markerCorners3d.clear();
		}
	}
	
}
int main(int argc, char** argv)
{
	// 写入相机内参
//	Mat cam = (Mat_<double>(3,3) << 228.82, 0, 601.32, 0, 228.59, 363.39, 0, 0, 1);
//	Mat dist = (Mat_<double>(4,1) << 0.00490, 0.00105, 0.00012, 0.00047);
//	writeCameraParam("cameraConfigVideo.yaml", cam, dist);
	
	// 读取相机内参
	Mat cam, dist;
	if(!readCameraParam("cameraConfigVideo.yaml", cam, dist)){
		cout << "read camera config file error" << endl;
		return 1;
	}
	else
		cout << "read camera config file success" << endl;
	
	int mode = MODE_VIDEO;
	Ptr<aruco::Dictionary> aruco_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	
	if (mode == MODE_CAMERA) {
		VideoCapture cap(0);
		Mat frame;
		Mat light_roi;
		namedWindow("camera");
		
	//	double markerLength = 0.18;
		double rate = 0.0;

		while (true) {
			if (!cap.isOpened()) {
				cout << "cannot open camera" << endl;
				break;
			}
			cap.read(frame);
			rate = cap.get(CAP_PROP_FPS);
			putText(frame, "fps: " + to_string(rate), Point(10,15), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255,0,0));
			estimatePose_3(frame, cam, dist, aruco_dict);
			imshow("camera", frame);
			if (waitKey(1) == 'q') {
				break;
			}
		}
	}
	else if (mode == MODE_VIDEO) {
		VideoCapture cap("/Users/tongbohan/Desktop/line20191219_2.avi"); // 检查视频是否成功打开
		if (!cap.isOpened()){
			cout<<"read video error"<<endl;
			return 1;
		}
		// 取得帧速率
		double rate = cap.get(CAP_PROP_FPS);
		// 跳转到第1000帧开始读取视频
		//double position = 1000.0;
		double position = 1.0;
		cap.set(CAP_PROP_POS_FRAMES, position);

		Mat frame;
		
		// 根据帧速率计算帧之间的等待时间，单位为ms
		int delay = 1000/rate;
		// 循环遍历视频中的全部帧
		while (true) {
		// 读取下一帧(如果有)
			if (!cap.read(frame)){
				cout << "video ends" << endl;
				break;
			}
			estimatePose_3(frame, cam, dist, aruco_dict);
			imshow("camera", frame);
			if (waitKey(delay) == 'q') {
				break;
			}
		}
	}
	else {
		
	}

	
    return 0;
}
