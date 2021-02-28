//
//  main.cpp
//  vision_client
//
//  Created by 童博涵 on 2021/2/18.
//
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
#include "vision.hpp"

using namespace std;
using namespace cv;

int main(int argc, char** argv){
	Vision vision("/Users/tongbohan/Desktop/line20191219_2.avi");
	if(!vision.cap.isOpened()){
		cout << "read video error" << endl;
		return 1;
	}
	// 取得帧速率
	double rate = vision.cap.get(CAP_PROP_FPS);
	// 根据帧速率计算帧之间的等待时间，单位为ms
	int delay = 1000/rate;
	// 跳转到第1000帧开始读取视频
	//double position = 1000.0;
	double position = 1.0;
	vision.cap.set(CAP_PROP_POS_FRAMES, position);

	Mat frame;
	Mat light;
	Mat light_1;
	namedWindow("camera");
	namedWindow("light");
	namedWindow("light_1");
	while(true){
		if(!vision.cap.read(frame)){
			cout << "video ends" << endl;
			break;
		}
		vision.estimatePose_3(frame, light);
		vision.lightJudge(light, light_1);
		imshow("camera", frame);
		imshow("light", light);
		imshow("light_1", light_1);
		
		if (waitKey(delay) == 'q') {
			break;
		}
	}
	
	cout << "hello world" << endl;
	return 0;
}

//void writeCameraParam(string filename, Mat& camMatrix, Mat& distCoeffs){
//	FileStorage fs(filename, FileStorage::WRITE);
//	fs << "camera_matrix" << camMatrix;
//	fs << "distortion_coefficients" << distCoeffs;
//	fs.release();
//}
//
//bool readCameraParam(string filename, Mat& camMatrix, Mat& distCoeffs){
//	FileStorage fs(filename, FileStorage::READ);
//	if(!fs.isOpened())
//		return false;
//	fs["camera_matrix"] >> camMatrix;
//	fs["distortion_coefficients"] >> distCoeffs;
//	return true;
//}
//
//void readArucoMap(string filename, vector<int>& ids, vector<vector<Point3f>>& pts){
//	ifstream file(filename, ifstream::in);
//	if (!file) {
//		cout << "open file failed" << endl;
//		return;
//	}
//	string line, id, loc_x, loc_y;
//	vector<Point3f> pt_v;
//	Point3f pt;
//
//	while (getline(file, line)) {
//		istringstream sin(line);
//		vector<string> datas;
//		string data;
//		while (getline(sin, data, ';')) {
//			datas.push_back(data);
//		}
//		id = datas[0];
//		for(int i = 1; i < 6; i++) {
//			string str = datas[i];
//			auto pos = str.find(",");
//			if (pos != string::npos) {
////				cout << str.substr(0, pos) << endl;
////				cout << str.substr(pos+1, str.length()) << endl;
//				loc_x = str.substr(0, pos);
//				loc_y = str.substr(pos+1, str.length());
//				pt.x = (atof(loc_x.c_str()));
//				pt.y = (atof(loc_y.c_str()));
//				pt.z = 0.0;
//				pt_v.push_back(pt);
//			}
//
//		}
//		ids.push_back(atoi(id.c_str()));
//		pts.push_back(pt_v);
//		pt_v.clear();
//	}
////	for (vector<int>::iterator it = ids.begin(); it != ids.end(); it++) {
////		cout << *it << endl;
////	}
//}
//
//void estimatePose_1(Mat& image, double length, Mat camMat, Mat dist, Ptr<aruco::Dictionary> dictionary){
//	// 使用aruco内置函数估计相机位姿
//
//	vector<int> markerIds;
//	vector<vector<Point2f>> markerCorners;
//	vector<Vec3d> rvecs, tvecs;
//	Mat tvecs_m(3, 1, CV_64FC1);
//	Mat rotM(3,3,CV_64FC1);// 转换为矩阵
//	Mat rotT(3,3,CV_64FC1);
//	aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
//
//	if (markerIds.size() == 0) {
//		cout << "no aruco marker detected" << endl;
//	} else {
//		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
//		aruco::estimatePoseSingleMarkers(markerCorners, length, camMat, dist, rvecs, tvecs);
//		for (int i = 0; i < markerIds.size(); i++) {
//			aruco::drawAxis(image, camMat, dist, rvecs[i], tvecs[i], 0.1);
//			tvecs_m.at<double>(0,0) = tvecs[i][0];
//			tvecs_m.at<double>(1,0) = tvecs[i][1];
//			tvecs_m.at<double>(2,0) = tvecs[i][2];
//			Rodrigues(rvecs[i], rotM);
//			Rodrigues(tvecs[i], rotT);
//
//			// 计算旋转角度
//			double theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
//			double theta_y = atan2(-rotM.at<double>(2, 0),
//			sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
//			double theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));
//			theta_x = theta_x * (180 / M_PI);
//			theta_y = theta_y * (180 / M_PI);
//			theta_z = theta_z * (180 / M_PI);
//			putText(image, "angle: " + to_string(theta_x) + " " + to_string(theta_y) + " " + to_string(theta_z), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,0,0),1);
//
//			// 计算坐标 参考:https://blog.csdn.net/cocoaqin/article/details/77848588
//			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
//			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
//			cv2eigen(rotM, R_n);
//			cv2eigen(tvecs_m, T_n);
//			Eigen::Vector3f P_oc;
//			P_oc = -R_n.inverse() * T_n;
//			string str1 = to_string(P_oc(0));
//			string str2 = to_string(P_oc(1));
//			string str3 = to_string(P_oc(2));
//			putText(image, "pos: " + str1 + " " + str2 + " " + str3, Point(10, 45), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
//		}
//	}
//}
//
//void estimatePose_2(Mat& image, double length, Mat camMat, Mat dist, Ptr<aruco::Dictionary> dictionary, Mat& light_roi){
//	// 使用solvePnP估计相机位姿，需提前测量二维码长度，单位为米
//
//	vector<Point3f> markerCorners3d;
//	vector<Point2f> markerCorners2d;
//	vector<int> markerIds;
//	vector<vector<Point2f>> markerCorners;
//	Mat rvecs;
//	Mat_<float> tvecs;
//	Mat raux, taux;
//	Mat_<float> rotM(3, 3);
//	markerCorners3d.push_back(Point3f(-length/2,  length/2, 0));
//	markerCorners3d.push_back(Point3f( length/2,  length/2, 0));
//	markerCorners3d.push_back(Point3f( length/2, -length/2, 0));
//	markerCorners3d.push_back(Point3f(-length/2, -length/2, 0));
//
//	aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
//	if (markerIds.size() == 0) {
//		cout << "no aruco marker detected" << endl;
//	} else {
//		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
//		for (int i = 0; i < markerIds.size(); i++) {
//			markerCorners2d = markerCorners[i];
//			solvePnP(markerCorners3d, markerCorners2d, camMat, dist, raux, taux);
//			aruco::drawAxis(image, camMat, dist, rvecs, tvecs, 0.1);
//			raux.convertTo(rvecs, CV_32F);
//			taux.convertTo(tvecs, CV_32F);
//			Rodrigues(rvecs, rotM);
//
//			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
//			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
//			cv2eigen(rotM, R_n);
//			cv2eigen(tvecs, T_n);
//			Eigen::Vector3f P_oc;
//			P_oc = -R_n.inverse()*T_n;
//			string str1 = to_string(P_oc(0));
//			string str2 = to_string(P_oc(1));
//			string str3 = to_string(P_oc(2));
//			putText(image, "pos: " + str1 + " " + str2 + " " + str3, Point(10, 45), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
//		}
//	}
//}
//
//void estimatePose_3(Mat& image, Mat camMat, Mat dist, Ptr<aruco::Dictionary> dictionary, Mat& light_roi){
//	// 多个aruco，给定map
//
//	// 读取aruco地图
//	vector<int> totalIds;
//	vector<vector<Point3f>> totalPts;
//	readArucoMap("aruco_video.txt", totalIds, totalPts); // aruco_video为视频中的map   aruco_test为打印的map
//	if (totalIds.size() != totalPts.size()) {
//		cout << "aruco file error" << endl;
//		return;
//	}
////	else {
////		cout << "read aruco file success" << endl;
////	}
//
////	for (int i = 0; i < totalIds.size(); i++) {
////		cout << totalIds[i] << endl;
////		for (int j = 0; j < totalPts[i].size(); j++) {
////			cout << totalPts[i][j];
////		}
////		cout << endl;
////	}
//
//	vector<Point3f> markerCorners3d;
//	vector<Point2f> markerCorners2d;
//	vector<int> markerIds;
//	vector<vector<Point2f>> markerCorners;
//	Mat rvecs;
//	Mat_<float> tvecs;
//	Mat raux, taux;
//	Mat_<float> rotM(3, 3);
//	int marker_id_current;
//	int marker_index = -1;
//	double length = 0.0;
//	Point2f right_corner, left_corner, mid;
//
//	aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
//	if (markerIds.size() == 0) {
//		cout << "no aruco marker detected" << endl;
//	}
//	else {
//		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
//		for (int i = 0; i < markerIds.size(); i++) {
//			marker_id_current = markerIds[i];
//			markerCorners2d = markerCorners[i];
//			for (int j = 0; j < totalIds.size(); j++) {
//				if (totalIds[j] == marker_id_current) {
//					marker_index = j;
//				}
//			}
//			if (marker_index < 0) {
//				cout << "detect error" << endl;
//				continue;
//			}
//			else {
//				length = abs((totalPts[marker_index][0].x-totalPts[marker_index][1].x)*2)*0.01;
//				for (int k = 1; k < 5; k++) {
//					markerCorners3d.push_back(totalPts[marker_index][k]);
//				}
//			}
//			solvePnP(markerCorners3d, markerCorners2d, camMat, dist, raux, taux);
//
//			//aruco::drawAxis(image, camMat, dist, rvecs, tvecs, 0.1);
//			raux.convertTo(rvecs, CV_32F);
//			taux.convertTo(tvecs, CV_32F);
//			Rodrigues(rvecs, rotM);
//			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
//			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;
//			cv2eigen(rotM, R_n);
//			cv2eigen(tvecs, T_n);
//			Eigen::Vector3f P_oc;
//			P_oc = -R_n.inverse()*T_n;
//			string str1 = to_string(P_oc(0)-totalPts[marker_index][0].x);
//			string str2 = to_string(P_oc(1)-totalPts[marker_index][0].y);
//			string str3 = to_string(P_oc(2)-totalPts[marker_index][0].z);
//			putText(image, "pos_x: " + str1, Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
//			putText(image, "pos_y: " + str2, Point(10, 45), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
//			putText(image, "pos_z: " + str3, Point(10, 65), FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,255,0),1);
//			markerCorners3d.clear();
//
//			if (markerIds[i] == 0) {
//				left_corner = markerCorners[i][0];
//			}
//			else if (markerIds[i] == 1) {
//				right_corner = markerCorners[i][1];
//			}
//			mid = (left_corner + right_corner) / 2;
//			//circle(outputImage, mid, 28, Scalar(0,0,255,5));
//			Point2f pt1, pt2;
//			double markerLen = abs((left_corner - right_corner).x);
//			//cout <<"markerLen: " << markerLen << endl;
//
//			pt1.x = 1.07 * left_corner.x;
//			pt1.y = left_corner.y - 1.15* markerLen;
//			pt2.x = 0.96 * right_corner.x;
//			pt2.y = 0.97 * right_corner.y;
//
//			//cout << pt1 << pt2 << endl;
//
//			if(pt1.x<=0 || pt1.y<=0 || pt2.x<=0 || pt2.y<=0)
//				continue;
//			Rect rect(pt1.x, pt1.y, pt2.x-pt1.x, pt2.y-pt1.y);
//			light_roi = image(rect);
//			rectangle(image, pt1, pt2, Scalar(255,255,0), 4);
//		}
//	}
//}
//
//void lightJudge(Mat& image, Mat& light) {
//	// 检测信号灯 目前存在少量误检测状况
//	Mat gray, thr;
//	cvtColor(image, gray, COLOR_BGR2GRAY);
//	threshold(gray, gray, 225, 255, THRESH_BINARY);//阈值取出较亮部分
//	Mat kernel = getStructuringElement(MORPH_CROSS, Size(3,3));
//	morphologyEx(gray, thr, MORPH_OPEN, kernel);//开操作，减小干扰
//	light = Mat::zeros(thr.rows, thr.cols, CV_8UC3);
//	//查找所有轮廓
//	vector<vector<Point>> contours;
//	vector<Vec4i> hierarcy;
//	findContours(thr, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);//检测最外围轮廓，保存所有轮廓点
//	//查找最大面积
//	int largest_area = 0;
//	int largest_contour_index = 0;
//	vector<Point> max_contour;
//	double H, S, V;
//
//	if(contours.size() > 0){
//		for(int i = 0; i < contours.size(); i++){
//			double area = contourArea(contours[i], false);//计算面积
//			if(area > largest_area){
//				largest_area = area;
//				largest_contour_index = i;
//				max_contour = contours[largest_contour_index];
//			}
//		}
////		cout << largest_area << endl;
//		drawContours(light, contours, -1, Scalar(0,0,255));
//
//		if(largest_area > 10){
//			int count_g = 0, count_y = 0, count_r = 0;
//			drawContours(light, contours, largest_contour_index, Scalar(255,255,0));
//			Mat temp = image.clone();
//			cvtColor(temp, temp, COLOR_BGR2HSV);
//			for(int a = 0; a < temp.rows; a++){
//				for(int b = 0; b < temp.cols; b++){
//					Point2f pt(a, b);
//					if(pointPolygonTest(max_contour, pt, true)){
//						H = temp.at<Vec3b>(a,b)[0];
//						S = temp.at<Vec3b>(a,b)[1];
//						V = temp.at<Vec3b>(a,b)[2];
////						printf("%d %d %d\n", temp.at<Vec3b>(a,b)[0],temp.at<Vec3b>(a,b)[1],temp.at<Vec3b>(a,b)[2]);
//						if((S > 43 && S < 255) || (V > 46 && V < 255)){
//							if(H > 35 && H < 77){
////								cout << "green" << endl;
//								count_g++;
//							}
//							else if(H > 26 && H < 34){
////								cout << "yellow" << endl;
//								count_y++;
//							}
//							else if((H > 0 && H < 10) || (H > 156 && H < 180)){
////								cout << "red" << endl;
//								count_r++;
//							}
//						}
//					}
//				}
//			}
//			if (count_g > count_r && count_g > count_y) {
//				cout << "green" << endl;
//			}
//			else if (count_y > count_g && count_y > count_r) {
//				cout << "yellow" << endl;
//			}
//			else {
//				cout << "red" << endl;
//			}
//		}
//	}
//}
//
//int main(int argc, char** argv)
//{
//	// 写入相机内参
////	Mat cam = (Mat_<double>(3,3) << 228.82, 0, 601.32, 0, 228.59, 363.39, 0, 0, 1);
////	Mat dist = (Mat_<double>(4,1) << 0.00490, 0.00105, 0.00012, 0.00047);
////	writeCameraParam("cameraConfigVideo.yaml", cam, dist);
//
//	// 读取相机内参
//	Mat cam, dist;
//	if(!readCameraParam("cameraConfigVideo.yaml", cam, dist)){
//		cout << "read camera config file error" << endl;
//		return 1;
//	}
//	else
//		cout << "read camera config file success" << endl;
//
//	int mode = MODE_VIDEO;
//	Ptr<aruco::Dictionary> aruco_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
//
//	if (mode == MODE_CAMERA) {
//		VideoCapture cap(0);
//		Mat frame;
//		Mat light;
//		namedWindow("camera");
//		namedWindow("light");
//
//	//	double markerLength = 0.18;
//		double rate = 0.0;
//
//		while (true) {
//			if (!cap.isOpened()) {
//				cout << "cannot open camera" << endl;
//				break;
//			}
//			cap.read(frame);
//			rate = cap.get(CAP_PROP_FPS);
//			putText(frame, "fps: " + to_string(rate), Point(10,15), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255,0,0));
//			estimatePose_3(frame, cam, dist, aruco_dict, light);
//			imshow("camera", frame);
//			imshow("light", light);
//			if (waitKey(1) == 'q') {
//				break;
//			}
//		}
//		cap.release();
//	}
//	else if (mode == MODE_VIDEO) {
//		VideoCapture cap("/Users/tongbohan/Desktop/line20191219_2.avi"); // 检查视频是否成功打开
//		if (!cap.isOpened()){
//			cout<<"read video error"<<endl;
//			return 1;
//		}
//		// 取得帧速率
//		double rate = cap.get(CAP_PROP_FPS);
//		// 跳转到第1000帧开始读取视频
//		//double position = 1000.0;
//		double position = 1.0;
//		cap.set(CAP_PROP_POS_FRAMES, position);
//
//		Mat frame;
//		Mat light;
//		Mat light_thr;
//
//		// 根据帧速率计算帧之间的等待时间，单位为ms
//		int delay = 1000/rate;
//		// 循环遍历视频中的全部帧
//		while (true) {
//		// 读取下一帧(如果有)
//			if (!cap.read(frame)){
//				cout << "video ends" << endl;
//				break;
//			}
////			estimatePose_1(frame, 0.54, cam, dist, aruco_dict);
//			estimatePose_3(frame, cam, dist, aruco_dict, light);
//			lightJudge(light, light_thr);
//			imshow("camera", frame);
//			imshow("light", light);
//			imshow("light_thr", light_thr);
//			if (waitKey(delay) == 'q') {
//				break;
//			}
//		}
//		cap.release();
//	}
//	else {
//
//	}
//
//
//    return 0;
//}
