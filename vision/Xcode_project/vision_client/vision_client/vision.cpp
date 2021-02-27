//
//  vision.cpp
//  vision_client
//
//  Created by 童博涵 on 2021/2/27.
//

#include "vision.hpp"
Vision::Vision(int index){
	this->vision_mode = CAMERA_MODE;
	this->cap = VideoCapture(index);
	if(!readCameraParam("cameraConfigVideo.yaml")){
		cout << "read camera config file error" << endl;
	}
	else{
		cout << "read camera config file success" << endl;
	}
}
Vision::Vision(const string& filename){
	this->vision_mode = VIDEO_MODE;
	this->cap = VideoCapture(filename);
	if(!readCameraParam("cameraConfigVideo.yaml")){
		cout << "read camera config file error" << endl;
	}
	else{
		cout << "read camera config file success" << endl;
	}
}
Vision::~Vision(){
	this->cap.release();
}

void Vision::writeCameraParam(string filename){
	FileStorage fs(filename, FileStorage::WRITE);
	fs << "camera_matrix" << camMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs.release();
}

bool Vision::readCameraParam(string filename){
	FileStorage fs(filename, FileStorage::READ);
	if(!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}

void Vision::readArucoMap(string filename, vector<int>& ids, vector<vector<Point3f>>& pts){
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

void Vision::detectLight(Mat& image, int markerId, vector<Point2f> markerCorner, Mat& light_roi){
	Point2f right_corner, left_corner, mid;
	if (markerId == 0) {
		left_corner = markerCorner[0];
	}
	else if (markerId == 1) {
		right_corner = markerCorner[1];
	}
	mid = (left_corner + right_corner) / 2;
	//circle(outputImage, mid, 28, Scalar(0,0,255,5));
	Point2f pt1, pt2;
	double markerLen = abs((left_corner - right_corner).x);
	//cout <<"markerLen: " << markerLen << endl;
	pt1.x = 1.07 * left_corner.x;
	pt1.y = left_corner.y - 1.15* markerLen;
	pt2.x = 0.96 * right_corner.x;
	pt2.y = 0.97 * right_corner.y;
					
//	cout << pt1 << pt2 << endl;
	if(pt1.x<=0 || pt1.y<=0 || pt2.x<=0 || pt2.y<=0)
		return;
	Rect rect(pt1.x, pt1.y, pt2.x-pt1.x, pt2.y-pt1.y);
	light_roi = image(rect);
	rectangle(image, pt1, pt2, Scalar(255,255,0), 4);
}

void Vision::estimatePose_1(Mat& image, double length, Mat& light_roi){
	// 使用aruco内置函数估计相机位姿
	
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
	vector<Vec3d> rvecs, tvecs;
	Mat tvecs_m(3, 1, CV_64FC1);
	Mat rotM(3,3,CV_64FC1);// 转换为矩阵
	Mat rotT(3,3,CV_64FC1);
	aruco::detectMarkers(image, aruco_dict, markerCorners, markerIds);
	
	if (markerIds.size() == 0) {
		cout << "no aruco marker detected" << endl;
	} else {
		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
		aruco::estimatePoseSingleMarkers(markerCorners, length, camMatrix, distCoeffs, rvecs, tvecs);
		for (int i = 0; i < markerIds.size(); i++) {
			aruco::drawAxis(image, camMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
			tvecs_m.at<double>(0,0) = tvecs[i][0];
			tvecs_m.at<double>(1,0) = tvecs[i][1];
			tvecs_m.at<double>(2,0) = tvecs[i][2];
			Rodrigues(rvecs[i], rotM);
			Rodrigues(tvecs[i], rotT);
			
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
			detectLight(image, markerIds[i], markerCorners[i], light_roi);
		}
	}
}

void Vision::estimatePose_2(Mat& image, double length, Mat& light_roi){
	// 使用solvePnP估计相机位姿，需提前测量二维码长度，单位为米
	
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
	
	aruco::detectMarkers(image, aruco_dict, markerCorners, markerIds);
	if (markerIds.size() == 0) {
		cout << "no aruco marker detected" << endl;
	} else {
		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
		for (int i = 0; i < markerIds.size(); i++) {
			markerCorners2d = markerCorners[i];
			solvePnP(markerCorners3d, markerCorners2d, camMatrix, distCoeffs, raux, taux);
			aruco::drawAxis(image, camMatrix, distCoeffs, rvecs, tvecs, 0.1);
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
			detectLight(image, markerIds[i], markerCorners[i], light_roi);
		}
	}
}

void Vision::estimatePose_3(Mat& image, Mat& light_roi){
	// 多个aruco，给定map
	
	// 读取aruco地图
	vector<int> totalIds;
	vector<vector<Point3f>> totalPts;
	readArucoMap("aruco_video.txt", totalIds, totalPts); // aruco_video为视频中的map   aruco_test为打印的map
	if (totalIds.size() != totalPts.size()) {
		cout << "aruco file error" << endl;
		return;
	}
//	else {
//		cout << "read aruco file success" << endl;
//	}
	
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
	Point2f right_corner, left_corner, mid;
	
	aruco::detectMarkers(image, aruco_dict, markerCorners, markerIds);
	if (markerIds.size() == 0) {
		cout << "no aruco marker detected" << endl;
	}
	else {
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
			solvePnP(markerCorners3d, markerCorners2d, camMatrix, distCoeffs, raux, taux);

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
			markerCorners3d.clear();
			
			detectLight(image, markerIds[i], markerCorners[i], light_roi);
		}
	}
	
}
