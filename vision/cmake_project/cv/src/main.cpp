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
	Vision vision("1.mp4");
//	Vision vision(0);
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

	do{
		vision.cap >> frame;
		if(!vision.cap.read(frame)){
			cout << "video ends" << endl;
			break;
		}
		vision.estimatePose_3(frame, light);
		int result = vision.lightJudge(light, light_1);
		if(result == 2){
			putText(frame, "green", Point(10,85),FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,0,0),1);
		}
		else if(result == 1){
			putText(frame, "yellow", Point(10,85),FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,0,0),1);
		}
		else if(result == 0){
			putText(frame, "red", Point(10,85),FONT_HERSHEY_SIMPLEX, 0.45, Scalar(255,0,0),1);
		}
		imshow("camera", frame);
		//imwrite("im.jpg", frame);
		if (!light.empty()) {
			imshow("light", light);
			imshow("light_1", light_1);
		}


		if (waitKey(delay) == 'q') {
			break;
		}
	}while (!frame.empty());

	return 0;
}

