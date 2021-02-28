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

