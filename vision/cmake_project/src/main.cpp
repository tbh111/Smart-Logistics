#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>
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

void lightJudge(Mat& image, Mat& thr, Mat& light){

	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	threshold(gray, gray, 200, 255, THRESH_TOZERO);//阈值取出较亮部分
	Mat kernel = getStructuringElement(MORPH_CROSS, Size(3,3));
	morphologyEx(gray, thr, MORPH_OPEN, kernel);//开操作，减小干扰
	light = Mat::zeros(thr.rows, thr.cols, CV_8UC3);
	//查找所有轮廓
	vector<vector<Point>> contours;
	vector<Vec4i> hierarcy;
	findContours(thr, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);//检测最外围轮廓，保存所有轮廓点
	//查找最大面积
	int largest_area = 0;
	int largest_contour_index = 0;
	vector<Point> max_contour;
	double H, S, V;
	
	if(contours.size() > 0){
		for(int i = 0; i < contours.size(); i++){
			double area = contourArea(contours[i], false);//计算面积
			if(area > largest_area){
				largest_area = area;
				largest_contour_index = i;
				max_contour = contours[largest_contour_index];
			}
		}
		cout << largest_area << endl;
		drawContours(light, contours, -1, Scalar(0,0,255));
		
		if(largest_area > 10){
			drawContours(light, contours, largest_contour_index, Scalar(255,255,0));
			Mat temp = image.clone();
			cvtColor(temp, temp, COLOR_BGR2HSV);
			for(int a = 0; a < temp.rows; a++){
				for(int b = 0; b < temp.cols; b++){
					Point2f pt(a, b);
					if(pointPolygonTest(max_contour, pt, true)){
						H = temp.at<Vec3b>(a,b)[0];
						S = temp.at<Vec3b>(a,b)[1];
						V = temp.at<Vec3b>(a,b)[2];
						//printf("%d %d %d\n", temp.at<Vec3b>(a,b)[0],temp.at<Vec3b>(a,b)[1],temp.at<Vec3b>(a,b)[2]);
						if((S > 43 && S < 255) || (V > 46 && V < 255)){
							if(H > 35 && H < 77){
								cout << "green" << endl;	
							}
							else if(H > 26 && H < 34){
								cout << "yellow" << endl;
							}
							else if((H > 0 && H < 10) || (H > 156 && H < 180)){
								cout << "red" << endl;
							}
						}
					}
				}
			}
			
			
		}
	
	}
	
}

int main(int argc, char** argv){
	// 写入相机内参
	//Mat cam = (Mat_<double>(3,3) << 228.82, 0, 601.32, 0, 228.59, 363.39, 0, 0, 1);
	//Mat dist = (Mat_<double>(4,1) << 0.00490, 0.00105, 0.00012, 0.00047);
	//writeCameraParam("/home/tbh/Desktop/cv/src/cameraConfig.yaml", cam, dist);
	
	// 读取相机内参
	Mat cam, dist;
	if(!readCameraParam("/home/tbh/Desktop/cv/src/cameraConfig.yaml", cam, dist)){
		cout << "read camera config file error" << endl;
		return 1;
	}
	else
		cout << "read camera config file success" << endl;
	// 打开距离记录文件
	ofstream f_dis;
	f_dis.open("/home/tbh/Desktop/cv/bin/distance_log.txt", ios::out|ios::trunc);
	if(!f_dis.is_open()){
		cout << "can't open distance record file" << endl;
		return 1;
	}
	// 打开视频文件
	VideoCapture cap("/home/tbh/Desktop/cv/bin/line20191219_2.avi"); // 检查视频是否成功打开
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
	bool stop(false);
	Mat frame; // 当前视频帧
	Mat outputImage;
	Mat trafficROI;
	Mat imgSlice;
	Mat findLight;
	
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners;
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
	namedWindow("Traffic light ROI");
	namedWindow("Detected aruco");
	namedWindow("sliced image");
	namedWindow("find light");
	
	Point2f right_corner, left_corner, mid;
	
	// 根据帧速率计算帧之间的等待时间，单位为ms 
	int delay = 1000/rate;
	// 循环遍历视频中的全部帧 
	while (!stop) {
	// 读取下一帧(如果有)
		if (!cap.read(frame))
			break;
		frame.copyTo(outputImage);
		aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
		if(markerIds.size() == 0){
			cout<<"no aruco detected"<<endl;
		}
		else{
			aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
			vector<Vec3d> rvecs, tvecs;// 输出的旋转/平移向量,使坐标点从世界坐标系旋转/平移到相机坐标系
			Mat rvecs_m(3,1,CV_64FC1), tvecs_m(3,1,CV_64FC1);
			Mat rotM(3,3,CV_64FC1);// 转换为矩阵
			Mat rotT(3,3,CV_64FC1);
        	aruco::estimatePoseSingleMarkers(markerCorners, 0.54, cam, dist, rvecs, tvecs);//第二个参数为marker的尺寸，默认为米

			for(int i = 0; i < markerIds.size(); i++){
				//cout<<markerIds[i]<<endl;
				if(markerIds[i] < 10){
					aruco::drawAxis(outputImage, cam, dist, rvecs[i], tvecs[i], 0.1);
					rvecs_m.at<double>(0,0) = rvecs[i][0];
					rvecs_m.at<double>(1,0) = rvecs[i][1];
        			rvecs_m.at<double>(2,0) = rvecs[i][2];
        			tvecs_m.at<double>(0,0) = tvecs[i][0];
        			tvecs_m.at<double>(1,0) = tvecs[i][1];
        			tvecs_m.at<double>(2,0) = tvecs[i][2];
        			Rodrigues(rvecs_m, rotM);
        			Rodrigues(tvecs_m, rotT);	
        			
        			//根据旋转矩阵求出坐标旋转角 参考:https://www.jianshu.com/p/b97406d8833c
					double theta_x = atan2(rotM.at<double>(2, 1), rotM.at<double>(2, 2));
    				double theta_y = atan2(-rotM.at<double>(2, 0),
    				sqrt(rotM.at<double>(2, 1)*rotM.at<double>(2, 1) + rotM.at<double>(2, 2)*rotM.at<double>(2, 2)));
					double theta_z = atan2(rotM.at<double>(1, 0), rotM.at<double>(0, 0));

					//将弧度转化为角度
					theta_x = abs(theta_x * (180 / M_PI));
					theta_y = abs(theta_y * (180 / M_PI));
					theta_z = abs(theta_z * (180 / M_PI));
					//cout << "theta_x: " << theta_x << " theta_y: " << theta_y << " theta_z: " << theta_z << endl;
					putText(outputImage, "angle: " + to_string(theta_x) + " " + to_string(theta_y) + " " + to_string(theta_z), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0),2);
					//将相机坐标系转换到世界坐标系
					Mat camPos = rotM.inv() * (-rvecs_m);
					//cout << "camPos" << camPos << endl;
					f_dis << camPos.at<double>(0) << endl;
					string str1 = to_string(abs(camPos.at<double>(0)));
					string str2 = to_string(abs(camPos.at<double>(1)));
					string str3 = to_string(abs(camPos.at<double>(2)));
					putText(outputImage, "pos: " + str1 + " " + str2 + " " + str3, Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0),2);
				}
				if(markerIds[i] == 0){
					left_corner = markerCorners[i][0];
				}
				else if(markerIds[i] == 1){
					right_corner = markerCorners[i][1];
				}
				mid = (left_corner + right_corner) / 2;
				//circle(outputImage, mid, 28, Scalar(0,0,255,5));
				Point2f pt1, pt2;
				double markerLen = abs((left_corner - right_corner).x);
				//cout <<"markerLen: " << markerLen << endl;
				
				pt1.x = 1.035*left_corner.x;
				pt1.y = left_corner.y - 1.2 *  markerLen;
				pt2.x = right_corner.x;
				pt2.y = 0.975 * right_corner.y;
				
				//cout << pt1 << pt2 << endl;
				rectangle(outputImage, pt1, pt2, Scalar(255,255,0), 4);
				if(pt1.x<=0 || pt1.y<=0 || pt2.x<=0 || pt2.y<=0)
					continue;
				Rect rect(pt1.x, pt1.y, pt2.x-pt1.x, pt2.y-pt1.y);
				//cout << rect.size() << rect.tl() << rect.br() << rect.width << endl;
				trafficROI = frame(rect);
			}
		}
		lightJudge(trafficROI, imgSlice, findLight);
		//if(trafficROI.rows!=0)
		imshow("Traffic light ROI", trafficROI);
		imshow("Detected aruco", outputImage);
		imshow("sliced image", imgSlice);
		imshow("find light", findLight);
		// 等待一段时间，或者通过按键停止 
		if (waitKey(delay) >= 0)
			stop = true;
	}
	// 关闭视频文件
	// 不是必须的，因为类的析构函数会调用 
	cap.release();
	f_dis.close();
	cout << "end process" << endl;
	return 0;
}
