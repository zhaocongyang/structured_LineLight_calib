// use for measurement
#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

class structlight
{
public:
	structlight();
	~structlight();
	cv::Mat cameraIntrinsic;//相机的内参矩阵
	cv::Mat distCoeffs;//5个畸变系数
	double lightPlaneFormular[4];//光平面参数 Ax + By + Cz = D

	cv::Mat Rw;//旋转向量
	cv::Mat Tw;//位移向量

	void readParameters();
};

