
#ifndef STRUCTURE_LIGHT_H
#define STRUCTURE_LIGHT_H

#include "struct_light_calib.h"


class structure_light 
{
public:
	structure_light(int x, cv::Size patternSize, cv::Size patternLength);
	
	double planeFormular[4] = { 0 };
	
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	
	std::vector<cv::Mat> R;
	std::vector<cv::Mat> T;
	
	cv::Mat Rw;
	cv::Mat Tw;

	std::vector<bool> isRecognize;

	std::vector<std::vector<cv::Point3f>> calibBoardPoint;
	std::vector<cv::Point3f> lightPlanePoint;
	
	std::vector<std::vector<cv::Point2f>> calibImagePoint;

	std::vector<std::vector<cv::Point2f>> lightPlaneOriImagePoint;//像素级的中心点
	std::vector<std::vector<cv::Point2f>> lightPlaneSubpixelImagePoint;//亚像素级的中心点
	int imageCount;
	cv::Size patternSize;
	cv::Size PatternLength;

	void generateCalibboardPoint();
	void outputResult();
};

#endif