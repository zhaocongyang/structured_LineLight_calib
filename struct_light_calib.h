//
// Created by 赵丛杨 on 2018/12/23.
//

#ifndef TTTTT_STRUCT_LIGHT_CALIB_H
#define TTTTT_STRUCT_LIGHT_CALIB_H


#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <iostream>
#include "opencv2/imgcodecs.hpp"
//#include "structure_light.h"

using namespace std;
using namespace cv;

class structure_light;

int cameraCalib(structure_light &a, double &reprojectionError);

void stegerLine(structure_light &a);
//void greyCenter(Mat &image, structure_light &a);

void crossPoint(structure_light &a, std::vector<std::vector<cv::Point2f>> &crossPoint);

void crossRatio(structure_light &light, std::vector<std::vector<cv::Point2f>> &crossPoint);

void lightPlainFitting(structure_light &light);

void extratLightLine(int imageIndex, cv::Mat &outputImage, structure_light &a);


#endif //TTTTT_STRUCT_LIGHT_CALIB_H
