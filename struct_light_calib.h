//
// Created by 赵丛杨 on 2018/12/23.
//

#ifndef TTTTT_STRUCT_LIGHT_CALIB_H
#define TTTTT_STRUCT_LIGHT_CALIB_H


#include <vector>
#include <string>
#include <iostream>
#include "structlight.h"

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

void calc3DCoordination(structlight a, vector<Point2f> &centerPoint, vector<Point3f> &calcPoint);


void EstimateError(structure_light StructureLightCalib, structlight StructlightMeasure, vector<vector<float>> BackProjectError, int imageCount);
void EstimateError2(structure_light StructureLightCalib, structlight StructlightMeasure, vector<vector<float>> BackProjectError, int imageCount);

double get_norm(double *x, int n);
double normalize(double *x, int n);
inline double product(double*a, double *b, int n);
void orth(double *a, double *b, int n);
bool svd(vector<vector<double> > A, int K, vector<vector<double> > &U, vector<double> &S, vector<vector<double> > &V);

#endif //TTTTT_STRUCT_LIGHT_CALIB_H
