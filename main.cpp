/*
 * 使用圆形靶标进行标定
 * 2019.1 29修订；修改人：zcy ；功能：实现线结构光+单目系统的标定
 * 2019.4.26修订；修改人：zcy ；增加功能：增加线结构光测量精度评估（第一版评估）计算标定7点到重构点云的最小距离
 * 2019.4.27修订；修改人：zcy ；增加功能：增加线结构光测量精度评估（第二版评估）计算标定7点到重构点云拟合成的直线的距离
 */
#include "structure_light.h"
#include "struct_light_calib.h"
#include "structlight.h"
#include <fstream>
#include <math.h>
#include <stdlib.h>  
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <vector>

using namespace cv;
using namespace std;

int MAX_ITER = 100000;
double eps = 0.0000001;

//定义一些全局变量作为标定的靶标参数
int imageCount = 11;//图像的数量
Size patternSize(7, 7);//靶标（圆的数量）
Size2f patternLength(37.5, 37.5);//两个圆形标记之间的距离
Size image_size(1920, 1080);
//patternType:
//0:circle;
//1:chessboard;
bool iscircle = true;



int main()
{
	structure_light lineStructureLight(imageCount, patternSize, patternLength);
	
	double reprojectionError = 0.0;
	cameraCalib(lineStructureLight, reprojectionError);//摄像机标定
	cout << "Camera calibration reprojection error: " << reprojectionError << " pixels." << endl;

	//光条中心提取
	//steger光条中心提取算法
	stegerLine(lineStructureLight);

	//结构光光条直线拟合与相交点坐标提取
	vector<vector<Point2f>> intersectPoint;//光平面和靶标平面上的点
	crossPoint(lineStructureLight, intersectPoint);
	//交比不变求取光平面三维坐标
	crossRatio(lineStructureLight, intersectPoint);

	//拟合光平面
	lightPlainFitting(lineStructureLight);

	// 输出点云的数量
	cout << lineStructureLight.lightPlanePoint.size() << endl;

	// 将点云存储在一个TXT文件中
	ofstream outfile;
	outfile.open("pointcloud.txt", ios::binary | ios::app | ios::in | ios::out);
	for (int k = 0; k < lineStructureLight.lightPlanePoint.size(); k++)
	{
		outfile << lineStructureLight.lightPlanePoint[k].x << " ";
		outfile << lineStructureLight.lightPlanePoint[k].y << " ";
		outfile << lineStructureLight.lightPlanePoint[k].z << "\n";
	}
	outfile.close();//关闭文件，保存文件
	//输出标定结果
	lineStructureLight.outputResult();

	//对标定结果进行评估（方法：类反投影）
	structlight StructlightMeasure;
	StructlightMeasure.readParameters();
	vector<vector<float>> BackProjectError;
	EstimateError2(lineStructureLight, StructlightMeasure, BackProjectError, imageCount);
	cout << "finsih!" << endl;
	return 0;

}
