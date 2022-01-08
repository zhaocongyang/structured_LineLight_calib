//
// Created by 赵丛杨 on 2018/12/23.
//
#include "struct_light_calib.h"
#include "structure_light.h"
#include "structlight.h"
#include <fstream>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>
#include <math.h>

extern int MAX_ITER;
extern double eps ;
extern int imageCount ;
extern Size patternSize;
extern Size2f patternLength;
extern Size image_size;
//patternType:
//0:circle;
//1:chessboard;
extern bool iscircle;

//摄像机标定
int cameraCalib(structure_light &a, double &reprojectionError)
{
	string format = ".jpg";
	for (int i = 0; i < a.imageCount; i++)
	{
		string index = to_string(i);
		string name = "./calib_picture/calib" + index + format;
		Mat pic = imread(name);

		Mat greyImage;
		cvtColor(pic, greyImage, COLOR_BGR2GRAY);

		bool result = true;
		vector<Point2f> targetPoint;
		//圆形靶标提取圆心点
		if (iscircle)
		{
			if (0 == findCirclesGrid(greyImage, a.patternSize, targetPoint))//提取靶标上圆斑的圆心
			{
				result = false;
				a.isRecognize.push_back(result);
				cout << "false-calib" << endl;
				continue;
			}
			else
			{
				result = true;
				a.isRecognize.push_back(result);
				a.calibImagePoint.push_back(targetPoint);
				cout << "true-calib" << endl;
			}
		}
		//棋盘格靶标提取角点
		else
		{
			if (0 == findChessboardCorners(greyImage, a.patternSize, targetPoint))
			{
				result = false;
				a.isRecognize.push_back(result);
				continue;
			}
			else
			{
				result = true;
				a.isRecognize.push_back(result);
				find4QuadCornerSubpix(greyImage, targetPoint, Size(5, 5));
				a.calibImagePoint.push_back(targetPoint);
				drawChessboardCorners(greyImage, patternSize, targetPoint, true);
				imwrite("./save/Corner.jpg", greyImage);
			}
		}
	}

	a.generateCalibboardPoint();
	/* 摄像头标定得到相机内参和畸变矩阵以及外参（旋转矩阵R和平移矩阵T）*/
	double rms = calibrateCamera(a.calibBoardPoint, a.calibImagePoint, image_size, a.cameraMatrix, a.distCoeffs, a.R, a.T);
	cout << "Reprojection error:" << rms << endl;

	a.Rw = a.R[0];
	a.Tw = a.T[0];
	/* 重投影评估单目摄像头标定精度 */
	double err = 0.0;
	double mean_err = 0.0;
	double total_err = 0.0;
	int i;
	vector<Point2f> reprojectionPoint;

	for (i = 0; i < a.calibBoardPoint.size(); i++)
	{
		vector<Point3f> tempPointSet = a.calibBoardPoint[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, a.R[i], a.T[i], a.cameraMatrix, a.distCoeffs, reprojectionPoint);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = a.calibImagePoint[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, reprojectionPoint.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(reprojectionPoint[j].x, reprojectionPoint[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);

		mean_err = err / (a.patternSize.width*a.patternSize.height);
		total_err += mean_err;
	}
	reprojectionError = total_err / a.calibBoardPoint.size();
	return 0;
}
//################# RANSAC算法拟合直线相关函数 #########################//
//生成[0,1]之间符合均匀分布的数
double uniformRandom(void)
{
	return (double)rand() / (double)RAND_MAX;
}

//直线样本中两随机点位置不能太近
bool verifyComposition(const vector<Point2f> pts)
{
	cv::Point2d pt1 = pts[0];
	cv::Point2d pt2 = pts[1];
	if (abs(pt1.x - pt2.x) < 5 && abs(pt1.y - pt2.y) < 5)
		return false;

	return true;
}

//根据点集拟合直线ax+by+c=0，res为残差
void calcLinePara(vector<Point2f> pts, double &a, double &b, double &c, double &res)
{
	res = 0;
	Vec4f line;
	vector<Point2f> ptsF;
	for (unsigned int i = 0; i < pts.size(); i++)
		ptsF.push_back(pts[i]);

	fitLine(ptsF, line, DIST_L2, 0, 1e-2, 1e-2);

	a = line[1];
	b = -line[0];
	c = line[0] * line[3] - line[1] * line[2];

	for (unsigned int i = 0; i < pts.size(); i++)
	{
		double resid_ = fabs(pts[i].x * a + pts[i].y * b + c);
		res += resid_;
	}
	res /= pts.size();
}

//得到直线拟合样本，即在直线采样点集上随机选2个点
bool getSample(vector<int> set, vector<int> &sset)
{
	int i[2];
	if (set.size() > 2)
	{
		do
		{
			for (int n = 0; n < 2; n++)
				i[n] = int(uniformRandom() * (set.size() - 1));
		} while (!(i[1] != i[0]));
		for (int n = 0; n < 2; n++)
		{
			sset.push_back(i[n]);
		}
	}
	else
	{
		return false;
	}
	return true;
}


//RANSAC直线拟合
void fitLineRANSAC(vector<Point2f> ptSet, double &a, double &b, double &c, vector<bool> &inlierFlag)
{
	double residual_error = 2.99; //内点阈值

	bool stop_loop = false;
	int maximum = 0;  //最大内点数

	//最终内点标识及其残差
	inlierFlag = vector<bool>(ptSet.size(), false);
	vector<double> resids_(ptSet.size(), 3);
	int sample_count = 0;
	int N = 500;

	double res = 0;

	// RANSAC
	srand((unsigned int)time(NULL)); //设置随机数种子
	vector<int> ptsID;
	for (unsigned int i = 0; i < ptSet.size(); i++)
		ptsID.push_back(i);
	while (N > sample_count && !stop_loop)
	{
		vector<bool> inlierstemp;
		vector<double> residualstemp;
		vector<int> ptss;
		int inlier_count = 0;
		if (!getSample(ptsID, ptss))
		{
			stop_loop = true;
			continue;
		}

		vector<Point2f> pt_sam;
		pt_sam.push_back(ptSet[ptss[0]]);
		pt_sam.push_back(ptSet[ptss[1]]);

		if (!verifyComposition(pt_sam))
		{
			++sample_count;
			continue;
		}

		// 计算直线方程
		calcLinePara(pt_sam, a, b, c, res);
		//内点检验
		for (unsigned int i = 0; i < ptSet.size(); i++)
		{
			Point2f pt = ptSet[i];
			double resid_ = fabs(pt.x * a + pt.y * b + c);
			residualstemp.push_back(resid_);
			inlierstemp.push_back(false);
			if (resid_ < residual_error)
			{
				++inlier_count;
				inlierstemp[i] = true;
			}
		}
		// 找到最佳拟合直线
		if (inlier_count >= maximum)
		{
			maximum = inlier_count;
			resids_ = residualstemp;
			inlierFlag = inlierstemp;
		}
		// 更新RANSAC迭代次数，以及内点概率
		if (inlier_count == 0)
		{
			N = 500;
		}
		else
		{
			double epsilon = 1.0 - double(inlier_count) / (double)ptSet.size(); //野值点比例
			double p = 0.99; //所有样本中存在1个好样本的概率
			double s = 2.0;
			N = int(log(1.0 - p) / log(1.0 - pow((1.0 - epsilon), s)));
		}
		++sample_count;
	}

	//利用所有内点重新拟合直线
	vector<Point2f> pset;
	for (unsigned int i = 0; i < ptSet.size(); i++)
	{
		if (inlierFlag[i])
			pset.push_back(ptSet[i]);
	}

	calcLinePara(pset, a, b, c, res);
}
//##################### RANSAC算法拟合直线部分完成 ######################


//提取结构光光条中心点（steger算法）
void stegerLine(structure_light &a)
{
    for (int k = 0; k < a.imageCount; k++)
    {
        if (a.isRecognize[k] == false)
            continue;

        Mat lightLineImage;
        extratLightLine(k, lightLineImage, a);

        imwrite("./save/image.jpg", lightLineImage);
        //一阶偏导数
        Mat m1, m2;
        m1 = (Mat_<float>(1, 2) << 1, -1);
        m2 = (Mat_<float>(2, 1) << 1, -1);

        Mat dx, dy;
        filter2D(lightLineImage, dx, CV_32FC1, m1);
        filter2D(lightLineImage, dy, CV_32FC1, m2);

        //二阶偏导数
        Mat m3, m4, m5;
        m3 = (Mat_<float>(1, 3) << 1, -2, 1);     //二阶x偏导
        m4 = (Mat_<float>(3, 1) << 1, -2, 1);     //二阶y偏导
        m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1); //二阶xy偏导

        Mat dxx, dyy, dxy;
        filter2D(lightLineImage, dxx, CV_32FC1, m3);
        filter2D(lightLineImage, dyy, CV_32FC1, m4);
        filter2D(lightLineImage, dxy, CV_32FC1, m5);

        vector<Point2f> oriImagePoint;
        vector<Point2f> subPixelImagePoint;
        //hessian矩阵提取光条中心
        for (int i = 0; i < lightLineImage.cols; i++)
        {
            for (int j = 0; j < lightLineImage.rows; j++)
            {
                //通过灰度值确定光条像素

                if (lightLineImage.at<uchar>(j, i) !=0)
                {
                    Mat hessian(2, 2, CV_32FC1);
                    hessian.at<float>(0, 0) = dxx.at<float>(j, i);
                    hessian.at<float>(0, 1) = dxy.at<float>(j, i);
                    hessian.at<float>(1, 0) = dxy.at<float>(j, i);
                    hessian.at<float>(1, 1) = dyy.at<float>(j, i);

                    Mat eValue, eVectors;
                    eigen(hessian, eValue, eVectors);

                    double nx, ny;
                    double fmaxD = 0;
                    if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))
                    {
                        nx = eVectors.at<float>(0, 0);
                        ny = eVectors.at<float>(0, 1);
                        fmaxD = eValue.at<float>(0, 0);
                    }
                    else
                    {
                        nx = eVectors.at<float>(1, 0);
                        ny = eVectors.at<float>(1, 1);
                        fmaxD = eValue.at<float>(1, 0);
                    }

                    double t = -(nx*dx.at<float>(j, i) + ny*dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j, i) + 2 * nx*ny*dyy.at<float>(j, i) + ny*ny*dyy.at<float>(j, i));

                    if ((fabs(t*nx) <= 0.5) && (fabs(t*ny) <= 0.5))
                    {
                        Point2i oriPoint;
                        oriPoint.x = i;
                        oriPoint.y = j;
                        oriImagePoint.push_back(oriPoint);

                        Point2f subpixelPoint;
                        subpixelPoint.x = i + t*nx;
                        subpixelPoint.y = j + t*ny;
                        subPixelImagePoint.push_back(subpixelPoint);//亚像素的光条中心点
                    }
                }
            }
        }
        a.lightPlaneOriImagePoint.push_back(oriImagePoint);
        //a.lightPlaneSubpixelImagePoint.push_back(subPixelImagePoint);
		/* RANSAC算法拟合直线，滤除坏点 */
		//Mat ImageRANSAC = imread(".\\light_picture\\light1.jpg");
		double A, B, C;
		vector<bool> inliers;
		fitLineRANSAC(subPixelImagePoint, A, B, C, inliers);
		//for (unsigned int i = 0; i < subPixelImagePoint.size(); i++) {
		//	if (inliers[i])
		//		circle(ImageRANSAC, subPixelImagePoint[i], 3, Scalar(0, 255, 0), 3, 16);
		//	else
		//		circle(ImageRANSAC, subPixelImagePoint[i], 3, Scalar(0, 0, 255), 3, 16);
		//}

		B = B / A;
		C = C / A;
		A = A / A;

		//绘制直线
		//Point2d ptStart, ptEnd;
		//ptStart.x = 0;
		//ptStart.y = -(A*ptStart.x + C) / B;
		//ptEnd.x = -(B*ptEnd.y + C) / A;
		//ptEnd.y = 0;
		//line(ImageRANSAC, ptStart, ptEnd, Scalar(0, 0, 0), 1, 16);
		//cout << "A:" << A << " " << "B:" << B << " " << "C:" << C << " " << endl;
		//for (int k = 0; k < subPixelImagePoint.size(); k++)//在图中画出些点，然后保存图片
		//{
		//	circle(ImageRANSAC, subPixelImagePoint[k], 1, Scalar(0, 255, 0));
		//}
		//imwrite(".\\save\\imageRANSAC.jpg", ImageRANSAC);
		//计算所有点到拟合直线的距离
		//Mat ImageNO = imread(".\\light_picture\\light1.jpg");
		vector<Point2f> subPixelImagePointTemp;
		for (int i = 0 ; i < subPixelImagePoint.size() ; i++)
		{
			float distance = abs(A*subPixelImagePoint[i].x + B*subPixelImagePoint[i].y + C) / sqrt(A*A + B*B);
			//cout << "点" << i << "到直线的距离 :  "<<  distance  << endl;
			if (distance > 1)
			{
				;
			}
			else
			{
				//circle(ImageNO, subPixelImagePoint[i], 1, Scalar(0, 255, 0));
				subPixelImagePointTemp.push_back(subPixelImagePoint[i]);
			}
		}
		subPixelImagePoint = subPixelImagePointTemp;
		a.lightPlaneSubpixelImagePoint.push_back(subPixelImagePoint);
		//imwrite(".\\save\\ImageNO.jpg", ImageNO);
		//Mat imagetemp = imread(".\\light_picture\\light1.jpg");
		//for (int ii = 0; ii < subPixelImagePoint.size(); ii++)
		//{
		//	circle(imagetemp, subPixelImagePoint[ii], 1, Scalar(0, 255, 0));
		//}
		//imwrite(".\\save\\imagetemp.jpg", imagetemp);
		//system("pause");

		/* RANSAC算法拟合直线部分结束 */

		Mat lightLineImageResult = Mat::zeros(lightLineImage.rows, lightLineImage.cols, CV_8U);
        for (int k = 0; k<subPixelImagePoint.size(); k++)//在图中画出些点，然后保存图片
        {
            circle(lightLineImageResult, subPixelImagePoint[k], 1, Scalar(255, 255, 255));
			//float distance1 = abs(A*subPixelImagePoint[k].x + B * subPixelImagePoint[k].y + C) / sqrt(A*A + B * B);
			//cout << "点" << k << "到直线的距离 :  " << distance1 << endl;
        }
		string format = ".jpg";
		string index = to_string(k);
		string name = "./save/result" + index + format;
        imwrite(name, lightLineImageResult);
        oriImagePoint.clear();
        subPixelImagePoint.clear();
    }
}

//结构光光条直线拟合与相交点坐标提取
//输入结构体类和点vector 结束之后vector就是光平面和靶标平面相交的直线上的点（ 7*9 = 63个 ）
void crossPoint(structure_light &a, vector<vector<Point2f>> &crossPoint)
{
    for (int i = 0; i < imageCount; i++)
    {
        if (a.isRecognize[i] == false)
            continue;
        Vec4f lightLine;
        vector<Point2f> lightLinePoint = a.lightPlaneSubpixelImagePoint[i];//取出第i张标定图像 的亚像素光条中心点
        fitLine(lightLinePoint, lightLine, DIST_L2, 0, 1e-2, 1e-2);//将第i张图的 光条中心点进行直线拟合
        vector<Point2f> cornerPoint = a.calibImagePoint[i];//靶标平面上圆形标记中心的点（49个）的二维坐标
        vector<Point2f> cornerlinePoint;//存储某一列的圆形标记的中心
        Vec4f cornerLine;//一列圆标记中心拟合出的直线的参数
        vector<Point2f> lightcornerPoint;//提取出的光条上的点
        //提取光条上的点，思路：取一列（或者一行）圆形标记的中心，拟合成直线，计算拟合出直线和光条的交点
        for (int m = 0; m < patternSize.width; m++)//遍历所有列7次
        {
            for (int n = 0; n < patternSize.height; n++)
            {
                cornerlinePoint.push_back(cornerPoint[n*patternSize.width + m]);//取第m列圆形标记的中心
            }
            fitLine(cornerlinePoint, cornerLine, DIST_L2, 0, 1e-2, 1e-2);//将第m列的圆心拟合成直线
            //求出第m列圆心拟合出来的直线和光条直线的交点
            double k1 = cornerLine[1] / cornerLine[0];
            double b1 = cornerLine[3] - k1*cornerLine[2];
            double k2 = lightLine[1] / lightLine[0];
            double b2 = lightLine[3] - k2*lightLine[2];
            Point2f temp;
            temp.x = (b2 - b1) / (k1 - k2);
            temp.y = k1*temp.x + b1;
            //将交点压入向量
            lightcornerPoint.push_back(temp);
            cornerlinePoint.clear();
        }
        crossPoint.push_back(lightcornerPoint);
    }
}

//利用交比不变求取光平面三维坐标
void crossRatio(structure_light &light, vector<vector<Point2f>> &crossPoint)
{
    for (int i = 0; i < imageCount; i++)
    {
        if (light.isRecognize[i] == false)
            continue;
        vector<Point2f> tempCrossPoint = crossPoint[i];//某一张图中的光条上的7个点的二维坐标
        vector<Point2f> tempCornerPoint = light.calibImagePoint[i];//第i个靶标平面上圆形标记中心的点（49个）的二维坐标
        vector<Point3f> tempWorldPoint = light.calibBoardPoint[i];//第i个靶标平面上圆形标记中心的点（49个）的三维点坐标
        vector<Point3f> tempCornerLinePoint;//用于存光条上的点的三维坐标
        //计算第i个靶标上的7个光条上的点的三维坐标（靶标坐标系下）循环7次
        for (int m = 0; m < tempCrossPoint.size(); m++)//循环7次
        {
            //读取每条圆斑点形成直线上前三个特征点的坐标
            Point2f a, b, c;
            Point3f A, B, C;
            a = tempCornerPoint[m];
            b = tempCornerPoint[patternSize.width+m];
            c = tempCornerPoint[2 * patternSize.width+m];
            A = tempWorldPoint[m];
            B = tempWorldPoint[patternSize.width + m];
            C = tempWorldPoint[2 * patternSize.width + m];

            //计算交比
            double crossRatio = ((a.y - c.y) / (b.y - c.y)) / ((a.y - tempCrossPoint[m].y) / (b.y - tempCrossPoint[m].y));
            //double crossRatio = ((a.x - c.x) / (b.x - c.x)) / ((a.x - tempCrossPoint[m].x) / (b.x - tempCrossPoint[m].x));
            //已知4个点的图像坐标与3个点世界坐标，可以计算其中1点的世界坐标（现在已知三个圆形标记中心和一个光条上的点的图像坐标，和三个标记点的三维坐标）
            Point3f crPoint;
			crPoint.x = m * patternLength.width;
			crPoint.y = (crossRatio*(B.y - C.y)*A.y - B.y*(A.y - C.y)) / (crossRatio*(B.y - C.y) - (A.y - C.y));//解出Y坐标
            crPoint.z = 0;
            cout << "ccs Point:" << crPoint << endl;
            tempCornerLinePoint.push_back(crPoint);
        }

        Mat Rvec = light.R[i]; Mat T1 = light.T[i];//靶标坐标系和摄像机坐标系之间的旋转平移矩阵
        Mat RMat1,RMat2;
        Rodrigues(Rvec, RMat1);//1x3 -> 3x3 旋转向量（1x3）与旋转矩阵（3x3），二者可以通过罗德里格斯相互转化

        //将靶标坐标系下的三维坐标转换成摄像机坐标系下的三维坐标
        vector<Point3f> tempPoint;
        for (int n = 0; n < tempCornerLinePoint.size(); n++)
        {
            Point3f realPoint;//摄像机坐标系下的坐标，接下来利用旋转变换矩阵将坐标转化
            realPoint.x = RMat1.at<double>(0, 0)*tempCornerLinePoint[n].x + RMat1.at<double>(0, 1)*tempCornerLinePoint[n].y + T1.at<double>(0, 0);
            realPoint.y = RMat1.at<double>(1, 0)*tempCornerLinePoint[n].x + RMat1.at<double>(1, 1)*tempCornerLinePoint[n].y + T1.at<double>(1, 0);
            realPoint.z = RMat1.at<double>(2, 0)*tempCornerLinePoint[n].x + RMat1.at<double>(2, 1)*tempCornerLinePoint[n].y + T1.at<double>(2, 0);
            cout << "wcs Point:" << realPoint << endl << endl;
            light.lightPlanePoint.push_back(realPoint);
        }
        tempCornerLinePoint.clear();
        tempWorldPoint.clear();
        tempCornerPoint.clear();
        tempCrossPoint.clear();
    }
}

//结构光的光平面拟合
void lightPlainFitting(structure_light &light)
{
    Mat A = Mat::zeros(3, 3, CV_64FC1);//定义拟合所需要的三个矩阵
    Mat B = Mat::zeros(3, 1, CV_64FC1);
    Mat X = Mat::zeros(3, 1, CV_64FC1);

    A.at<double>(2, 2) = light.lightPlanePoint.size();
    for (int i = 0; i < light.lightPlanePoint.size(); i++)
    {
        A.at<double>(0, 0) += light.lightPlanePoint[i].x*light.lightPlanePoint[i].x;
        A.at<double>(0, 1) += light.lightPlanePoint[i].x*light.lightPlanePoint[i].y;
        A.at<double>(0, 2) += light.lightPlanePoint[i].x;
        A.at<double>(1, 0) += light.lightPlanePoint[i].x*light.lightPlanePoint[i].y;
        A.at<double>(1, 1) += light.lightPlanePoint[i].y*light.lightPlanePoint[i].y;
        A.at<double>(1, 2) += light.lightPlanePoint[i].y;
        A.at<double>(2, 0) += light.lightPlanePoint[i].x;
        A.at<double>(2, 1) += light.lightPlanePoint[i].y;
        B.at<double>(0, 0) += light.lightPlanePoint[i].x*light.lightPlanePoint[i].z;
        B.at<double>(1, 0) += light.lightPlanePoint[i].y*light.lightPlanePoint[i].z;
        B.at<double>(2, 0) += light.lightPlanePoint[i].z;
    }

    solve(A, B, X);

    cout << "X:" << X << endl << endl;

    light.planeFormular[0] = -X.at<double>(0, 0);//x
    light.planeFormular[1] = -X.at<double>(1, 0);//y
    light.planeFormular[2] = 1;//z
    light.planeFormular[3] = X.at<double>(2, 0);//常数项
}

//差量法提取
void extratLightLine(int imageIndex, Mat &outputImage, structure_light &a)
{
    string num = to_string(imageIndex);
    string index = ".jpg";
    string name1 = "./calib_picture/calib" + num + index;
    string name2 = "./light_picture/light" + num + index;

    Mat oriImage = imread(name1);
    Mat lightImage = imread(name2);

    cvtColor(oriImage, oriImage, COLOR_BGR2GRAY);
    cvtColor(lightImage, lightImage, COLOR_BGR2GRAY);
    Mat diff;
    absdiff(oriImage, lightImage, diff);
    imwrite("./save/diff.jpg", diff);

    Mat element1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(diff, diff, MORPH_OPEN, element1);
    imwrite("./save/open.jpg", diff);

    threshold(diff, diff, 30, 255, THRESH_BINARY);
    imwrite("./save/threshold.jpg", diff);

    Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(diff, diff, element2);
    imwrite("./save/erode.jpg", diff);

    vector<vector<Point>>  allmaskPoint;
    allmaskPoint.push_back(vector<Point>());
    Mat mask = Mat::zeros(oriImage.size(), CV_8UC1);
    Point2f tempPoint;
    Point maskPoint;
    vector<Point2f> tempPointSet = a.calibImagePoint[imageIndex];
    vector<Point2f> maskPointSet;

    tempPoint = tempPointSet[0];
    maskPoint.x = cvRound(tempPoint.x);
    maskPoint.y = cvRound(tempPoint.y);
    allmaskPoint[0].push_back(maskPoint);

    tempPoint = tempPointSet[a.patternSize.width - 1];
    maskPoint.x = cvRound(tempPoint.x);
    maskPoint.y = cvRound(tempPoint.y);
    allmaskPoint[0].push_back(maskPoint);

    tempPoint = tempPointSet[a.patternSize.width*a.patternSize.height - 1];
    maskPoint.x = cvRound(tempPoint.x);
    maskPoint.y = cvRound(tempPoint.y);
    allmaskPoint[0].push_back(maskPoint);

    tempPoint = tempPointSet[a.patternSize.width*(a.patternSize.height - 1)];
    maskPoint.x = cvRound(tempPoint.x);
    maskPoint.y = cvRound(tempPoint.y);
    allmaskPoint[0].push_back(maskPoint);

    drawContours(mask, allmaskPoint, 0, Scalar(255), FILLED, 8);
    diff.copyTo(outputImage, mask);
    imwrite("./save/mask.jpg", mask);
    imwrite("./save/ROI.jpg", outputImage);

}

//计算摄像机坐标系下的 光条的三维坐标
void calc3DCoordination(structlight a, vector<Point2f> &centerPoint, vector<Point3f> &calcPoint)
{
	double focalLength = 3.5;//摄像机的焦距
	double dx, dy;
	dx = focalLength / a.cameraIntrinsic.at<double>(0, 0);//fx=f/dx fy=f/dy 摄像机在u轴和v轴方向上的尺度因子
	dy = focalLength / a.cameraIntrinsic.at<double>(1, 1);
	double u0 = a.cameraIntrinsic.at<double>(0, 2);//主偏移点
	double v0 = a.cameraIntrinsic.at<double>(1, 2);
	Point3f ccsPoint1(0, 0, 0);//图像上的点，以厘米为单位的三维坐标
	if (centerPoint.size() == 0)
	{
		cout << "No lightline in the image! " << endl;
		return;
	}
	for (int i = 0; i < centerPoint.size(); i++)//遍历一边提取出来的光条的二维点
	{
		Point2f imgPoint = centerPoint[i];
		//cout << "Image Point:" << imgPoint << endl << endl;//************
		// 计算 摄像机坐标系下，图像上光条中心点，以厘米为单位的 三维坐标
		ccsPoint1.x = dx * imgPoint.x - u0 * dx;
		ccsPoint1.y = dy * imgPoint.y - v0 * dy;
		ccsPoint1.z = focalLength;

		// 射线和光平面的交点，计算光条点的世界坐标系
		Point3f worldPoint(0, 0, 0);
		worldPoint.x = a.lightPlaneFormular[3] / (a.lightPlaneFormular[0]
			+ a.lightPlaneFormular[1] * ccsPoint1.y / ccsPoint1.x
			+ a.lightPlaneFormular[2] * ccsPoint1.z / ccsPoint1.x);
		worldPoint.y = ccsPoint1.y*worldPoint.x / ccsPoint1.x;// x 求出来之后 y,z可以利用比例求出
		worldPoint.z = ccsPoint1.z*worldPoint.x / ccsPoint1.x;

		calcPoint.push_back(worldPoint);
	}
	//// 将三维坐标写入TXT文件中
	//ofstream outfile;
	//outfile.open("pointcloud.txt", ios::binary | ios::in | ios::out);
	//for (int k = 0; k < calcPoint.size(); k++)
	//{
	//	outfile << calcPoint[k].x << "   ";
	//	outfile << calcPoint[k].y << "   ";
	//	outfile << calcPoint[k].z << "\n";
	//}
	//outfile.close();
}
/* 误差评估函数，思路是计算标定用的7个点和点云之间的平均距离，进一步评估测量效果 */
void EstimateError(structure_light StructureLightCalib, structlight StructlightMeasure, vector<vector<float>> BackProjectError, int imageCount)
{
	/* 评估之前定义靶标的列数和评估的图像编号 */
	int BoardRow = 7;
	int ImageNum = 2;
	vector<float> BackProjectErrorTemp;
	/* 首先取出光条和每列圆斑的交点的三维坐标，这是标定（拟合平面）用的 */
	vector<Point3f> CrossPoint;//光条和每列圆斑的交点的三维坐标（所有图的）
	CrossPoint = StructureLightCalib.lightPlanePoint;
	/* 用拟合出来的平面计算光条中心的三维坐标 */
	vector<Point3f> calcPoint;//第ImageNum张图光条中心的三维坐标
	calc3DCoordination(StructlightMeasure, StructureLightCalib.lightPlaneSubpixelImagePoint[ImageNum], calcPoint);
	Mat imagetemp = imread("./light_picture/light4.jpg");
	for (int ii = 0; ii < StructureLightCalib.lightPlaneSubpixelImagePoint[ImageNum].size(); ii++)
	{
		circle(imagetemp, StructureLightCalib.lightPlaneSubpixelImagePoint[ImageNum][ii], 1, Scalar(0, 255, 0));
	}
	imwrite("./save/imagetemp.jpg", imagetemp);
	/* 计算点之间的最小距离 评估误差 */
	for (int i = ImageNum * BoardRow; i < (ImageNum + 1) * BoardRow; i++)
	{
		float MinDistacne = sqrt((CrossPoint[i].x - calcPoint[0].x)*(CrossPoint[i].x - calcPoint[0].x)
			+ (CrossPoint[i].y - calcPoint[0].y)*(CrossPoint[i].y - calcPoint[0].y)
			+ (CrossPoint[i].z - calcPoint[0].z)*(CrossPoint[i].z - calcPoint[0].z));

		for (int k = 0; k < calcPoint.size(); k++)
		{
			float distance = sqrt((CrossPoint[i].x - calcPoint[k].x)*(CrossPoint[i].x - calcPoint[k].x)
				+ (CrossPoint[i].y - calcPoint[k].y)*(CrossPoint[i].y - calcPoint[k].y)
				+ (CrossPoint[i].z - calcPoint[k].z)*(CrossPoint[i].z - calcPoint[k].z));
			if (MinDistacne > distance)
			{
				MinDistacne = distance;
			}
			else
			{
				MinDistacne = MinDistacne;
			}

		}
		BackProjectErrorTemp.push_back(MinDistacne);
		cout << "最小距离" << MinDistacne << endl;
	}
	BackProjectError.push_back(BackProjectErrorTemp);

	// 将三维坐标写入TXT文件中
	ofstream outfile;
	outfile.open("pointcloud2.txt", ios::binary | ios::in | ios::out);
	for (int k = ImageNum * BoardRow; k < (ImageNum + 1) * BoardRow; k++)
	{
		outfile << CrossPoint[k].x << "          ";
		outfile << CrossPoint[k].y << "          ";
		outfile << CrossPoint[k].z << "\n";
	}
	for (int k = 0; k < calcPoint.size(); k++)
	{
		outfile << calcPoint[k].x << "          ";
		outfile << calcPoint[k].y << "          ";
		outfile << calcPoint[k].z << "\n";
	}
	outfile.close();

}

/* 误差评估函数，思路是计算标定用的7个点和点云拟合出的直线之间的平均距离，进一步评估测量效果 */
void EstimateError2(structure_light StructureLightCalib, structlight StructlightMeasure, vector<vector<float>> BackProjectError, int imageCount)
{
	/* 评估之前定义靶标的列数和评估的图像编号 */
	int BoardRow = 7;
	int ImageNum = 4;
	/* 首先取出光条和每列圆斑的交点的三维坐标，这是标定（拟合平面）用的 */
	vector<Point3f> CrossPoint;//光条和每列圆斑的交点的三维坐标（所有图的）
	CrossPoint = StructureLightCalib.lightPlanePoint;
	/* 用拟合出来的平面计算光条中心的三维坐标 */
	vector<Point3f> calcPoint;//第ImageNum张图光条中心的三维坐标
	calc3DCoordination(StructlightMeasure, StructureLightCalib.lightPlaneSubpixelImagePoint[ImageNum], calcPoint);
	Mat imagetemp = imread("./light_picture/light4.jpg");
	for (int ii = 0; ii < StructureLightCalib.lightPlaneSubpixelImagePoint[ImageNum].size(); ii++)
	{
		circle(imagetemp, StructureLightCalib.lightPlaneSubpixelImagePoint[ImageNum][ii], 1, Scalar(0, 255, 0));
	}
	imwrite("./save/imagetemp.jpg", imagetemp);
	/*################# 对重构的点云进行直线拟合 #################*/
	/*把坐标转换成矩阵 N*3的矩阵 */
	vector<vector<double>> A_pointcloud;
	for (int i = 0; i < calcPoint.size(); i++)
	{
		vector<double> A_pointcloudTemp;
		A_pointcloudTemp.push_back(calcPoint[i].x);
		A_pointcloudTemp.push_back(calcPoint[i].y);
		A_pointcloudTemp.push_back(calcPoint[i].z);
		A_pointcloud.push_back(A_pointcloudTemp);
	}
	cout << A_pointcloud.size() << endl;
	/* 拟合的直线必过所有坐标的算数平均值 得到直线上一点 */
	double LineX0, LineY0, LineZ0, X_sum = 0, Y_sum = 0, Z_sum = 0;
	for (int i = 0; i < A_pointcloud.size(); i++)
	{
		X_sum = X_sum + A_pointcloud[i][0];
		Y_sum = Y_sum + A_pointcloud[i][1];
		Z_sum = Z_sum + A_pointcloud[i][2];
	}
	LineX0 = X_sum / A_pointcloud.size();
	LineY0 = Y_sum / A_pointcloud.size();
	LineZ0 = Z_sum / A_pointcloud.size();//平均值就是直线经过的点

	/* 协方差矩阵奇异变换（SVD分解）得到直线方向向量 */
	vector<vector<double> > U;
	vector<double> S;
	vector<vector<double> > V;
	for (int i = 0; i < A_pointcloud.size(); i++)
	{
		A_pointcloud[i][0] = A_pointcloud[i][0] - LineX0;
		A_pointcloud[i][1] = A_pointcloud[i][1] - LineY0;
		A_pointcloud[i][2] = A_pointcloud[i][2] - LineZ0;
	}
	svd(A_pointcloud, 1, U, S, V);
	cout << "V矩阵大小 " << V.size() << " [ " << V[0][0] << " " << V[0][1] << " " << V[0][2] << " ] " << endl;
	//直线方程 (x - x0)/m = (y - y0)/n = (z - z0)/p = t ;x0,y0,z0是LineX0，LineY0，LineZ0；m,n,p对应这V的三个元素
	/*################# 拟合完成 #################*/
	/* 计算点之间的最小距离 评估误差 */
	vector<float> BackProjectErrorTemp;
	for (int i = ImageNum * BoardRow; i < (ImageNum + 1) * BoardRow; i++)
	{
		float MinDistacne = sqrt((CrossPoint[i].x - calcPoint[0].x)*(CrossPoint[i].x - calcPoint[0].x)
			+ (CrossPoint[i].y - calcPoint[0].y)*(CrossPoint[i].y - calcPoint[0].y)
			+ (CrossPoint[i].z - calcPoint[0].z)*(CrossPoint[i].z - calcPoint[0].z));
		float t, Xc, Yc, Zc;//t直线方程的参数，Xc,Yc,Zc是标定用的点做直线的垂线的垂足；下边是计算公式，目的是计算标定用点到拟合直线的距离
		t = -(V[0][0] * (LineX0 - CrossPoint[i].x) + V[0][1] * (LineY0 - CrossPoint[i].y) + V[0][2] * (LineZ0 - CrossPoint[i].z)) / (V[0][0] * V[0][0] + V[0][1] * V[0][1] + V[0][2] * V[0][2]);
		Xc = V[0][0] * t + LineX0;
		Yc = V[0][1] * t + LineY0;
		Zc = V[0][2] * t + LineZ0;
		float distance = sqrt((CrossPoint[i].x - Xc)*(CrossPoint[i].x - Xc)
			+ (CrossPoint[i].y - Yc)*(CrossPoint[i].y - Yc)
			+ (CrossPoint[i].z - Zc)*(CrossPoint[i].z - Zc));
		BackProjectErrorTemp.push_back(distance);
		cout << "点到线的距离" << distance << endl;
	}
	/* 计算7个点的平均距离 */
	float DistanceSum = 0, DistanceMaen = 0;
	for (int i = 0; i < BackProjectErrorTemp.size(); i++)
	{
		DistanceSum = DistanceSum + BackProjectErrorTemp[i];
	}
	DistanceMaen = DistanceSum / BackProjectErrorTemp.size();
	cout << "平均距离: " << DistanceMaen << endl;
	BackProjectError.push_back(BackProjectErrorTemp);

	// 将三维坐标写入TXT文件中
	ofstream outfile;
	outfile.open("pointcloud2.txt", ios::binary | ios::in | ios::out);
	for (int k = ImageNum * BoardRow; k < (ImageNum + 1) * BoardRow; k++)
	{
		outfile << CrossPoint[k].x << "          ";
		outfile << CrossPoint[k].y << "          ";
		outfile << CrossPoint[k].z << "\n";
	}
	for (int k = 0; k < calcPoint.size(); k++)
	{
		outfile << calcPoint[k].x << "          ";
		outfile << calcPoint[k].y << "          ";
		outfile << calcPoint[k].z << "\n";
	}
	outfile.close();

}
/* ############ SVD相关代码 ########################*/
double get_norm(double *x, int n) {
	double r = 0;
	for (int i = 0; i < n; i++)
		r += x[i] * x[i];
	return sqrt(r);
}
double normalize(double *x, int n) {
	double r = get_norm(x, n);
	if (r < eps)
		return 0;
	for (int i = 0; i < n; i++)
		x[i] /= r;
	return r;
}

inline double product(double*a, double *b, int n) {
	double r = 0;
	for (int i = 0; i < n; i++)
		r += a[i] * b[i];
	return r;
}

void orth(double *a, double *b, int n) {//|a|=1
	double r = product(a, b, n);
	for (int i = 0; i < n; i++)
		b[i] -= r * a[i];

}

bool svd(vector<vector<double> > A, int K, vector<vector<double> > &U, vector<double> &S, vector<vector<double> > &V) {
	int M = A.size();
	int N = A[0].size();
	U.clear();
	V.clear();
	S.clear();
	S.resize(K, 0);
	U.resize(K);
	for (int i = 0; i < K; i++)
		U[i].resize(M, 0);
	V.resize(K);
	for (int i = 0; i < K; i++)
		V[i].resize(N, 0);


	srand(time(0));
	double *left_vector = new double[M];
	double *next_left_vector = new double[M];
	double *right_vector = new double[N];
	double *next_right_vector = new double[N];
	int col = 0;
	for (int col = 0; col < K; col++) {
		double diff = 1;
		double r = -1;
		while (1) {
			for (int i = 0; i < M; i++)
				left_vector[i] = (float)rand() / RAND_MAX;
			if (normalize(left_vector, M) > eps)
				break;
		}

		for (int iter = 0; diff >= eps && iter < MAX_ITER; iter++) {
			memset(next_left_vector, 0, sizeof(double)*M);
			memset(next_right_vector, 0, sizeof(double)*N);
			for (int i = 0; i < M; i++)
				for (int j = 0; j < N; j++)
					next_right_vector[j] += left_vector[i] * A[i][j];

			r = normalize(next_right_vector, N);
			if (r < eps) break;
			for (int i = 0; i < col; i++)
				orth(&V[i][0], next_right_vector, N);
			normalize(next_right_vector, N);

			for (int i = 0; i < M; i++)
				for (int j = 0; j < N; j++)
					next_left_vector[i] += next_right_vector[j] * A[i][j];
			r = normalize(next_left_vector, M);
			if (r < eps) break;
			for (int i = 0; i < col; i++)
				orth(&U[i][0], next_left_vector, M);
			normalize(next_left_vector, M);
			diff = 0;
			for (int i = 0; i < M; i++) {
				double d = next_left_vector[i] - left_vector[i];
				diff += d * d;
			}

			memcpy(left_vector, next_left_vector, sizeof(double)*M);
			memcpy(right_vector, next_right_vector, sizeof(double)*N);
		}
		if (r >= eps) {
			S[col] = r;
			memcpy((char *)&U[col][0], left_vector, sizeof(double)*M);
			memcpy((char *)&V[col][0], right_vector, sizeof(double)*N);
		}
		else {
			cout << r << endl;
			break;
		}
	}
	delete[] next_left_vector;
	delete[] next_right_vector;
	delete[] left_vector;
	delete[] right_vector;

	return true;
}
/* ############ SVD相关代码结束 ########################*/

