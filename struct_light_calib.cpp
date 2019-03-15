//
// Created by 赵丛杨 on 2018/12/23.
//
#include "struct_light_calib.h"
#include "structure_light.h"

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
        string name = "calib_picture/calib" + index + format;
        Mat pic = imread(name);

        Mat greyImage;
        cvtColor(pic, greyImage, COLOR_BGR2GRAY);

        bool result = true;
        vector<Point2f> targetPoint;
        //圆形靶标提取圆心点
        if (iscircle)
        {
            if (0 == findCirclesGrid(greyImage, a.patternSize, targetPoint))
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
                imwrite("save/Corner.jpg", greyImage);
            }
        }
    }

    a.generateCalibboardPoint();

    double rms = calibrateCamera(a.calibBoardPoint, a.calibImagePoint, image_size, a.cameraMatrix, a.distCoeffs, a.R, a.T);
    cout << "Reprojection error:" << rms << endl;

    a.Rw = a.R[0];
    a.Tw = a.T[0];

    double err = 0.0;
    double mean_err = 0.0;
    double total_err = 0.0;
    int i;
    vector<Point2f> reprojectionPoint;

    for (i = 0; i<a.calibBoardPoint.size(); i++)
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

//提取结构光光条中心点（steger算法）
void stegerLine(structure_light &a)
{
    for (int k = 0; k < a.imageCount; k++)
    {
        if (a.isRecognize[k] == false)
            continue;

        Mat lightLineImage;
        extratLightLine(k, lightLineImage, a);

        imwrite("save/image.jpg", lightLineImage);
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
        a.lightPlaneSubpixelImagePoint.push_back(subPixelImagePoint);
        for (int k = 0; k<subPixelImagePoint.size(); k++)
        {

            circle(lightLineImage, subPixelImagePoint[k], 1, Scalar(0, 255, 0));
        }
        imwrite("save/result.jpg", lightLineImage);
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
            //读取每条角点形成直线上前三个特征点的坐标
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
            crPoint.x = m*patternLength.width;
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
    string name1 = "calib_picture/calib" + num + index;
    string name2 = "light_picture/light" + num + index;

    Mat oriImage = imread(name1);
    Mat lightImage = imread(name2);

    cvtColor(oriImage, oriImage, COLOR_BGR2GRAY);
    cvtColor(lightImage, lightImage, COLOR_BGR2GRAY);
    Mat diff;
    absdiff(oriImage, lightImage, diff);
    imwrite("save/diff.jpg", diff);

    Mat element1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(diff, diff, MORPH_OPEN, element1);
    imwrite("save/open.jpg", diff);

    threshold(diff, diff, 30, 255, THRESH_BINARY);
    imwrite("save/threshold.jpg", diff);

    Mat element2 = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(diff, diff, element2);
    imwrite("save/erode.jpg", diff);

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
    imwrite("save/mask.jpg", mask);
    imwrite("save/ROI.jpg", outputImage);

}