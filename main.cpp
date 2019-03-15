/*
 * 使用圆形靶标进行标定
 * 2019.1.19 修订
 * 修改人：赵丛杨
 * 功能：实现线结构光+单目系统的标定
 */
#include "structure_light.h"
#include "struct_light_calib.h"


using namespace cv;
using namespace std;
//定义一些全局变量作为标定的靶标参数
int imageCount = 6;//图像的数量
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


}


