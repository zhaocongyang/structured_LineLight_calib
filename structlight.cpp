#include "structlight.h"

using namespace cv;

structlight::structlight()
{
	
}


structlight::~structlight()
{
}

void structlight::readParameters()
{
	FileStorage fs;
	fs.open("Parameters.xml", FileStorage::READ);
	if (fs.isOpened() == false)
	{
		std::cout << "Need calibration result!" << std::endl;
		exit(-1);
	}
	fs["cameraMatrix"] >> cameraIntrinsic;
	fs["distCoeffs"] >> distCoeffs;
	fs["Formular_A"] >> lightPlaneFormular[0];
	fs["Formular_B"] >> lightPlaneFormular[1];
	fs["Formular_C"] >> lightPlaneFormular[2];
	fs["Formular_D"] >> lightPlaneFormular[3];
	fs["Rw"] >> Rw;
	fs["Tw"] >> Tw;
	
}