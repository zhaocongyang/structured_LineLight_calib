// use for calibration
#include "structure_light.h"
#include "struct_light_calib.h"

structure_light::structure_light(int x, cv::Size patternSize, cv::Size patternLength) {
	structure_light::imageCount = x;
	structure_light::patternSize = patternSize;
	structure_light::PatternLength = patternLength;
}

void structure_light::generateCalibboardPoint()
{
	for (int i = 0; i < structure_light::imageCount; i++)
	{
		if (structure_light::isRecognize[i])
		{
			std::vector<cv::Point3f> tempPoint;
			for (int j = 0; j < structure_light::patternSize.height; j++)
			{
				for (int k = 0; k < structure_light::patternSize.width; k++)
				{
					cv::Point3f temp;
					temp.x = k*structure_light::PatternLength.width;
					temp.y = j*structure_light::PatternLength.height;
					temp.z = 0;
					tempPoint.push_back(temp);
				}
			}
			structure_light::calibBoardPoint.push_back(tempPoint);
			tempPoint.clear();
		}
	}
}

void structure_light::outputResult()
{
	cv::FileStorage out("Parameters.xml", cv::FileStorage::WRITE);
	out << "cameraMatrix" << cameraMatrix;
	out << "distCoeffs" << distCoeffs;
	out << "Formular_A" << planeFormular[0];
	out << "Formular_B" << planeFormular[1];
	out << "Formular_C" << planeFormular[2];
	out << "Formular_D" << planeFormular[3];
	out << "Rw" << Rw;
	out << "Tw" << Tw;
}