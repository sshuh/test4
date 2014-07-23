#pragma once

#ifdef _WIN32

#include "OpenCV2.4.3/include/opencv2/opencv.hpp"
#endif

#ifdef __linux__
#include "opencv2/opencv.hpp"
#endif

using namespace cv;

class ClassExtractColor
{
public:
	cv::Mat m_imgBGR[3];
	cv::Mat m_imgTemp;
	cv::Mat m_imgRofB;
	cv::Mat m_imgGofB;
	cv::Mat m_imgRofS;
	cv::Mat m_imgDilated;

	cv::Mat m_imgThresholded;

	int Initiate( cv::Mat imgSrcColor );
	int ExtractColor_Red( cv::Mat imgSrcColor, cv::Mat imgResultBinary );
};


class ClassContourProcess
{
public:
	typedef struct
	{
		double pos[2];	// pos in image
		double area;	// pixel area

		double mu[2];	// moment of inertia
		double aspectratio;// aspectratio
	} ContourStates;

	std::vector <ContourStates> m_vContourStates;
	cv::Mat m_imgContours;
	cv::Moments m_ContourMoments;

	int Initiate( cv::Mat imgSrc );
	int ExtractContours( cv::Mat imgSrcBinary );
	int FindTheBiggestMarker( std::vector <ContourStates> vContourStates );
	
	int EstimateDistanceToMarker( double dAreaActual, double dAreaMeasured, double *dPosImg, float *fCameraAtt, double *dDistResult );
};