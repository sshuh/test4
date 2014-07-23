// Sungsik Huh
// 2013.09.23
// 

#ifndef CAM_POINTGREY_H_
#define CAM_POINTGREY_H_
#endif /* CAM_POINTGREY_H_ */

#define PTGREY_CAM_LIB_VER		2	// 1 or 2

// ------------------------------ _WIN32 --------------------------------
#ifdef _WIN32
#pragma once
#include <Windows.h>
#include "OpenCV2.4.3/include/opencv2/opencv.hpp"

	#if( PTGREY_CAM_LIB_VER == 2 )
	// PGR Includes
	#include "ptgrey2/FlyCapture2.h"
	#include "ptgrey2/FlyCapture2GUI.h"
	using namespace FlyCapture2;
	#endif

	#if( PTGREY_CAM_LIB_VER == 1 )
	// ---- FlyCapture 1.8 ----
	#include "ptgrey1/pgrflycapture.h"
	typedef struct FlyCaptureImage_tmp
	{
		int iRows;
		int iCols;
		int iRowInc;
		FlyCaptureVideoMode videoMode;
		FlyCaptureTimestamp timeStamp;
		unsigned char* pData;
		bool bStippled;
		bool bTmp[3];
		FlyCapturePixelFormat pixelFormat;
		int iNumImages;
		unsigned long  ulReserved[5];
	} FlyCaptureImageTmp;
	#endif

#endif
/*
#include "hs2def.h"

#if( FSLAM_VERSION == 1 )
	#include "F_SLAM.h"
#endif
#if( FSLAM_VERSION == 2 )
	#include "F_SLAM2.h"
#endif
#if( FSLAM_VERSION == 3 )
	#include "F_SLAM3.h"
#endif
#if( FSLAM_VERSION == 4 )
	#include "F_SLAM4.h"
#endif


#include "Feature_Test.h"

#include "HS2_LaserCamSLAM.h"

#include "HS2_DataVideoLog.h"
*/

#ifdef __linux__
// PGR Includes
#include "flycapture/FlyCapture2.h"
#include "flycapture/FlyCapture2GUI.h"

#include "opencv2/opencv.hpp"
using namespace FlyCapture2;

#ifndef Sleep
#define Sleep(x)	usleep(x*1000)
#endif

#endif



using namespace cv;

class ClassCameraImage
{
public:
	ClassCameraImage()
	{
		//imgSrc.create( 480, 640, CV_8UC3 );
		//imgSrc.setTo( 255 );
		//circle( imgSrc, Point(200,200), 50, CV_RGB(255,0,0), 1 );
	}

	int Initiate( int height, int width, int channel )
	{
		imgSrc.create( height, width, CV_MAKETYPE(CV_8U,channel) );
		imgSrc.setTo( 255 );
		circle( imgSrc, Point(200,200), 50, CV_RGB(255,0,0), 1 );
		return 1;
	}

	~ClassCameraImage()
	{
		imgSrc.release();
	}

	cv::Mat imgSrc;
};


class ClassCamPointgrey
{
public:
	//int width;
	//int height;

	cv::Size SizeOfCamImage;

#if( PTGREY_CAM_LIB_VER == 2 )
	// ---- FlyCapture 2.3 ----
	FlyCapture2::BusManager m_BusManager;
	FlyCapture2::PGRGuid m_Guid;
	FlyCapture2::Camera m_Cam;
	FlyCapture2::FC2Config m_Config;
	FlyCapture2::Image m_Image;
	FlyCapture2::Image m_ImageColor;
	FlyCapture2::Error m_Error;
#endif

#if( PTGREY_CAM_LIB_VER == 1 )
	// ---- FlyCapture 1.8 ----
	FlyCaptureError error;
	FlyCaptureContext pContext;
	FlyCaptureImageTmp pImage;
	FlyCaptureImageTmp pImageBGR;
#endif

	int InitiateCamera( int iParam );
	void StopCamera();
	int GrabImage();

	// for camera parameter setting
	FlyCapture2::TimeStamp m_stTimeStamp;
	unsigned int m_uiTimestamp;
	unsigned int m_uiTimestampUser;
	int m_iCamSetting;
	void SetCameraParameter();

public:
	cv::Mat m_imgSrcColor;
	cv::Mat m_imgSrcGray;


//	cv::VideoCapture m_capture;
//	cv::VideoWriter m_videoWriter;
//	int m_iVideoWriterOn;

	//cv::Mat m_imgUndistorted;
	//cv::Mat mat_CamIntrinsic;
	//cv::Mat mat_CamDistCoeff;
	//cv::Mat m_imgMap1;
	//cv::Mat m_imgMap2;
};




