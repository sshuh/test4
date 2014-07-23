#pragma once

#include "hs2def.h"

#include "HS2_Utils.h"

//#include "Feature_Test.h"

//#include "HS2_LaserCamSLAM.h"

#include "OpenCV2.4.3/include/opencv2/opencv.hpp"
using namespace cv;



class ClassDataVideoLog
{
public:
	enum{ PROCESS_OK=1, ERROR_UNKNOWN=0, ERROR_FILE_OPEN=-1, ERROR_VIDEO_OPEN=-2, ERROR_FILE_READ=-3, ERROR_VIDEO_READ=-4 };

	//cv::Mat m_imgUndistorted;
	cv::Mat m_imgSrcColor;
	cv::Mat m_imgSrcGray;



////////////////////////////////  LOG   ////////////////////////////////////
	// Logging
	cv::VideoWriter m_videoWriter;
	int m_iVideoWriterOn;   

	int DataVideoLogging( cv::Mat imgSrc );
	int DataLogging();
	int VideoLogging( cv::Mat imgSrc );

	void LogVideo_Open(char *writeFileName);
	void LogVideo_Loop(cv::Mat imgSrc);
	void LogVideo_Close();

	int LogData_Open(char *writeFileName);
	void LogData_Loop();
	void LogData_Close();


	int LogDataProperty( char *cFilename );	// write data properties to first section of the file 
	

////////////////////////////////  REPLAY   ////////////////////////////////////
//--------------------------------------------------------
// related to dat+video replay
//--------------------------------------------------------
	FILE* m_DataFile;
	StoredRobotStates_New m_StoredRobotStates;

	// video
	cv::VideoCapture m_capture;
	int ReplayVideo_Open( char* cNameVideoFile );
	int  ReplayVideo_Loop();
	void ReplayVideo_Close();

	// data
	int ReplayData_Open( char* cNameDataFile );
	int  ReplayData_Loop();
	void ReplayData_Close();

	int ReadDataProperty();	// load data properties at the first section of the file 


	// video + data
	int ReplayDataVideo_Open( int *iReplayDatVideo_On );
	int ReplayDataVideo_Loop( int *iReplayDatVideo_On );
	int ReplayDataVideo_Close();
	
	long m_nCountDatReading;
	int m_nCountVideoReading;

	// For old format data file
	StoredRobotStates_Legacy m_StoredRobotStates_Legacy;
	int ReplayDataVideo_Open_Legacy( int *iReplayDatVideo_On );
	int ReplayDataVideo_Loop_Legacy( int *iReplayDatVideo_On );

};




