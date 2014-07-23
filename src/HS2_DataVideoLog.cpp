//////////////////////////////////
// Data and Video Logging Class //
//////////////////////////////////

//#include "stdafx.h"
#include "HS2_DataVideoLog.h"

extern SharedStates g_States;
extern StructSensorsDownlink	g_TcpSensorData;

extern LaserCamSlamStates g_LaserCamSlam;


extern short g_sMotorControl[10];

extern LaserParameters g_LaserParam1;

extern MCL3dStates g_Mcl3dStates;

int ClassDataVideoLog::DataVideoLogging( cv::Mat imgSrc )
{
	// --------------------Data Video Logging--------------------
	if( g_States.LogDataVideo_On == g_States.Off )
	{
		// No Logging
	}
	else if( g_States.LogDataVideo_On == g_States.On_Initialize )	// Log Data Video Started
	{
		CTime today = CTime::GetCurrentTime();
		char cNameDate[80];
		sprintf_s( cNameDate, sizeof(cNameDate), "hs2log_%02d%02d%02d_%02dh%02dm%02ds", 
					today.GetYear()-2000, today.GetMonth(), today.GetDay(), today.GetHour(), today.GetMinute(), today.GetSecond() );

		LogData_Open( cNameDate );
		LogDataProperty( cNameDate );

		LogVideo_Open( cNameDate );
		g_States.LogDataVideo_On = g_States.On_Loop;
	}
	else if( g_States.LogDataVideo_On == g_States.On_Loop )	// Log Data Video Loop
	{
		LogData_Loop();

		LogVideo_Loop( imgSrc );
	}
	else if( g_States.LogDataVideo_On == g_States.On_Terminate )	// Log Data Video End
	{
		LogData_Close();
		LogVideo_Close();
		g_States.LogDataVideo_On = g_States.Off;
	}

	return PROCESS_OK;
}



unsigned WINAPI Thread_DataLogging( void* arg )
{
	ClassDataVideoLog log1;

	while( 1 )
	{
		log1.DataLogging();
		Sleep( 50 );
	}


	return 1;
}

int ClassDataVideoLog::DataLogging()
{
	// --------------------Data Logging--------------------
	if( g_States.LogData_On == g_States.Off )
	{
		// No Logging
	}
	else if( g_States.LogData_On == g_States.On_Initialize )	// Log Data Started
	{
		CTime today = CTime::GetCurrentTime();
		char cNameDate[80];
		sprintf_s( cNameDate, sizeof(cNameDate), "hs2log_%02d%02d%02d_%02dh%02dm%02ds", 
					today.GetYear()-2000, today.GetMonth(), today.GetDay(), today.GetHour(), today.GetMinute(), today.GetSecond() );

		LogData_Open( cNameDate );
		LogDataProperty( cNameDate );
		g_States.LogData_On = g_States.On_Loop;
	}
	else if( g_States.LogData_On == g_States.On_Loop )	// Log Data Loop
	{
		LogData_Loop();
	}
	else if( g_States.LogData_On == g_States.On_Terminate )	// Log Data End
	{
		LogData_Close();
		g_States.LogData_On = g_States.Off;
	}

	return PROCESS_OK;
}


int ClassDataVideoLog::VideoLogging( cv::Mat imgSrc )
{
	// --------------------Video Logging--------------------
	if( g_States.LogVideo_On == g_States.Off )
	{
		// No Logging
	}
	else if( g_States.LogVideo_On == g_States.On_Initialize )	// Log Video Started
	{
		CTime today = CTime::GetCurrentTime();
		char cNameDate[80];
		sprintf_s( cNameDate, sizeof(cNameDate), "hs2log_%02d%02d%02d_%02dh%02dm%02ds", 
					today.GetYear()-2000, today.GetMonth(), today.GetDay(), today.GetHour(), today.GetMinute(), today.GetSecond() );

		LogVideo_Open( cNameDate );
		g_States.LogVideo_On = g_States.On_Loop;
	}
	else if( g_States.LogVideo_On == g_States.On_Loop )	// Log Video Loop
	{
		LogVideo_Loop( imgSrc );
	}
	else if( g_States.LogVideo_On == g_States.On_Terminate )	// Log Video End
	{
		LogVideo_Close();
		g_States.LogVideo_On = g_States.Off;
	}

	return PROCESS_OK;
}


///////////////////////////////////////////////
///////////////  Log Video   //////////////////
///////////////////////////////////////////////

void ClassDataVideoLog::LogVideo_Open( char *cNameDate )
{
	char cNameFullFile[80];
	sprintf_s( cNameFullFile, sizeof(cNameFullFile), "%s.avi", cNameDate );

	m_videoWriter.open( cNameFullFile, CV_FOURCC('X', 'V', 'I', 'D'), 15.0, cv::Size(640,480), 1 );
	// ('X', 'V', 'I', 'D')// ('M', 'P', '4', '3')

	m_iVideoWriterOn = 1;
}

void ClassDataVideoLog::LogVideo_Loop( Mat imgSrc )
{
	m_videoWriter.write( imgSrc );
}

void ClassDataVideoLog::LogVideo_Close()
{
	m_videoWriter.release();
}


///////////////////////////////////////////////
///////////////  Log Data   ///////////////////
///////////////////////////////////////////////

int ClassDataVideoLog::LogData_Open( char *cNameDate )
{
	char cNameFullFile[80];
	sprintf_s( cNameFullFile, sizeof(cNameFullFile), "%s.dat", cNameDate );

	m_DataFile = fopen(cNameFullFile, "wb");
	if( !m_DataFile ) 
		return ERROR_FILE_OPEN;

	return PROCESS_OK;
}


void ClassDataVideoLog::LogData_Loop()
{
	//memcpy( &(m_StoredRobotStates.RobotState[0]), g_LaserCamSlam.fMeanRobot, 3*sizeof(double) );
	//memcpy(m_StoredRobotStates.RobotVel, g_dProcRobotVel, 3*sizeof(double) );
	//memcpy(m_StoredRobotStates.RobotAtt, g_dProcRobotAtt, 3*sizeof(double) );

	for( int i=0; i<15; i++ )
		m_StoredRobotStates.RobotState[i] = (double)(g_LaserCamSlam.fMeanRobot[i]);

	// reserved 9 * 8
	//for( int i=15; i<24; i++ )
	//	m_StoredRobotStates.RobotState[i] = (double)(g_LaserCamSlam.fMeanRobot[i]);

	memcpy(m_StoredRobotStates.IMUdata, g_TcpSensorData.IMUdata, 12*sizeof(double) );
	memcpy(m_StoredRobotStates.ServoState, g_TcpSensorData.ServoState, 3*sizeof(double) );
	memcpy(m_StoredRobotStates.MotorControl, g_sMotorControl, 10*sizeof(short) );
	//memcpy(m_StoredRobotStates.ImageProperty, g_sImageProperty, 3*sizeof(short) );
	memcpy(m_StoredRobotStates.LRFdata, g_TcpSensorData.LRFdata, 1081*sizeof(short) );

	// reserved 10 * 4
	//for( int i=0; i<10; i++ )
	//	m_StoredRobotStates.OtherStates[i] = (int)(other states);

	fwrite(&m_StoredRobotStates, (size_t)(sizeof(StoredRobotStates_New)), 1, m_DataFile);
}


void ClassDataVideoLog::LogData_Close()
{
	fclose(m_DataFile);
}



// write data properties to first section of the file 
int ClassDataVideoLog::LogDataProperty( char *cFilename )
{
	char cDataProperty[1000];
	
	//memcpy( &cDataProperty[0], cFilename, 100 );	
	unsigned int iIdx = 0;
	unsigned int nLen;

	char *cFlag = "HS2_Data_File ";
	nLen = strnlen( cFlag, 20 );
	strncpy( &cDataProperty[iIdx], cFlag, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	nLen = strnlen( cFilename, 80 );
	strncpy( &cDataProperty[iIdx], cFilename, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = ' ';
	cDataProperty[iIdx++] = '\n';

	char *cLaserProperty = "#Laser:Hokuyo:UTM-30LX:Roll=0.0:Pitch=0.0:Yaw=0.0:PX=0.0:PY=0.0:PZ=0.0 ";
	nLen = strnlen( cLaserProperty, 100 );
	strncpy( &cDataProperty[iIdx], cLaserProperty, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	char *cCameraProperty = "#Camera:Pointgrey:FMVU-03MTC:Roll=0.0:Pitch=0.0:Yaw=0.0:PX=0.0:PY=0.0:PZ=0.0:FLhor=330.0:FLver=330.0:PPhor=320:PPver=240:Rd1=0.0:Rd2=0.0:Rd3=0.0:Rd4=0.0 ";
	nLen = strnlen( cCameraProperty, 200 );
	strncpy( &cDataProperty[iIdx], cCameraProperty, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	char *cServoProperty = "#Servo:Robotis:MX-28:Roll=0.0:Pitch=0.0:Yaw=0.0:PX=0.0:PY=0.0:PZ=0.0 ";
	nLen = strnlen( cServoProperty, 100 );
	strncpy( &cDataProperty[iIdx], cServoProperty, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	char *cImuProperty = "#IMU:Microstrain:3DMGX3-25 ";
	nLen = strnlen( cImuProperty, 100 );
	strncpy( &cDataProperty[iIdx], cImuProperty, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	char *cGpsProperty = "#GPS:NONE ";
	nLen = strnlen( cGpsProperty, 100 );
	strncpy( &cDataProperty[iIdx], cGpsProperty, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	char *cTcpGroundPacketProperty = "#DataStoreStruct:double_RobotPos[3]:double_RobotVel[3]:double_RobotAtt[3]:double_IMUdata[12]:double_ServoState[3]:short_MotorControl[40]:short_ImageProperty[3]:unsigned short_LRFdata[1081] ";
	nLen = strnlen( cTcpGroundPacketProperty, 200 );
	strncpy( &cDataProperty[iIdx], cTcpGroundPacketProperty, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	char *cEtcProperty = "#ETC:VIDEO_Recording:NONE ";
	nLen = strnlen( cEtcProperty, 100 );
	strncpy( &cDataProperty[iIdx], cEtcProperty, nLen );
	iIdx += nLen;
	cDataProperty[iIdx++] = '\n';

	cDataProperty[iIdx] = '\0';
	if( iIdx >= 1000 )
	{
		printf("ERROR:RecordDataProperty\n");
		return 0;
	}
	
	printf( cDataProperty );
	
	fwrite(cDataProperty, 1000, 1, m_DataFile);

	return PROCESS_OK;
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////                             ////////////////////////////////////////////////
///////////////////////////////////////            REPLAY           ////////////////////////////////////////////////
///////////////////////////////////////                             ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// data+video
unsigned WINAPI Thread_ReplayDataVideo( void* arg )
{

	ClassDataVideoLog ReplayLog1;

	if( ReplayLog1.ReplayDataVideo_Open( &(g_States.ReplayDataVideo_On) ) != ReplayLog1.PROCESS_OK )
	{
		printf("ReplayDataVideo_Open was not initialized\n");
		return -1;
	}

	//g_States.ReplayVideo_On = 1;
	namedWindow( "m_imgSrcColor" );

	F_SLAM4		Fslam4;

	Feature_Test test1;

	HS2_LaserCamSLAM LaserCamSlam1;

#ifdef FEATURE_MANAGEMENT_STANDALONE_TEST
	FeatureManager test_FeatureTracking;
#endif

	Sleep(100);

	while( g_States.ReplayDataVideo_On != 0 )
	{

		if( g_States.ReplayDataVideo_On == 1 )
		{
			cv::waitKey(100);
			continue;
		}

		if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_PredictionLoop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
		}


		if( ReplayLog1.ReplayDataVideo_Loop( &(g_States.ReplayDataVideo_On) ) != ReplayLog1.PROCESS_OK )
			break;

		cv::cvtColor( ReplayLog1.m_imgSrcColor, ReplayLog1.m_imgSrcGray, CV_BGR2GRAY );


		// FSLAM
		if( g_States.FusionSLAM_On == g_States.On_Initialize )
		{
			Fslam4.EKF_InitEKF( ReplayLog1.m_imgSrcGray );

			g_States.FusionSLAM_On = g_States.On_Loop;
		}
		else if( g_States.FusionSLAM_On == g_States.On_Loop )
		{
			Fslam4.EKF_Loop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
		}


		if( g_States.LaserCamSLAM_On == g_States.On_Initialize )
		{
			LaserCamSlam1.Filter_Initial( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
			g_States.LaserCamSLAM_On = g_States.On_Loop;
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_CorrectionLoop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor, g_TcpSensorData, g_States, &g_LaserCamSlam );
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Terminate )
		{
			LaserCamSlam1.Filter_Terminate();
			g_States.LaserCamSLAM_On = g_States.Off;
		}

#ifdef FEATURE_MANAGEMENT_STANDALONE_TEST
		if( g_States.Test_FeatureTrackingStandalone == g_States.On_Initialize )
		{		
			test_FeatureTracking.StandaloneTest_Initial( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
			g_States.Test_FeatureTrackingStandalone = g_States.On_Loop;
		}
		else if( g_States.Test_FeatureTrackingStandalone == g_States.On_Loop )
		{
			test_FeatureTracking.StandaloneTest_Loop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
		}
#endif


		if( g_States.Test == 1 )
		{
			test1.Initial_ForTest( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
			g_States.Test = 2;
		}
		else if( g_States.Test == 2 )
		{
			test1.Loop_ForTest( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
		}


		imshow( "m_imgSrcColor", ReplayLog1.m_imgSrcColor );
		
		if( g_States.ReplayDataVideo_On == 5 )	// for one-step forward replay
		{
			g_States.ReplayDataVideo_On = 1;	// then, stop
		}

		cv::waitKey(50);
	}

	g_States.ReplayDataVideo_On = 0;

	printf("Thread_ReplayDataVideo ended\n");
	
	return 1;
}


// Video
unsigned WINAPI Thread_ReplayVideo( void* arg )
{

	ClassDataVideoLog ReplayLog1;

	char *cNameDataFile = GetAVIfromFileDialog();
	if( cNameDataFile == NULL )
		return -1;

	if( ReplayLog1.ReplayVideo_Open( cNameDataFile ) != ReplayLog1.PROCESS_OK )
	{
		printf("ReplayVideo_Open was not initialized\n");
		return -1;
	}

	//g_States.ReplayVideo_On = 1;
	namedWindow( "m_imgSrcColor" );

	Feature_Test test1;

#ifdef FEATURE_MANAGEMENT_STANDALONE_TEST
	FeatureManager test_FeatureTracking;
#endif

	Sleep(100);

	while( g_States.ReplayVideo_On != 0 )
	{
		if( g_States.ReplayVideo_On == 1 )
		{
			cv::waitKey(100);
			continue;
		}

		if( ReplayLog1.ReplayVideo_Loop() != ReplayLog1.PROCESS_OK )
			break;

		cv::cvtColor( ReplayLog1.m_imgSrcColor, ReplayLog1.m_imgSrcGray, CV_BGR2GRAY );


#ifdef FEATURE_MANAGEMENT_STANDALONE_TEST
		if( g_States.Test_FeatureTrackingStandalone == g_States.On_Initialize )
		{		
			test_FeatureTracking.StandaloneTest_Initial( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
			g_States.Test_FeatureTrackingStandalone = g_States.On_Loop;
		}
		else if( g_States.Test_FeatureTrackingStandalone == g_States.On_Loop )
		{
			test_FeatureTracking.StandaloneTest_Loop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
		}
#endif

		imshow( "m_imgSrcColor", ReplayLog1.m_imgSrcColor );
		
		if( g_States.ReplayVideo_On == 5 )	// for one-step forward replay
		{
			g_States.ReplayVideo_On = 1;	// then, stop
		}

		cv::waitKey(50);
	}

	g_States.ReplayVideo_On = 0;

	printf("Thread_ReplayVideo ended\n");
	
	return 1;
}



// data file
unsigned WINAPI Thread_ReplayData( void* arg )
{
	ClassDataVideoLog ReplayLog1;

	// data file
	char *cNameDataFile = GetDATfromFileDialog();
	if( cNameDataFile == NULL )
		return -1;

	if( ReplayLog1.ReplayData_Open( cNameDataFile ) != ReplayLog1.PROCESS_OK )
	{
		printf("ReplayData_Open was not initialized\n");
		return -1;
	}

	ReplayLog1.ReadDataProperty();

	Sleep(100);
	g_States.ReplayData_On = 10;

	while( g_States.ReplayData_On != 0 )
	{
		if( ReplayLog1.ReplayData_Loop() != ReplayLog1.PROCESS_OK )
		{
			printf("Replay data ended\n");
			break;
		}
		Sleep(50);
	}

	printf("Thread_ReplayData ended\n");
	
	return 1;
}

//////////////////////////////////////////////////

int ClassDataVideoLog::ReplayDataVideo_Open( int *iReplayDatVideo_On )
{
	(*iReplayDatVideo_On) = 10;
	m_nCountDatReading = 0;

	// data file
	char *cNameDataFile = GetDATfromFileDialog();
	if( cNameDataFile == NULL )
		return -1;

	ReplayData_Open( cNameDataFile );
	ReadDataProperty();

	// video file
	char cNameVideoFile[200];
	size_t nStrLength = strlen( cNameDataFile );
	strcpy( cNameVideoFile, cNameDataFile );
	cNameVideoFile[nStrLength-3] = 'a';
	cNameVideoFile[nStrLength-2] = 'v';
	cNameVideoFile[nStrLength-1] = 'i';
	
	ReplayVideo_Open( cNameVideoFile );

	return 1;
}


int ClassDataVideoLog::ReplayDataVideo_Loop( int *iReplayDatVideo_On )
{
	if( ReplayData_Loop() != PROCESS_OK )
	{
		printf("Replay data ended\n");
		return ERROR_FILE_READ;
	}

	if( ReplayVideo_Loop() != PROCESS_OK )
	{
		printf("Replay video ended\n");
		return ERROR_VIDEO_READ;
	}

	return PROCESS_OK;
}


int ClassDataVideoLog::ReplayDataVideo_Close()
{
	ReplayData_Close();
	ReplayVideo_Close();

	return 1;
}


int ClassDataVideoLog::ReplayData_Open( char* cNameDataFile )
{
	m_DataFile = fopen(cNameDataFile, "rb");	// ***** 반드시 "rb"로 read binary 해야함!! 이거 "r"이면 read text 모드임
	if( !m_DataFile ) 
	{
		printf("Dat loading failed.\n");
		return ERROR_FILE_OPEN;
	}
	printf("Dat loading started.\n");
	return PROCESS_OK;
}

// load data properties at the first section of the file 
int ClassDataVideoLog::ReadDataProperty()
{
	char cDataProperty[1000];

	fseek( m_DataFile, 0, SEEK_SET );

	fread( cDataProperty, 1000, 1, m_DataFile);

	printf( "%s", cDataProperty );

	for( int i=0; i<1000; i++ )
	{
		if( cDataProperty[i] == '#' )
		{
			if( strncmp( &(cDataProperty[i]), "#Laser:Hokuyo:UTM-30LX", 22) == 0 )	// 0: identical
			{	
				g_LaserParam1.SetLaserVendorParameter( "UTM-30LX", 23, 60000, 1440, 0, 1080, 540, 2400 );
				g_LaserParam1.SetLaserUserDefinedParameter( 0.25, -135.0, 135.0, 180, 900 );

				for(int i=g_LaserParam1.LeftEndStep; i<=g_LaserParam1.RightEndStep; i++)
				{
					g_LaserParam1.LUT_CosScanAngle[i] = (float)( cos(D2R*( g_LaserParam1.LeftEndAngle+((double)(i-g_LaserParam1.LeftEndStep)*g_LaserParam1.AngleResolution) )) );
					g_LaserParam1.LUT_SinScanAngle[i] = (float)( sin(D2R*( g_LaserParam1.LeftEndAngle+((double)(i-g_LaserParam1.LeftEndStep)*g_LaserParam1.AngleResolution) )) );
				}

				g_States.LRF_On = 1;
			}

			if( strncmp( &(cDataProperty[i]), "#Laser:Hokuyo:URG-04LX", 22) == 0 )	// 0: identical
			{	
				g_LaserParam1.SetLaserVendorParameter( "URG-04LX", 23, 60000, 1440, 0, 1080, 540, 2400 );
				g_LaserParam1.SetLaserUserDefinedParameter( 0.25, -135.0, 135.0, 180, 900 );

				for(int i=g_LaserParam1.LeftEndStep; i<=g_LaserParam1.RightEndStep; i++)
				{
					g_LaserParam1.LUT_CosScanAngle[i] = (float)( cos(D2R*( g_LaserParam1.LeftEndAngle+((double)(i-g_LaserParam1.LeftEndStep)*g_LaserParam1.AngleResolution) )) );
					g_LaserParam1.LUT_SinScanAngle[i] = (float)( sin(D2R*( g_LaserParam1.LeftEndAngle+((double)(i-g_LaserParam1.LeftEndStep)*g_LaserParam1.AngleResolution) )) );
				}

				g_States.LRF_On = 1;
			}

		}
	}


	if( fseek( m_DataFile, 1000, SEEK_SET ) )
		printf("fseek error  ");

	return PROCESS_OK;
}


int ClassDataVideoLog::ReplayData_Loop()
{
	if( fread(&m_StoredRobotStates, sizeof(StoredRobotStates_New), 1, m_DataFile) != 1 )
	{
		printf("fread error  ");
		return ERROR_FILE_READ;
	}

	//memcpy(g_dRecRobotPos, (&m_StoredRobotStates.RobotState[0]), 3*sizeof(double) );
	//memcpy(g_dRecRobotVel, m_StoredRobotStates.RobotVel, 3*sizeof(double) );
	//memcpy(g_dRecRobotAtt, m_StoredRobotStates.RobotAtt, 3*sizeof(double) );

	//for( int i=0; i<15; i++ )
	//	(double)fMeanRobot[i] = m_StoredRobotStates.RobotState[i];

	// reserved 9 * 8
	//for( int i=15; i<24; i++ )
	//	(double)fMeanRobot[i] = m_StoredRobotStates.RobotState[i];

	memcpy(g_TcpSensorData.IMUdata, m_StoredRobotStates.IMUdata, 12*sizeof(double) );
	memcpy(g_TcpSensorData.ServoState, m_StoredRobotStates.ServoState, 3*sizeof(double) );
	memcpy(g_sMotorControl, m_StoredRobotStates.MotorControl, 10*sizeof(short) );

	memcpy(g_TcpSensorData.LRFdata, m_StoredRobotStates.LRFdata, 1081*sizeof(short) );
	
	// reserved 10 * 4
	//for( int i=0; i<10; i++ )
	//	(int)(other states) = m_StoredRobotStates.OtherStates[i];

	if( fseek( m_DataFile, 1000+m_nCountDatReading*(long)(sizeof(StoredRobotStates_New)), SEEK_SET ) )
		printf("fseek error  ");

	m_nCountDatReading += 1;

	return PROCESS_OK;

}

void ClassDataVideoLog::ReplayData_Close()
{
	fclose( m_DataFile );
}


/////////////////////////////////////////////////
////////////// ReplayVideo ///////////
/////////////////////////////////////////////////

int ClassDataVideoLog::ReplayVideo_Open( char* cNameVideoFile )
{
	if( !m_capture.open( cNameVideoFile ) )
	{
		printf("Dat loading failed.\n");
		return ERROR_VIDEO_OPEN;
	}

	m_imgSrcColor.create( 480, 640, CV_8UC3 );
	m_imgSrcGray.create( 480, 640, CV_8UC1 );

	g_States.ReplayDataVideo_On = 10;	// 10 for continuous replay

	return PROCESS_OK;
}


int ClassDataVideoLog::ReplayVideo_Loop()
{
/*
	m_nCountVideoReading = (int)m_capture.get( CV_CAP_PROP_POS_FRAMES );
	//m_nCountVideoReading = (int)cvGetCaptureProperty( m_capture, CV_CAP_PROP_POS_FRAMES );
	m_nCountDatReading = m_nCountVideoReading+2;

	//printf("dat:%d frames:%d\n", m_nCountDatReading, m_nCountVideoReading );

	m_nCountDatReading++;


	if( (*iReplayDatVideo_On) == -10 )
	{
		//m_nCountDatReading -= 100;
		m_nCountVideoReading += 800;
		m_capture.set( CV_CAP_PROP_POS_FRAMES, m_nCountVideoReading );
		//cvSetCaptureProperty( m_capture, CV_CAP_PROP_POS_FRAMES, (double)m_nCountVideoReading );

		//m_nCountVideoReading = (int)cvGetCaptureProperty( m_capture, CV_CAP_PROP_POS_FRAMES );
		//m_nCountDatReading = m_nCountVideoReading+2;

		(*iReplayDatVideo_On) = 10;
	}
*/
	if( !(m_capture.read( m_imgSrcColor ) ) )
		return ERROR_VIDEO_READ;

	return PROCESS_OK;
}

void ClassDataVideoLog::ReplayVideo_Close()
{
	m_capture.release();
	m_imgSrcColor.release();
	m_imgSrcGray.release();
}




//--------------------------------------------------------
// replay data video legacy
//--------------------------------------------------------


// data+video
unsigned WINAPI Thread_ReplayDataVideo_Legacy( void* arg )
{

	ClassDataVideoLog ReplayLog1;

	if( ReplayLog1.ReplayDataVideo_Open_Legacy( &(g_States.ReplayDataVideoLegacy_On) ) != ReplayLog1.PROCESS_OK )
	{
		printf("ReplayDataVideo_Open_Legacy was not initialized\n");
		return -1;
	}

	//g_States.ReplayVideo_On = 1;
	namedWindow( "m_imgSrcColor" );

	F_SLAM4		Fslam4;

	HS2_LaserCamSLAM LaserCamSlam1;

	Sleep(100);

	while( g_States.ReplayDataVideo_On != 0 )
	{

		if( g_States.ReplayDataVideo_On == 1 )
		{
			cv::waitKey(100);
			continue;
		}

		if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_PredictionLoop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
		}


		if( ReplayLog1.ReplayDataVideo_Loop_Legacy( &(g_States.ReplayDataVideoLegacy_On) ) != ReplayLog1.PROCESS_OK )
			break;

		cv::cvtColor( ReplayLog1.m_imgSrcColor, ReplayLog1.m_imgSrcGray, CV_BGR2GRAY );


		// FSLAM
		if( g_States.FusionSLAM_On == 1 )
		{
			Fslam4.EKF_InitEKF( ReplayLog1.m_imgSrcGray );

			g_States.FusionSLAM_On = 2;
		}
		else if( g_States.FusionSLAM_On == 2 )
		{
			Fslam4.EKF_Loop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
		}


		if( g_States.LaserCamSLAM_On == g_States.On_Initialize )
		{
			LaserCamSlam1.Filter_Initial( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor );
			g_States.LaserCamSLAM_On = g_States.On_Loop;
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_CorrectionLoop( ReplayLog1.m_imgSrcGray, ReplayLog1.m_imgSrcColor, g_TcpSensorData, g_States, &g_LaserCamSlam );
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Terminate )
		{
			LaserCamSlam1.Filter_Terminate();
			g_States.LaserCamSLAM_On = g_States.Off;
		}


		imshow( "m_imgSrcColor", ReplayLog1.m_imgSrcColor );
		
		if( g_States.ReplayDataVideo_On == 5 )	// for one-step forward replay
		{
			g_States.ReplayDataVideo_On = 1;	// then, stop
		}

		cv::waitKey(50);
	}

	g_States.ReplayDataVideoLegacy_On = 0;

	printf("Thread_ReplayDataVideoLegacy ended\n");
	
	return 1;
}




int ClassDataVideoLog::ReplayDataVideo_Open_Legacy( int *iReplayDatVideo_On )
{
	(*iReplayDatVideo_On) = 10;
	m_nCountDatReading = 0;

	// data file
	char* cNameDataFile = GetDATfromFileDialog();
	m_DataFile = fopen(cNameDataFile, "rb");	// ***** 반드시 "rb"로 read binary 해야함!! 이거 "r"이면 read text 모드임
	if( !m_DataFile ) 
	{
		printf("Dat loading failed.\n");
		return -1;
	}
	printf("Dat loading started.\n");


	// video file
	m_imgSrcColor = cvCreateImage(cvSize(640,480),8,3);
	m_imgSrcGray = cvCreateImage(cvSize(640,480),8,1);
	
	// video file
	char cNameVideoFile[200];
	size_t nStrLength = strlen( cNameDataFile );
	strcpy( cNameVideoFile, cNameDataFile );
	cNameVideoFile[nStrLength-3] = 'a';
	cNameVideoFile[nStrLength-2] = 'v';
	cNameVideoFile[nStrLength-1] = 'i';
	ReplayVideo_Open( cNameVideoFile );

	// ----------- UTM-30LX ------------
	g_LaserParam1.NumTotalMeas = 1081;
	g_LaserParam1.LeftEndStep = 0;
	g_LaserParam1.RightEndStep = 1080;
	g_LaserParam1.CenterFrontStep = 540;

	g_LaserParam1.AngleResolution = 0.25;	// URG-04LX: 360/1024=0.3515625
	g_LaserParam1.LeftEndAngle = -135.0;
	g_LaserParam1.RightEndAngle = 135.0;

	g_LaserParam1.Left90DegStep = 180;
	g_LaserParam1.Right90DegStep = 900;

	for(int i=g_LaserParam1.LeftEndStep; i<=g_LaserParam1.RightEndStep; i++)
	{
		g_LaserParam1.LUT_CosScanAngle[i] = (float)( cos(D2R*( g_LaserParam1.LeftEndAngle+((double)(i-g_LaserParam1.LeftEndStep)*g_LaserParam1.AngleResolution) )) );
		g_LaserParam1.LUT_SinScanAngle[i] = (float)( sin(D2R*( g_LaserParam1.LeftEndAngle+((double)(i-g_LaserParam1.LeftEndStep)*g_LaserParam1.AngleResolution) )) );
	}
	// ----------- UTM-30LX ------------

	g_States.LRF_On = g_States.On_Replay;

	return 1;
}


int ClassDataVideoLog::ReplayDataVideo_Loop_Legacy( int *iReplayDatVideo_On )
{
	// get data from DAT

	if( fread(&m_StoredRobotStates_Legacy, sizeof(StoredRobotStates_Legacy), 1, m_DataFile) != 1 )
	{
		printf("fread error  ");
		return 0;
	}

	memcpy(&(g_LaserCamSlam.fMeanRobot[0]), m_StoredRobotStates_Legacy.RobotPos, 3*sizeof(double) );
	memcpy(&(g_LaserCamSlam.fMeanRobot[3]), m_StoredRobotStates_Legacy.RobotVel, 3*sizeof(double) );
	memcpy(&(g_LaserCamSlam.fMeanRobot[6]), m_StoredRobotStates_Legacy.RobotAtt, 3*sizeof(double) );
	memcpy(g_TcpSensorData.IMUdata, m_StoredRobotStates_Legacy.IMUdata, 12*sizeof(double) );
	memcpy(g_TcpSensorData.ServoState, m_StoredRobotStates_Legacy.ServoState, 3*sizeof(double) );
	memcpy(g_sMotorControl, m_StoredRobotStates_Legacy.MotorControl, 10*sizeof(short) );
	//memcpy(g_sImageProperty, m_StoredRobotStates_Legacy.ImageProperty, 3*sizeof(short) );
/*
	// 한번에 fread 하지 않고 받아야할만큼 받을때까지 여러번 나눠서 fread하는 방법
	int seek_result;
	long remaining = 1081;
	while( remaining != 0 )
	{
		Sleep(20);
		seek_result = fseek( m_DataFile, (nCount+1)*(long)(sizeof(H2003)) -remaining*(long)(sizeof(short)) , SEEK_SET );
		readLeng = fread(&(_h2003.LRFdata[1081-remaining]), sizeof(short), (size_t)(remaining), m_DataFile);
		remaining = (size_t)(remaining) - readLeng;
		printf("read%d  remain%d  seek[%d]", readLeng, remaining, seek_result );
	}
*/
	memcpy(g_TcpSensorData.LRFdata, m_StoredRobotStates_Legacy.LRFdata, 1081*sizeof(short) );

	if( fseek( m_DataFile, m_nCountDatReading*(long)(sizeof(StoredRobotStates_Legacy)), SEEK_SET ) )
		printf("fseek error  ");
	
	m_nCountVideoReading = (int)m_capture.get( CV_CAP_PROP_POS_FRAMES );
	//m_nCountVideoReading = (int)cvGetCaptureProperty( m_capture, CV_CAP_PROP_POS_FRAMES );
	m_nCountDatReading = m_nCountVideoReading+2;

	//printf("dat:%d frames:%d\n", m_nCountDatReading, m_nCountVideoReading );

	m_nCountDatReading++;


	if( (*iReplayDatVideo_On) == -10 )
	{
		//m_nCountDatReading -= 100;
		m_nCountVideoReading += 800;
		m_capture.set( CV_CAP_PROP_POS_FRAMES, m_nCountVideoReading );
		//cvSetCaptureProperty( m_capture, CV_CAP_PROP_POS_FRAMES, (double)m_nCountVideoReading );

		//m_nCountVideoReading = (int)cvGetCaptureProperty( m_capture, CV_CAP_PROP_POS_FRAMES );
		//m_nCountDatReading = m_nCountVideoReading+2;

		(*iReplayDatVideo_On) = 10;
	}


	if( !(ReplayVideo_Loop() ) )
	{
		printf("Replay video ended\n");
		return 0;
	}

	return 1;
}
