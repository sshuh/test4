// Sungsik Huh
// 2013.09.23
// 

//#include "stdafx.h"

#include "CamPointgrey.h"


// for FOCAL = 340
// 43.25deg --> 43*4+1 +1 +43*4+1 = 347 points
//unsigned short LRFpt2pixelnum[347];	// begin at LRFdata[367]~[713]

//unsigned char LRFpixelcolor[640][3];
//extern double g_dServoState[3];

/*
extern SharedStates g_States;
extern StructSensorsDownlink	g_TcpSensorData;

ClassCameraImage g_CameraImage1;


unsigned WINAPI Thread_CameraPointgrey_Example( void *arg )
{
	Cam_FFMV cam_ffmv1;

	ClassDataVideoLog log1;

	int *iParam = (int*)arg;
	if( cam_ffmv1.InitCamera(*iParam) != 1 )
	{
		printf( "FFMV camera was not connected\n" );
		return -1;
	}
	namedWindow( "m_imgSrcColor" );
	moveWindow( "m_imgSrcColor", 100, 500 );

	g_States.CAM_On = 1;

	F_SLAM4		Fslam4;


	Feature_Test test1;

	HS2_LaserCamSLAM LaserCamSlam1;

	Sleep(100);

	while( g_States.CAM_On == 1 )
	{
		if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_PredictionLoop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
		}

		cam_ffmv1.GrabImage();

		cv::cvtColor( cam_ffmv1.m_imgSrcColor, cam_ffmv1.m_imgSrcGray, CV_BGR2GRAY );

		// -------------------- Fusion SLAM --------------------
		if( g_States.FusionSLAM_On == 1 )
		{
			Fslam4.EKF_InitEKF( cam_ffmv1.m_imgSrcGray );
			g_States.FusionSLAM_On = 2;
		}
		else if( g_States.FusionSLAM_On == 2 )
		{
			Fslam4.EKF_Loop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
		}

		if( g_States.LaserCamSLAM_On == g_States.On_Initialize )
		{
			LaserCamSlam1.Filter_Initial( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			g_States.LaserCamSLAM_On = g_States.On_Loop;
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_CorrectionLoop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Terminate )
		{
			LaserCamSlam1.Filter_Terminate();
			g_States.LaserCamSLAM_On = g_States.Off;
		}

		imshow( "m_imgSrcColor", cam_ffmv1.m_imgSrcColor );

		// video server
		if( g_States.TcpCommVideoServer_On == 1 )
			cam_ffmv1.m_imgSrcColor.copyTo( g_CameraImage1.imgSrc );

		// --------------------Data Video Record--------------------
		log1.DataVideoLogging( cam_ffmv1.m_imgSrcColor );

		log1.VideoLogging( cam_ffmv1.m_imgSrcColor );



		// TEST
		if( g_States.Test == 1 )
		{
			test1.Initial_ForTest( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			g_States.Test = 2;
		}
		else if( g_States.Test == 2 )
		{
			test1.Loop_ForTest( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
		}

		cv::waitKey(30);
//		Sleep(50);
	}

	LaserCamSlam1.Filter_Terminate();

	cam_ffmv1.StopCamera();

	//g_States.CAM_On = 0;

	printf("FFMV camera disconnected\n");
	
	return 1;
}
*/


#if( PTGREY_CAM_LIB_VER == 2 )	// ---- FlyCapture 2.3 ----

int ClassCamPointgrey::InitiateCamera( int iParam )
{
	m_iCamSetting = iParam;	// 0:None, 1:Fixed, 2:Adaptive and fix
	m_uiTimestampUser = 0;

	FlyCapture2::FC2Version version;
	FlyCapture2::Utilities::GetLibraryVersion( &version );

	printf("Flycapture ver %d.%d.%d.%d\n", version.major, version.minor, version.type, version.build );
	
	m_Error = m_BusManager.GetCameraFromIndex(0, &m_Guid);
	if( m_Error != PGRERROR_OK )
		return -1;

	m_Error = m_Cam.Connect(&m_Guid);
	if( m_Error != PGRERROR_OK )
		return -1;

	m_Error = m_Cam.StartCapture();
	m_Error = m_Cam.RetrieveBuffer(&m_Image);

	SizeOfCamImage.width = m_Image.GetCols();
	SizeOfCamImage.height = m_Image.GetRows();

	m_imgSrcColor.create( SizeOfCamImage, CV_8UC3 );
	m_imgSrcGray.create( SizeOfCamImage, CV_8UC1 );

	return 1;
}


int ClassCamPointgrey::GrabImage()	// ---- FlyCapture 2.3 ----
{
	SetCameraParameter();

	m_Error = m_Cam.RetrieveBuffer(&m_Image);
	m_Error = m_Image.Convert(PIXEL_FORMAT_BGR, &m_ImageColor);// convert to rgb type
	memcpy( m_imgSrcColor.data, m_ImageColor.GetData(), m_ImageColor.GetDataSize() );

	m_stTimeStamp = m_Image.GetTimeStamp();
	m_uiTimestamp = (unsigned int)(m_stTimeStamp.seconds*1000) + (unsigned int)(m_stTimeStamp.microSeconds*0.001);
	//printf("%d\n", m_uiTimestamp);
	return 1;
}


void ClassCamPointgrey::StopCamera()	// ---- FlyCapture 2.3 ----
{
	m_Error = m_Cam.StopCapture();
	m_Error = m_Cam.Disconnect();

}


void ClassCamPointgrey::SetCameraParameter()
{
	// m_iCamSetting = 0:None, 1:Fixed, 2:Adaptive fixed, 3:
	switch( m_iCamSetting )
	{
	case 0:
		{

			break;
		}
	case 1:
		{
			if( m_uiTimestampUser == 10 )
			{
				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.absValue = 0.06f;
				pCamProperty.autoManualMode = false;
				//pCamProperty.onOff = false;
				//pCamProperty.onePush = false;
				//pCamProperty.absControl = false;
				//pCamProperty.valueA = 1;
				m_Cam.SetProperty( &pCamProperty );
			}

			if( m_uiTimestampUser == 50 )
			{
				FlyCapture2::Property pCamProperty( FRAME_RATE );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.absValue = 5.00f;
				pCamProperty.autoManualMode = false;
				pCamProperty.onOff = false;
				//pCamProperty.onePush = false;
				pCamProperty.absControl = true;
				//pCamProperty.valueA = 1;
				m_Cam.SetProperty( &pCamProperty );
			}

			if( m_uiTimestampUser++ == 100 )
			{

				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Error = m_Cam.GetProperty( &pCamProperty );
				pCamProperty.absValue = 0.06f;
				pCamProperty.autoManualMode = false;
				//pCamProperty.onOff = false;
				//pCamProperty.onePush = false;
				//pCamProperty.absControl = false;
				//pCamProperty.valueA = 1;
				m_Error = m_Cam.SetProperty( &pCamProperty );
				Sleep(100);
				m_Error = m_Cam.GetProperty( &pCamProperty );
				Sleep(100);

				//pCamProperty.absValue = 0.096f;
				//pCamProperty.autoManualMode = false;
				//pCamProperty.onOff = false;
				
				//m_Cam.SetProperty( &pCamProperty );
				//Sleep(100);
				//m_Cam.GetProperty( &pCamProperty );
				printf("Cam setting: Fixed(shutter=%.3f)\n", pCamProperty.absValue );
				m_iCamSetting = 0;
			}
			break;
		}
	case 2:
		{
			if( m_uiTimestampUser == 10 )
			{
				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.autoManualMode = true;
				m_Cam.SetProperty( &pCamProperty );
			}

			if( m_uiTimestampUser++ == 100 )
			{
				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.autoManualMode = false;
				m_Cam.SetProperty( &pCamProperty );
				m_Cam.GetProperty( &pCamProperty );
				printf("Cam setting: Adaptive fixed(shutter=%.3f)\n", pCamProperty.absValue );
				m_iCamSetting = 0;
			}

			break;
		}
	}

}

#endif



#if( PTGREY_CAM_LIB_VER == 1 )
int ClassCamPointgrey::InitiateCamera()
{
	// ---- FlyCapture 1.8 ----
	error = flycaptureCreateContext(&pContext);
	error = flycaptureInitialize(pContext, 0);

	if( error != FLYCAPTURE_OK )
		return 0;
 
	pImageBGR.pixelFormat = FLYCAPTURE_BGR;
	pImageBGR.pData = new unsigned char[2248*2048*3];

	float fValue;
	bool bOnePush=0, bOnOff=0, bAuto=0;
	//error = flycaptureGetCameraAbsPropertyEx( pContext, FLYCAPTURE_AUTO_EXPOSURE, &bOnePush, &bOnOff, &bAuto, &fValue );
	//fprintf( stderr, "error=%d   auto expo: onepush=%d  onoff=%d  auto=%d   %f\n", error, bOnePush, bOnOff, bAuto, fValue );
  
	bOnOff = 0, bAuto = 0;
	fValue = 15.f;
	error = flycaptureSetCameraAbsPropertyEx( pContext, FLYCAPTURE_SHUTTER, bOnePush, bOnOff, bAuto, fValue );

	error = flycaptureGetCameraAbsPropertyEx( pContext, FLYCAPTURE_SHUTTER, &bOnePush, &bOnOff, &bAuto, &fValue );
	fprintf( stdout, "error=%d   shutter: onepush=%d  onoff=%d  auto=%d   %f\n", error, bOnePush, bOnOff, bAuto, fValue );

	error = flycaptureStart(pContext,FLYCAPTURE_VIDEOMODE_ANY,FLYCAPTURE_FRAMERATE_ANY); 
	//error = flycaptureStart(pContext, FLYCAPTURE_VIDEOMODE_ANY, FLYCAPTURE_FRAMERATE_30); 
	//error=flycaptureStartCustomImage(pContext,3,0,0,width,height,100.0f,FLYCAPTURE_RAW16);
	//error=flycaptureStartCustomImage(pContext,3,0,0,640,480,100.0f,FLYCAPTURE_RAW16);
	fprintf( stdout, "error=%d\n", error );
	// ------


	m_imgSrcColor.create( 480, 640, CV_8UC3 );
	m_imgSrcGray.create( 480, 640, CV_8UC1 );

//	m_iVideoWriterOn = 0;

	return 1;
}


void ClassCamPointgrey::StopCamera()
{
	// ---- FlyCapture 1.8 ----
	flycaptureStop(pContext);
	flycaptureDestroyContext(pContext);
	delete[] pImageBGR.pData;

//	if(	m_iVideoWriterOn )
//		m_videoWriter.release();

}


int ClassCamPointgrey::GrabImage()
{
	// ---- FlyCapture 1.8 ----
	error = flycaptureGrabImage2( pContext, (FlyCaptureImage*)&pImage );
	error = flycaptureConvertImage( pContext, (FlyCaptureImage*)&pImage, (FlyCaptureImage*)&pImageBGR );
	memcpy(m_imgSrcColor->imageDataOrigin, pImageBGR.pData, pImageBGR.iRowInc*pImageBGR.iRows);// OpenCV IplImage¿¡ image data º¹»ç
	return 1;
}



void ClassCamPointgrey::CamSetting()
{
//	float fSetValue=0.0f, fGetValue=0.0f;
//	bool bOnePush=false, bOnOff=false, bAuto=false;
	//error = flycaptureGetCameraAbsPropertyEx( pContext, FLYCAPTURE_AUTO_EXPOSURE, &bOnePush, &bOnOff, &bAuto, &fValue );
	//fprintf( stderr, "error=%d   auto expo: onepush=%d  onoff=%d  auto=%d   %f\n", error, bOnePush, bOnOff, bAuto, fValue );

	m_Shutter.error = flycaptureGetCameraAbsPropertyEx( pContext, FLYCAPTURE_SHUTTER, &m_Shutter.bGetOnePush, &m_Shutter.bGetOnOff, &m_Shutter.bGetAuto, &m_Shutter.fGetValue );
	fprintf( stdout, "Current (SHUTTER): error=%d  get=%.3f  onepush=%d  onoff=%d  auto=%d\n"
						, m_Shutter.error, m_Shutter.fGetValue, m_Shutter.bGetOnePush, m_Shutter.bGetOnOff, m_Shutter.bGetAuto );


	m_Shutter.bSetAuto = false;		m_Shutter.bSetOnePush = false;	m_Shutter.bSetOnOff = true;
	m_Shutter.fSetValue = m_Shutter.fGetValue;//0.096;
	fprintf( stdout, "New Setting(SHUTTER): error=%d  set=%.3f  onepush=%d  onoff=%d  auto=%d\n"
						, m_Shutter.error, m_Shutter.fSetValue, m_Shutter.bSetOnePush, m_Shutter.bSetOnOff, m_Shutter.bSetAuto );
	m_Shutter.error = flycaptureSetCameraAbsPropertyEx( pContext, FLYCAPTURE_SHUTTER, m_Shutter.bSetOnePush, m_Shutter.bSetOnOff, m_Shutter.bSetAuto, m_Shutter.fSetValue );


	m_Shutter.error = flycaptureGetCameraAbsPropertyEx( pContext, FLYCAPTURE_SHUTTER, &m_Shutter.bGetOnePush, &m_Shutter.bGetOnOff, &m_Shutter.bGetAuto, &m_Shutter.fGetValue );
	fprintf( stdout, "Current (SHUTTER): error=%d  get=%.3f  onepush=%d  onoff=%d  auto=%d\n"
						, m_Shutter.error, m_Shutter.fGetValue, m_Shutter.bGetOnePush, m_Shutter.bGetOnOff, m_Shutter.bGetAuto );

	
/*
	fSetValue = 0.0f; bOnOff = false; bAuto = false;
	error = flycaptureSetCameraAbsPropertyEx( pContext, FLYCAPTURE_GAIN, bOnePush, bOnOff, bAuto, fSetValue );
	error2 = flycaptureGetCameraAbsPropertyEx( pContext, FLYCAPTURE_GAIN, &bOnePush, &bOnOff, &bAuto, &fGetValue );
	fprintf( stdout, "Eset=%d Eget=%d  GAIN set=%.3f  get=%.3f  onepush=%d  onoff=%d  auto=%d\n", error, error2, fSetValue, fGetValue, bOnePush, bOnOff, bAuto );

	fSetValue = 30.0f; bOnOff = true; bAuto = false;
	error = flycaptureSetCameraAbsPropertyEx( pContext, FLYCAPTURE_FRAME_RATE, bOnePush, bOnOff, bAuto, fSetValue );
	error2 = flycaptureGetCameraAbsPropertyEx( pContext, FLYCAPTURE_FRAME_RATE, &bOnePush, &bOnOff, &bAuto, &fGetValue );
	fprintf( stdout, "Eset=%d Eget=%d  FRAME_RATE set=%.3f  get=%.3f  onepush=%d  onoff=%d  auto=%d\n", error, error2, fSetValue, fGetValue, bOnePush, bOnOff, bAuto );
*/


}

#endif




/*
int Cam_FFMV::InitialCameraParameters()
{
	//m_imgUndistorted = cvCreateImage(cvSize(640,480),8,3);

//	m_matCamParam = ( Mat_ <double> (3,3) << CAM_FOCALL, 0.0, CAM_PP_HOR,   0.0, CAM_FOCALL, CAM_PP_VER,   0.0, 0.0, 1.0 );
//	m_matCamDistCoeff = ( Mat_ <double> (4,1) << CAM_RDIST_1,  CAM_RDIST_2,  CAM_RDIST_3,  CAM_RDIST_4 );

//	m_imgMap1 = cvCreateMat( m_imgSrcColor->height, m_imgSrcColor->width, CV_32FC1 );
//	m_imgMap2 = cvCreateMat( m_imgSrcColor->height, m_imgSrcColor->width, CV_32FC1 );

//	Mat matR;
//	initUndistortRectifyMap( m_matCamParam, m_matCamDistCoeff, matR, m_matCamParam, VideoSize, CV_32FC1, m_imgMap1, m_imgMap2 );

	return 1;
}


int Cam_FFMV::LoopCameraParameters()
{
//	cvUndistort2( m_imgSrc, m_imgUndistorted, mat_CamIntrinsic, mat_CamDistCoeff );
//	cvRemap( m_imgSrc, m_imgUndistorted, m_imgMapX, m_imgMapY );

	return 1;
}


int Cam_FFMV::TerminateCameraParameters()
{
	//m_imgMap1.release();
	//m_imgMap2.release();
	return 1;
}
*/


/*
int Thread_FFMVCamera()
{

	Cam_FFMV cam_ffmv1;

	if( !(cam_ffmv1.InitCamera()) )
	{
		fprintf( stdout, "FFMV camera was not connected\n" );
		return -1;
	}
	namedWindow( "m_imgSrcColor" );
	moveWindow( "m_imgSrcColor", 100, 500 );

	g_States.CAM_On = 1;


	#if( FSLAM_VERSION == 1 )
		F_SLAM		Fslam1;
	#endif
	#if( FSLAM_VERSION == 2 )
		F_SLAM2		Fslam2;
	#endif
	#if( FSLAM_VERSION == 3 )
		F_SLAM3		Fslam3;
	#endif
	#if( FSLAM_VERSION == 4 )
		F_SLAM4		Fslam4;
	#endif

	Feature_Test test1;

	HS2_LaserCamSLAM LaserCamSlam1;

	Sleep(100);

	while( 1 )
	{
		if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_PredictionLoop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
		}

		cam_ffmv1.GrabLoop();

		cv::cvtColor( cam_ffmv1.m_imgSrcColor, cam_ffmv1.m_imgSrcGray, CV_BGR2GRAY );
		//cvCvtColor( cam_ffmv1.m_imgSrcColor, cam_ffmv1.m_imgSrcGray, CV_BGR2GRAY );
		//cvRemap( m_img1Gray, m_img2, cam_ffmv1.m_imgMapX, cam_ffmv1.m_imgMapY );

		// -------------------- Fusion SLAM --------------------
		if( g_States.FusionSLAM_On == 1 )
		{
			#if( FSLAM_VERSION == 1 )
				Fslam1.EKF_InitEKF( cam_ffmv1.m_imgSrcGray );
			#endif
			#if( FSLAM_VERSION == 2 )
				Fslam2.EKF_InitEKF( cam_ffmv1.m_imgSrcGray );
			#endif
			#if( FSLAM_VERSION == 3 )
				Fslam3.EKF_InitEKF( cam_ffmv1.m_imgSrcGray );
			#endif
			#if( FSLAM_VERSION == 4 )
				Fslam4.EKF_InitEKF( cam_ffmv1.m_imgSrcGray );
			#endif

			g_States.FusionSLAM_On = 2;
		}
		else if( g_States.FusionSLAM_On == 2 )
		{
			#if( FSLAM_VERSION == 1 )
				Fslam1.EKF_Loop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			#endif
			#if( FSLAM_VERSION == 2 )
				Fslam2.EKF_Loop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			#endif
			#if( FSLAM_VERSION == 3 )
				Fslam3.EKF_Loop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			#endif
			#if( FSLAM_VERSION == 4 )
				Fslam4.EKF_Loop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			#endif	
		}


		if( g_States.LaserCamSLAM_On == g_States.On_Initialize )
		{
			LaserCamSlam1.Filter_Initial( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			g_States.LaserCamSLAM_On = g_States.On_Loop;
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Loop )
		{
			LaserCamSlam1.Filter_CorrectionLoop( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
		}
		else if( g_States.LaserCamSLAM_On == g_States.On_Terminate )
		{
			LaserCamSlam1.Filter_Terminate();
			g_States.LaserCamSLAM_On = g_States.Off;
		}


		imshow( "m_imgSrcColor", cam_ffmv1.m_imgSrcColor );



		// --------------------DatVideo Record--------------------
		if( g_States.DatVideoRecording_On == 1 )	// DatVideo Save Started
		{
			CTime today = CTime::GetCurrentTime();
			char output_file[80];
			sprintf_s( output_file, sizeof(output_file), "hs2log_%02d%02d%02d_%02dh%02dm%02ds.dat", 
						today.GetYear()-2000, today.GetMonth(), today.GetDay(), today.GetHour(), today.GetMinute(), today.GetSecond() );
			cam_ffmv1.m_DataFile = fopen(output_file, "wb");
			if( !cam_ffmv1.m_DataFile ) 
				return -1;

			cam_ffmv1.RecordDataProperty( output_file );

			cam_ffmv1.RecordVideo_Initial(output_file);
			g_States.DatVideoRecording_On = 2;
		}
		else if( g_States.DatVideoRecording_On == 2 )	// DatVideo Saving
		{
			cam_ffmv1.RecordVideo_Loop(cam_ffmv1.m_imgSrcColor);
		}
		else if( g_States.DatVideoRecording_On == 3 )	// DatVideo End
		{
			fclose(cam_ffmv1.m_DataFile);
			cam_ffmv1.RecordVideo_Terminate();
			g_States.DatVideoRecording_On = 0;
		}

		// TEST
		if( g_States.Test == 1 )
		{
			test1.Initial_ForTest( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
			g_States.Test = 2;
		}
		else if( g_States.Test == 2 )
		{
			test1.Loop_ForTest( cam_ffmv1.m_imgSrcGray, cam_ffmv1.m_imgSrcColor );
		}

		cvWaitKey(30);

//		Sleep(50);
	}

	LaserCamSlam1.Filter_Terminate();

	cam_ffmv1.StopCamera();

	g_States.CAM_On = 0;


	fprintf( stdout, "FFMV camera disconnected\n" );
	
	return 1;
}
*/