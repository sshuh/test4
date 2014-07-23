// Sungsik Huh
// 2013.09.23
// 

//#include "stdafx.h"
#include "RemoteComm.h"

/*
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

#include "CamPointgrey.h"

extern SharedStates g_States;

extern StructCommandUplink		g_TcpCommand;
extern StructSensorsDownlink	g_TcpSensorData;

extern StructMobileUplink		g_TcpCommand_Mobile;
extern StructMobileDownlink		g_TcpSensorData_Mobile;


extern ClassCameraImage g_CameraImage1;


extern TcpComm tcpMobile;
void Thread_TcpClientSend_Mobile(int maxThreads);
void Thread_TcpClientRecv_Mobile(int maxThreads);
*/


// -------------------------------- ClassCommClient_Bearnav -------------------------------------

unsigned short ClassCommClient_Bearnav::ComputeBodyChecksum(unsigned short *ptbody, int WordCount)
{
	short CompDataChecksum=0;
	for (int count=0; count<WordCount;count++)
		CompDataChecksum+=ptbody[count];
	return -CompDataChecksum;
}

int ClassCommClient_Bearnav::Initiate( char* cIpAddress, unsigned short usPort )
{
	tcpClient.SetSocketIPandPort( cIpAddress, usPort );
	return tcpClient.ConnectToTcpServer( tcpClient.m_cIpAddress, tcpClient.m_usPort );
}

int ClassCommClient_Bearnav::Terminate()
{
	tcpClient.CloseTcpConnection_Client();
	printf("ClassCommClient_Bearnav Terminated.\n");
	return 1;
}


// for MCL3d
int ClassCommClient_Bearnav::Loop_Send(SharedStates *stStates, MCL3dStates *stMcl3dStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav, PCK_COMMAND_BEARNAV *stTcpCommand_Bearnav )
{
	stTcpCommand_Bearnav->SyncWord = 0x81ff;
	stTcpCommand_Bearnav->TimeTag = 0.001*(double)(GetTickCount());	// get system time since it started.
	stTcpCommand_Bearnav->sFlagHS2 = 0;

	int nResult;
	while( stStates->TcpCommBearnav_On > stStates->Off )
	{
		stTcpCommand_Bearnav->TimeTag = 0.001*(double)(GetTickCount());	// get system time since it started.

		// NED to Geodetic : simple version
		// lon(W-E) 0.001 deg = 89.7448 m,   1 m = 0.00001114271 deg --> *10^7 --> 111.4271	dSlamPos[m]--> deg
		// lat(S-N) 0.001 deg = 110.9671 m,  1 m = 0.00000901168 deg --> *10^7 --> 90.1168
		stTcpCommand_Bearnav->llhSlam[0] = (36.367318 + (stMcl3dStates->dMeanPos[0]*0.00000901168))*D2R;	// lat(S-N)		dSlamMean[0]:X-axis 
		stTcpCommand_Bearnav->llhSlam[1] = (127.363566 + (stMcl3dStates->dMeanPos[1]*0.00001114271))*D2R;	// lon(W-E)		dSlamMean[1]:Y-axis
		stTcpCommand_Bearnav->llhSlam[2] = 75.0 + (-stMcl3dStates->dMeanPos[2]);	// height		dSlamMean[2]: -Z-axis 

		stTcpCommand_Bearnav->velSlam[0] = (float)(stMcl3dStates->dMeanVel[0]);	// [m/s]
		stTcpCommand_Bearnav->velSlam[1] = (float)(stMcl3dStates->dMeanVel[1]);
		stTcpCommand_Bearnav->velSlam[2] = (float)(-stMcl3dStates->dMeanVel[2]);

		// [0]hor-x, [1]hor-y, [2]vert, [3]vel
		stTcpCommand_Bearnav->stddevSlam[0] = 2.0f*(float)sqrt(stMcl3dStates->dCovPos[0]);//*g_dProcRobotPosCov[0]+g_dProcRobotPosCov[1]*g_dProcRobotPosCov[1]) );
		stTcpCommand_Bearnav->stddevSlam[1] = 2.0f*(float)sqrt(stMcl3dStates->dCovPos[1]);//g_TcpCommand_Bearnav.covSlam[0];
		stTcpCommand_Bearnav->stddevSlam[2] = 2.0f*(float)sqrt(stMcl3dStates->dCovPos[2]);
		stTcpCommand_Bearnav->stddevSlam[3] = 1.f;

		//g_TcpCommand_Bearnav.sFlagHS2 = 0;

		if( stMcl3dStates->flag == 1 )
		{
			stTcpCommand_Bearnav->Flag = 1;
			stMcl3dStates->flag = 0;

			// if stTcpSensorData_Bearnav->lcc provides wrong values
			if( (abs(stTcpSensorData_Bearnav->lcc[0])>10.f) || (abs(stTcpSensorData_Bearnav->lcc[1])>10.f) )
			{
				printf("LCC_OUT ");
				for( int i=0; i<3; i++ )
				{
					stMcl3dStates->fLccPosPrev[i] = stMcl3dStates->fLccPosCurr[i];
					stMcl3dStates->fLccPosCurr[i] = 0.f;
				}		
			}
			else
			{
				for( int i=0; i<3; i++ )
				{
					stMcl3dStates->fLccPosPrev[i] = stMcl3dStates->fLccPosCurr[i];
					stMcl3dStates->fLccPosCurr[i] = stTcpSensorData_Bearnav->lcc[i];
				}
			}
		}
		else
		{
			stTcpCommand_Bearnav->Flag = 0;
		}

		stTcpCommand_Bearnav->Checksum = ComputeBodyChecksum( (unsigned short*)(stTcpCommand_Bearnav), sizeof(PCK_COMMAND_BEARNAV)-2 );

		nResult = tcpClient.SendDataToServer( (char*)(stTcpCommand_Bearnav), sizeof(PCK_COMMAND_BEARNAV) );
		Sleep(20);
		if( nResult == -1 )	// check disconnection
		{

		}
	}
	return 1;
}


// for general use
int ClassCommClient_Bearnav::Loop_Send(SharedStates *stStates, EstimatedVehiclePoseStates *stEstPoseStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav, PCK_COMMAND_BEARNAV *stTcpCommand_Bearnav )
{
	stTcpCommand_Bearnav->SyncWord = 0x81ff;
	stTcpCommand_Bearnav->TimeTag = 0.0;
	stTcpCommand_Bearnav->sFlagHS2 = 0;

	int nResult;
	while( stStates->TcpCommBearnav_On > stStates->Off )
	{
		stTcpCommand_Bearnav->TimeTag += 1.0;

		// NED to Geodetic : simple version
		// lon(W-E) 0.001 deg = 89.7448 m,   1 m = 0.00001114271 deg --> *10^7 --> 111.4271	dSlamPos[m]--> deg
		// lat(S-N) 0.001 deg = 110.9671 m,  1 m = 0.00000901168 deg --> *10^7 --> 90.1168
		stTcpCommand_Bearnav->llhSlam[0] = (36.367318 + (stEstPoseStates->dMeanPos[0]*0.00000901168))*D2R;	// lat(S-N)		dSlamMean[0]:X-axis 
		stTcpCommand_Bearnav->llhSlam[1] = (127.363566 + (stEstPoseStates->dMeanPos[1]*0.00001114271))*D2R;	// lon(W-E)		dSlamMean[1]:Y-axis
		stTcpCommand_Bearnav->llhSlam[2] = 75.0 + (-stEstPoseStates->dMeanPos[2]);	// height		dSlamMean[2]: -Z-axis 

		stTcpCommand_Bearnav->velSlam[0] = (float)(stEstPoseStates->dMeanVel[0]);	// [m/s]
		stTcpCommand_Bearnav->velSlam[1] = (float)(stEstPoseStates->dMeanVel[1]);
		stTcpCommand_Bearnav->velSlam[2] = (float)(-stEstPoseStates->dMeanVel[2]);

		// [0]hor-x, [1]hor-y, [2]vert, [3]vel
		stTcpCommand_Bearnav->stddevSlam[0] = 2.0f*(float)sqrt(stEstPoseStates->dCovPos[0]);//*g_dProcRobotPosCov[0]+g_dProcRobotPosCov[1]*g_dProcRobotPosCov[1]) );
		stTcpCommand_Bearnav->stddevSlam[1] = 2.0f*(float)sqrt(stEstPoseStates->dCovPos[1]);//g_TcpCommand_Bearnav.covSlam[0];
		stTcpCommand_Bearnav->stddevSlam[2] = 2.0f*(float)sqrt(stEstPoseStates->dCovPos[2]);
		stTcpCommand_Bearnav->stddevSlam[3] = 1.f;

		//g_TcpCommand_Bearnav.sFlagHS2 = 0;

		if( stEstPoseStates->flag == 1 )
		{
			stTcpCommand_Bearnav->Flag = 1;
			stEstPoseStates->flag = 0;

			// if stTcpSensorData_Bearnav->lcc provides wrong values
			if( (abs(stTcpSensorData_Bearnav->lcc[0])>10.f) || (abs(stTcpSensorData_Bearnav->lcc[1])>10.f) )
			{
				printf("LCC_OUT ");
				for( int i=0; i<3; i++ )
				{
					stEstPoseStates->fLccPosPrev[i] = stEstPoseStates->fLccPosCurr[i];
					stEstPoseStates->fLccPosCurr[i] = 0.f;
				}		
			}
			else
			{
				for( int i=0; i<3; i++ )
				{
					stEstPoseStates->fLccPosPrev[i] = stEstPoseStates->fLccPosCurr[i];
					stEstPoseStates->fLccPosCurr[i] = stTcpSensorData_Bearnav->lcc[i];
				}
			}
		}
		else
		{
			stTcpCommand_Bearnav->Flag = 0;
		}

		stTcpCommand_Bearnav->Checksum = ComputeBodyChecksum( (unsigned short*)(stTcpCommand_Bearnav), sizeof(PCK_COMMAND_BEARNAV)-2 );

		nResult = tcpClient.SendDataToServer( (char*)(stTcpCommand_Bearnav), sizeof(PCK_COMMAND_BEARNAV) );
		Sleep(20);
		if( nResult == -1 )	// check disconnection
		{

		}
	}
	return 1;
}


int ClassCommClient_Bearnav::Loop_Recv( SharedStates *stStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav )
{
	int nResult;
	while( stStates->TcpCommBearnav_On > stStates->Off )
	{
		nResult = CheckBearnavDataM6002( stStates, stTcpSensorData_Bearnav );
		Sleep(1);		
		if( nResult == -1 )	// check disconnection
		{
			printf("Network disconnected: %d\n", nResult );
			Sleep(100);
		}
	}
	return 1;
}

int ClassCommClient_Bearnav::CheckBearnavDataM6002( SharedStates *stStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav )
{
	int nResult;
	char cBuf[4];
	int iCheckSyncWord = 0;

	while( 1 )
	{
		nResult = tcpClient.RecvDataFromServer( &cBuf[0], 1 );
		if( cBuf[0] == -1 )	// 0xff
		{
			nResult = tcpClient.RecvDataFromServer( &cBuf[1], 1 );
			if( cBuf[1] == -127 )	// 0x81
			{
				memcpy( &(stTcpSensorData_Bearnav->SyncWord), cBuf, 2 );
				break;
			}
			iCheckSyncWord++;
		}

		if( nResult < 1 )	// network disconnected
			return -1;
	}

	if( iCheckSyncWord > 0 )
		printf("SyncWordError: %d times ", iCheckSyncWord);

	nResult = tcpClient.RecvDataFromServer( (char*)(&stTcpSensorData_Bearnav->MessageID), (int)(sizeof(PCK_SENSORDATA_BEARNAV)-2) );
	if( nResult < 1 )
		return -1;

	if( stStates->TcpCommBearnav_On == stStates->PartiallyActivated )
		stStates->TcpCommBearnav_On = stStates->FullyActivated;

	return 1;
}




// ----------------------------------------------------------------------------------

int ClassRemoteCommServer::Initiate( unsigned short usPort )
{
	tcpServerSend.SetTcpServer( usPort );
	tcpServerSend.WaitForTcpClient();
	return 1;
}


int ClassRemoteCommServer::Terminate()
{
	tcpServerSend.CloseTcpServer();
	return 1;
}

int ClassRemoteCommServer::Loop( StructSensorsDownlink *stSensorData1, StructLaserdataDownlink *stLaserData1, int RemoteComm_States_On )
{
	unsigned int iTimestamp_Laser1 = 0;
	int nResult;

	stSensorData1->Header.ID[0] = 'H';
	stSensorData1->Header.ID[1] = 'S';
	stSensorData1->Header.ID[2] = '2';
	stSensorData1->Header.ID[3] = 'S';
	stSensorData1->Header.Length = (unsigned int)(sizeof(StructSensorsDownlink) -sizeof(StructCommHeader));

	stLaserData1->Header.ID[0] = 'H';
	stLaserData1->Header.ID[1] = 'S';
	stLaserData1->Header.ID[2] = '2';
	stLaserData1->Header.ID[3] = 'L';
	stLaserData1->Header.Length = (unsigned int)(sizeof(StructLaserdataDownlink) -sizeof(StructCommHeader));

	while( RemoteComm_States_On > 0 )
	{
		// sending sensors
		nResult = tcpServerSend.SendDataToClient( (char*)(stSensorData1), sizeof(StructSensorsDownlink) );
		printf("sent sensors(%d)\n", nResult );
		Sleep(10);


		// sending laser
		if( stLaserData1->Timestamp != iTimestamp_Laser1 )
			nResult = tcpServerSend.SendDataToClient( (char*)(stLaserData1), sizeof(StructLaserdataDownlink) );

		iTimestamp_Laser1 = stLaserData1->Timestamp;
		printf("sent laser(%d) time %d\n", nResult, stLaserData1->Timestamp );
		Sleep(10);
	}

	return 1;
}


int ClassRemoteCommServer::Loop( cv::Mat imgSrc, int RemoteComm_Video_On )
{
//	HS2_TcpipComm tcpVidSend;
//	tcpVidSend.SetTcpServer( TCPPORT_VIDEO_DOWNLINK );	// ~ listen
//	tcpVidSend.WaitForTcpClient();	// accept

	tcpServerSend.SetSendSocketBuffer( tcpServerSend.m_hServSock, 65535 );
	// ---
	
	//Mat imgSrc;
	cv::Mat imgSrc2;
	cv::namedWindow("imgSrc_Server");

	std::vector <int> compression_params;
	compression_params.push_back( CV_IMWRITE_JPEG_QUALITY );
	compression_params.push_back( 95 );

	int nResult;
	while( RemoteComm_Video_On > 0 )
	{
		cv::resize( imgSrc, imgSrc2, cv::Size(320,240) );
		//resize( imgSrc, imgSrc2, Size(320,240) );
		//imgSrc.copyTo( imgSrc2 );

		std::vector <uchar> buf;
		bool bResult = cv::imencode(".jpeg", imgSrc2, buf, compression_params );

		cv::imshow( "imgSrc_Server", imgSrc2 );

		//----
		nResult = tcpServerSend.SendLargeDataToClient( (char*)(&buf[0]), buf.size(), 3000);
		printf("Sent Video");

		// if client is disconnected
		if( nResult <= 0  )
		{
			Sleep(50);
			printf("ExternalVision : Waiting to Reconnect...\n");
			tcpServerSend.WaitForTcpClient();
			printf("ExternalVision : ReConnectTcpSocket_Server\n", nResult );
		}

		if( cv::waitKey( 30 ) == 27 )
			break;
	}

//	tcpServerSend.CloseTcpServer();
	imgSrc2.release();
	return 1;
}

// ----------------------------------------------------------------------------------

int ClassRemoteCommClient::Initiate( char *cIpAddress, unsigned short usPort )
{
	tcpClientRecv.SetSocketIPandPort( cIpAddress, usPort );
	tcpClientRecv.ConnectToTcpServer( tcpClientRecv.m_cIpAddress, tcpClientRecv.m_usPort );
	return 1;
}

int ClassRemoteCommClient::Terminate()
{
	tcpClientRecv.CloseTcpConnection_Client();
	return 1;
}

int ClassRemoteCommClient::Loop( StructSensorsDownlink *stSensorData1, StructLaserdataDownlink *stLaserData1, int RemoteComm_States_On )
{
	int nResult;
	StructCommHeader stHeaderTemp;

	while( RemoteComm_States_On > 0 )
	{
		// receiving header
		nResult = tcpClientRecv.RecvDataFromServer( (char*)(&stHeaderTemp), sizeof(StructCommHeader) );
		Sleep(1);

		if( strncmp(stHeaderTemp.ID, "HS2S", 4) == 0 )	// 0: identical
		{
			memcpy( &(stSensorData1->Header), &stHeaderTemp, sizeof(StructCommHeader) );
			// receiving sensors
			nResult = tcpClientRecv.RecvDataFromServer( (char*)(&(stSensorData1->Timestamp)), (int)(stSensorData1->Header.Length) );
			printf("recv sensors(%d) ", nResult);
			Sleep(1);
		}
		else if( strncmp(stHeaderTemp.ID, "HS2L", 4) == 0 )	// 0: identical
		{
			memcpy( &(stLaserData1->Header), &stHeaderTemp, sizeof(StructCommHeader) );
			// receiving laser
			nResult = tcpClientRecv.RecvDataFromServer( (char*)(&(stLaserData1->Timestamp)), (int)(stLaserData1->Header.Length) );
			printf("recv laser(%d) time %d\n", nResult, stLaserData1->Timestamp);
			Sleep(1);
		}

		// check disconnection
		if( nResult == -1 )
		{
			do
			{
				tcpClientRecv.CloseTcpConnection_Client();
				Sleep(3000);
				printf( "Try to ReConnectTcpSocket_Client\n" );
				nResult = tcpClientRecv.ReConnectToTcpServer( tcpClientRecv.m_cIpAddress, tcpClientRecv.m_usPort );
			}
			while( nResult < 0 );
		}
	}

	return 1;
}


int ClassRemoteCommClient::Loop( cv::Mat imgSrc, int RemoteComm_Video_On )
{
//	Mat imgSrc;
//	HS2_TcpipComm tcpVidRecv;
//	tcpVidRecv.SetSocketIPandPort( IPADDRESS_HS2SERVER, TCPPORT_VIDEO_DOWNLINK );
//	tcpVidRecv.ConnectToTcpServer( tcpVidRecv.m_cIpAddress, tcpVidRecv.m_usPort );

	tcpClientRecv.SetRecvSocketBuffer( tcpClientRecv.m_hClntSock, 65535 );
	int nResult;
	while( RemoteComm_Video_On > 0 )
	{
		int nTotalLength;
		nResult = tcpClientRecv.RecvDataFromServer( (char*)(&nTotalLength), sizeof(int) );

		std::vector <uchar> buf(nTotalLength);
		tcpClientRecv.RecvLargeDataFromServer( (char*)(buf.data()), nTotalLength, 3000);

		imgSrc = cv::imdecode( buf, CV_LOAD_IMAGE_COLOR );

		printf("Recv Video ");
		cv::imshow( "imgSrc_Client", imgSrc );
		
		if( cv::waitKey( 5 ) == 27 )
			break;
	}	
	return 1;
}

// ----------------------------------------------------------------------------------
// For Sensor Onboard TEST
/*
unsigned WINAPI Thread_TcpServerRecv_SensorOnboard(void *arg)
{
	HS2_TcpipComm tcpServerRecv_SensorOnboard;

	tcpServerRecv_SensorOnboard.SetTcpServer( TCPPORT_COMMAND_UPLINK );
	tcpServerRecv_SensorOnboard.WaitForTcpClient();

	while( 1 )
	{
		int nResult;
		char *cBuf = "abc\n";

		nResult = tcpServerRecv_SensorOnboard.RecvDataFromClient( cBuf, 4 );
		
	}

	tcpServerRecv_SensorOnboard.CloseTcpServer();

	return 1;
}


unsigned WINAPI Thread_TcpServerSend_SensorOnboard(void *arg)
{
	HS2_TcpipComm tcpServerSend_SensorOnboard;

	tcpServerSend_SensorOnboard.SetTcpServer( TCPPORT_SENSORDATA_DOWNLINK );
	tcpServerSend_SensorOnboard.WaitForTcpClient();

	while( 1 )
	{
		int nResult;
		char *cBuf = "abc\n";

		nResult = tcpServerSend_SensorOnboard.SendDataToClient( cBuf, 4 );
	}

	tcpServerSend_SensorOnboard.CloseTcpServer();

	return 1;
}

// ----------------------------------------------------------------------------------

// For Sensor GCS
unsigned WINAPI Thread_TcpClientRecv_SensorGCS(void *arg)
{
	HS2_TcpipComm tcpClientRecv_SensorGCS;

	tcpClientRecv_SensorGCS.SetSocketIPandPort( IPADDRESS_HS2SERVER, TCPPORT_SENSORDATA_DOWNLINK );

	tcpClientRecv_SensorGCS.ConnectToTcpServer( tcpClientRecv_SensorGCS.m_cIpAddress, tcpClientRecv_SensorGCS.m_usPort );

	while( 1 )
	{
		int nResult;
		char *cBuf = "abc\n";

		nResult = tcpClientRecv_SensorGCS.RecvDataFromServer( cBuf, 4 );
		
		if( nResult == -1 )
		{
			do
			{
				tcpClientRecv_SensorGCS.CloseTcpConnection_Client();

				Sleep(3000);
				
				fprintf( stdout, "Try to ReConnectTcpSocket_Client\n" );
				nResult = tcpClientRecv_SensorGCS.ReConnectToTcpServer( tcpClientRecv_SensorGCS.m_cIpAddress, tcpClientRecv_SensorGCS.m_usPort );
			}
			while( nResult < 0 );
		}
	}

	tcpClientRecv_SensorGCS.CloseTcpConnection_Client();

	return 1;
}


unsigned WINAPI Thread_TcpClientSend_SensorGCS(void *arg)
{
	HS2_TcpipComm tcpClientSend_SensorGCS;

	tcpClientSend_SensorGCS.SetSocketIPandPort( IPADDRESS_HS2SERVER, TCPPORT_COMMAND_UPLINK );

	tcpClientSend_SensorGCS.ConnectToTcpServer( tcpClientSend_SensorGCS.m_cIpAddress, tcpClientSend_SensorGCS.m_usPort );

	while( 1 )
	{
		int nResult;
		char *cBuf = "abc\n";

		nResult = tcpClientSend_SensorGCS.SendDataToServer( cBuf, 4 );
		
		if( nResult == -1 )
		{
			do
			{
				tcpClientSend_SensorGCS.CloseTcpConnection_Client();

				Sleep(3000);
				
				fprintf( stdout, "Try to ReConnectTcpSocket_Client\n" );
				nResult = tcpClientSend_SensorGCS.ReConnectToTcpServer( tcpClientSend_SensorGCS.m_cIpAddress, tcpClientSend_SensorGCS.m_usPort );
			}
			while( nResult < 0 );
		}
	}

	tcpClientSend_SensorGCS.CloseTcpConnection_Client();

	return 1;
}
*/
// ----------------------------------------------------------------------------------



/*
unsigned WINAPI Thread_RecvVideo2(void *arg)
{
	Mat imgSrc;

	HS2_TcpipComm tcpVidRecv;

	tcpVidRecv.SetSocketIPandPort( IPADDRESS_HS2SERVER, TCPPORT_VIDEO_DOWNLINK );

	tcpVidRecv.ConnectToTcpServer( tcpVidRecv.m_cIpAddress, tcpVidRecv.m_usPort );

	tcpVidRecv.SetRecvSocketBuffer( tcpVidRecv.m_hClntSock, 65535 );

	int nResult;
	while( 1 )
	{
		int nTotalLength;

		nResult = tcpVidRecv.RecvDataFromServer( (char*)(&nTotalLength), sizeof(int) );

		vector <uchar> buf(nTotalLength);
		
		tcpVidRecv.RecvLargeDataFromServer( (char*)(buf.data()), nTotalLength, 3000);

		imgSrc = imdecode( buf, CV_LOAD_IMAGE_COLOR );

		printf("Send ");
		imshow( "imgSrc_Client", imgSrc );
		
		if( waitKey( 5 ) == 27 )
			break;
	}

	return 1;
}

unsigned WINAPI Thread_SendVideo2(void *arg)
{
	//Mat imgSrc;
	Mat imgSrc2;

	namedWindow("imgSrc_Server");

	vector <int> compression_params;
	compression_params.push_back( CV_IMWRITE_JPEG_QUALITY );
	compression_params.push_back( 95 );
	// ---

	HS2_TcpipComm tcpVidSend;
	tcpVidSend.SetTcpServer( TCPPORT_VIDEO_DOWNLINK );	// ~ listen

	tcpVidSend.WaitForTcpClient();	// accept

	tcpVidSend.SetSendSocketBuffer( tcpVidSend.m_hServSock, 65535 );

	int nResult;
	while( 1 )
	{
		resize( g_CameraImage1.imgSrc, imgSrc2, Size(320,240) );
		//resize( imgSrc, imgSrc2, Size(320,240) );
		//imgSrc.copyTo( imgSrc2 );

		vector <uchar> buf;
		bool bResult = imencode(".jpeg", imgSrc2, buf, compression_params );

		imshow( "imgSrc_Server", imgSrc2 );

		//----

		nResult = tcpVidSend.SendLargeDataToClient( (char*)(&buf[0]), buf.size(), 3000);
		printf("SendVideoData");

		// if client is disconnected
		if( nResult <= 0  )
		{
			Sleep(50);
			printf("ExternalVision : Waiting to Reconnect...\n");
			tcpVidSend.WaitForTcpClient();
			printf("ExternalVision : ReConnectTcpSocket_Server\n", nResult );
		}

		if( waitKey( 30 ) == 27 )
			break;
	}

	tcpVidSend.CloseTcpServer();
	imgSrc2.release();
	return 1;
}
*/



// ---------------------------------- Mobile ---------------------------------
/*
int Thread_RunTcpClient_Mobile()
{
	tcpMobile.InitializeTcpClient( IPADDRESS_HS2SERVER, TCPPORT_MOBILE );
	g_States.TcpCommMobile_On = 1;

	int i=0;		DWORD threadID1;
	HANDLE handle1 = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)Thread_TcpClientRecv_Mobile, (LPVOID)i, 0, &threadID1);
//	int priority = GetThreadPriority( handle );
//	SetThreadPriority(handle, THREAD_PRIORITY_TIME_CRITICAL);
//	priority = GetThreadPriority( handle );

	int k=0;		DWORD threadID2;
	HANDLE handle2 = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)Thread_TcpClientSend_Mobile, (LPVOID)k, 0, &threadID2);

	return 1;
}


void Thread_TcpClientSend_Mobile(int maxThreads)
{
	while( g_States.TcpCommMobile_On == 1 )
	{
		tcpMobile.SendDataToMobile();
		//printf("Send Mobile\n");
		Sleep(1000);
	}

	tcpMobile.CloseTcpClient();

	return;
}


void Thread_TcpClientRecv_Mobile(int maxThreads)
{
	while(1)
	{
		
		tcpMobile.RecvDataFromMobile();
		//printf("Recv Mobile\n");
		Sleep(500);
	}

	tcpMobile.CloseTcpClient();

	return;
}


int TcpComm::SendDataToMobile()	// Mobile CLIENT -> SERVER
{
	//fflush( stdout );

	int nResult;// = 100;
	//if( (100<g_States.SendCommandToServer)&&(g_States.SendCommandToServer<200) )
	if( g_States.SendCommandToServer != 0 )
	{
		g_TcpCommand_Mobile.Command[0] = g_States.SendCommandToServer;
		nResult = send( m_hClntSock, (char*)(&g_TcpCommand_Mobile), sizeof(g_TcpCommand_Mobile), 0 );
		//printf("(SendResult%d)", nResult );

		g_States.SendCommandToServer = 0;
	}


	return 1;
}


int TcpComm::RecvDataFromMobile()	// at CLIENT
{
	int nResult;
	//int nLength = sizeof(g_TcpSensorData_Mobile);

	nResult = recv( m_hClntSock, (char*)(&g_TcpSensorData_Mobile), sizeof(g_TcpSensorData_Mobile), 0);


	if( nResult == -1 )
	{
		CloseTcpClient();

		Sleep(3000);
				
		fprintf( stdout, "Try to ReConnectTcpSocket_Client Mobile\n" );
		//ReConnectTcpSocket_Client( IPADDRESS_HS2SERVER, PORT_MOBILE );
	}

//	if(nResult == -1)	// 수신실패
//		ErrorHandling("read() error");

	if( g_TcpSensorData_Mobile.Status[0] != 0 )
	{
		if( g_TcpSensorData_Mobile.Status[0] & REMOTE_REC_ON )
		{
			fprintf( stdout, "Remote DatVideo Save Started\n" );
			g_States.ServerStatus |= REMOTE_REC_ON;
			g_States.ServerStatus = ~REMOTE_REC_OFF;
		}

		if( g_TcpSensorData_Mobile.Status[0] & REMOTE_REC_OFF )
		{
			fprintf( stdout, "Remote DatVideo Save Ended\n" );
			g_States.ServerStatus = ~REMOTE_REC_ON;
			g_States.ServerStatus |= REMOTE_REC_OFF;
		}
	}


	return 1;
}


int TcpComm::InitializeTcpClient( char* ipAddress, unsigned short port )
{

	return HS2_TcpipComm::ConnectToTcpServer( ipAddress, port );
}


int TcpComm::ReConnectTcpSocket_Client()// char* ipAddress, unsigned short port )
{
	return HS2_TcpipComm::ReConnectToTcpServer( m_cIpAddress, this->m_usPort );
}


int TcpComm::CloseTcpClient()
{
	return HS2_TcpipComm::CloseTcpConnection_Client();
}

*/

// ----------------------------------------
