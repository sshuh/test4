// Sungsik Huh
// 2013.09.23
// 

#ifndef REMOTE_COMM_H_
#define REMOTE_COMM_H_
#endif /* REMOTE_COMM_H_ */


#ifdef _WIN32
#pragma once
#include <stdio.h>
#include <math.h>
#include "OpenCV2.4.3/include/opencv2/opencv.hpp"
#endif


#ifdef __linux__
#include <stdio.h>
#include <math.h>
#include "opencv2/opencv.hpp"

#ifndef Sleep
#define Sleep(x)	usleep(x*1000)
#endif

#endif

#include "hs2def.h"
#include "HS2_TcpipComm.h"

/*
// Server: Sensor onboard
unsigned WINAPI Thread_TcpServerRecv_SensorOnboard(void *arg);
unsigned WINAPI Thread_TcpServerSend_SensorOnboard(void *arg);


// Client: Sensor GCS
unsigned WINAPI Thread_TcpClientRecv_SensorGCS(void *arg);
unsigned WINAPI Thread_TcpClientSend_SensorGCS(void *arg);	


// onboard video downlink
unsigned WINAPI Thread_RecvVideo2(void *arg);
unsigned WINAPI Thread_SendVideo2(void *arg);


int Thread_RunTcpClient_Mobile();

class TcpComm : HS2_TcpipComm
{
// common
public:
	int SendDataToMobile();
	int RecvDataFromMobile();

	int ErrorHandling( char* message );

	int InitializeTcpClient( char* ipAddress, unsigned short port );
	int ReConnectTcpSocket_Client();// char* ipAddress, unsigned short port );
	int CloseTcpClient();
};
*/


class ClassCommClient_Bearnav
{
public:
	HS2_TcpipComm tcpClient;

	int Initiate( char *cIpAddress, unsigned short usPort );
	int Loop_Send(SharedStates *stStates, MCL3dStates *stMcl3dStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav, PCK_COMMAND_BEARNAV *stTcpCommand_Bearnav );
	int Loop_Send(SharedStates *stStates, EstimatedVehiclePoseStates *stEstPoseStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav, PCK_COMMAND_BEARNAV *stTcpCommand_Bearnav );
	int Loop_Recv(SharedStates *stStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav );
	unsigned short ComputeBodyChecksum(unsigned short *ptbody, int WordCount);
	int CheckBearnavDataM6002( SharedStates *stStates, PCK_SENSORDATA_BEARNAV *stTcpSensorData_Bearnav );
	int Terminate();
};

class ClassRemoteCommServer
{
public:
	HS2_TcpipComm tcpServerSend;

	int Initiate( unsigned short usPort );
	int Loop( StructSensorsDownlink *stSensorData1, StructLaserdataDownlink *stLaserData1, int RemoteComm_States_On );
	int Loop( cv::Mat imgSrc, int RemoteComm_Video_On );
	int Terminate();
};


class ClassRemoteCommClient
{
public:
	HS2_TcpipComm tcpClientRecv;

	int Initiate( char *cIpAddress, unsigned short usPort );
	int Loop( StructSensorsDownlink *stSensorData1, StructLaserdataDownlink *stLaserData1, int RemoteComm_States_On );
	int Loop( cv::Mat imgSrc, int RemoteComm_Video_On );
	int Terminate();
};

