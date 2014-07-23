// Sungsik Huh
// 2013.10.18
// 

#define USE_POSIX_THREAD	1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#ifdef USE_POSIX_THREAD
// POSIX threads
#include "pthread.h"
#else
//#include <windows.h>	// _beginthreadex()
#include <process.h>	// _beginthreadex()
#endif

#include "hs2def.h"
#include "RemoteComm.h"

#include "CamPointgrey.h"
#include "HS2_CValgorithms.h"

#include "HS2_LaserHokuyo.h"



using namespace std;

SharedStates g_States;
StructLaserParameters g_LaserParam1;

StructSensorsDownlink	g_TcpSensorData;
StructLaserdataDownlink g_LaserData1;

StructCommandUplink		g_TcpCommand;

PCK_COMMAND_BEARNAV			g_TcpCommand_Bearnav;
PCK_SENSORDATA_BEARNAV		g_TcpSensorData_Bearnav;

EstimatedVehiclePoseStates g_stEstPoseStates;
ClassCommClient_Bearnav g_CommBearnav1;

ClassCameraImage g_CameraImage1;

// -----------------------------------------------------------------------------





// ---------------------- Thread_CommClientSend_Bearnav -----------------------
#ifdef USE_POSIX_THREAD
void *Thread_CommClientSend_Bearnav(void *arg)
{
	g_CommBearnav1.Loop_Send( &g_States, &g_stEstPoseStates, &g_TcpSensorData_Bearnav, &g_TcpCommand_Bearnav );
	//g_CommBearnav1.Terminate();
	pthread_exit( (void *)1 );
	return (void *)1;
}
#else
unsigned WINAPI Thread_CommClientSend_Bearnav(void *arg)
{
	g_CommBearnav1.Loop_Send( &g_States, &g_stEstPoseStates, &g_TcpSensorData_Bearnav, &g_TcpCommand_Bearnav );
	//g_CommBearnav1.Terminate();
	return 1;
}
#endif


// ---------------------------Thread_CommClientRecv_Bearnav -----------------------------------
#ifdef USE_POSIX_THREAD
void *Thread_CommClientRecv_Bearnav(void *arg)
{
	g_CommBearnav1.Initiate( IPADDRESS_BEARNAV, PORT_BEARNAV );
	int iParam2=0;		
	// Create a mutex
	//mutexCommand = new pthread_mutex_t;
	//pthread_mutex_init(mutexCommand, NULL);
	// Create a thread
	pthread_t pthreadCommClientSend_Bearnav;
	if( pthread_create( &pthreadCommClientSend_Bearnav, NULL, Thread_CommClientSend_Bearnav, (void*)iParam2 ) != 0 ) 
	{
		return 0;
	}
	g_CommBearnav1.Loop_Recv( &g_States, &g_TcpSensorData_Bearnav );
	g_CommBearnav1.Terminate();
	pthread_exit( (void *)1 );
	return (void *)1;
}
#else
unsigned WINAPI Thread_CommClientRecv_Bearnav(void *arg)
{
	g_CommBearnav1.Initiate( IPADDRESS_BEARNAV, PORT_BEARNAV );

	int iParam2=0;		
	unsigned int threadID2;
	HANDLE handle2 = (HANDLE)_beginthreadex(NULL, 0, Thread_CommClientSend_Bearnav, (void*)&iParam2, 0, &threadID2);

	g_CommBearnav1.Loop_Recv( &g_States, &g_TcpSensorData_Bearnav );
	g_CommBearnav1.Terminate();

	return 1;
}
#endif

// ---------------------------Thread_RemoteCommServer: States-----------------------------------
#ifdef USE_POSIX_THREAD
void *Thread_RemoteCommServer_StatesDownlink(void *arg)
#else
unsigned WINAPI Thread_RemoteCommServer_StatesDownlink( void *arg )
#endif
{
	ClassRemoteCommServer Server_StatesDownlink;
	Server_StatesDownlink.Initiate( TCPPORT_SENSORDATA_DOWNLINK );
	Server_StatesDownlink.Loop( &g_TcpSensorData, &g_LaserData1, g_States.RemoteComm_States_On );
	Server_StatesDownlink.Terminate();
#ifdef USE_POSIX_THREAD
	pthread_exit( (void *)1 );
	return (void *)1;
#else
	return 1;
#endif
}


// ---------------------------Thread_RemoteCommServer: Video-----------------------------------
#ifdef USE_POSIX_THREAD
void *Thread_RemoteCommServer_VideoDownlink(void *arg)
#else
unsigned WINAPI Thread_RemoteCommServer_VideoDownlink( void *arg )
#endif
{
	ClassRemoteCommServer Server_StatesDownlink;
	Server_StatesDownlink.Initiate( TCPPORT_VIDEO_DOWNLINK );
	Server_StatesDownlink.Loop( g_CameraImage1.imgSrc, g_States.RemoteComm_Video_On );
	Server_StatesDownlink.Terminate();
#ifdef USE_POSIX_THREAD
		pthread_exit( (void *)1 );
		return (void *)1;
#else
	return 1;
#endif
}


// ---------------------------Thread_CameraPointgrey2-----------------------------------
#ifdef USE_POSIX_THREAD
void *Thread_CameraPointgrey2( void *arg )
#else
unsigned WINAPI Thread_CameraPointgrey2( void *arg )
#endif
{
	ClassCamPointgrey cam1;

	int *iParam = (int*)arg;
	if( cam1.InitiateCamera(*iParam) != 1 )
	{
		printf( "FFMV camera was not connected\n" );
#ifdef USE_POSIX_THREAD
		pthread_exit( (void *)1 );
		return (void *)1;
#else
		return -1;
#endif
	}
	namedWindow( "m_imgSrcColor" );

	g_States.CAM_On = 1;

	Sleep(100);

	g_CameraImage1.Initiate( cam1.SizeOfCamImage.height, cam1.SizeOfCamImage.width, 3 );
	cam1.GrabImage();
	cv::waitKey(30);
		
	ClassExtractColor ec1;
	ec1.Initiate( cam1.m_imgSrcColor );

	Mat imgBinary;
	imgBinary.create( cam1.m_imgSrcColor.size(), CV_8UC1 );
	ClassContourProcess cp1;
	cp1.Initiate( imgBinary );

	double dPosTarget[3] = {0.0, 0.0, 0.0};

	while( g_States.CAM_On == 1 )
	{
		cam1.GrabImage();

		ec1.ExtractColor_Red( cam1.m_imgSrcColor, imgBinary );
	
		cp1.ExtractContours( imgBinary );

		int iIdxBig = 0;
		iIdxBig = cp1.FindTheBiggestMarker( cp1.m_vContourStates );

		if( iIdxBig >= 0 )
		{
			cv::circle( cam1.m_imgSrcColor, cv::Point2d(cp1.m_vContourStates[iIdxBig].pos[0],cp1.m_vContourStates[iIdxBig].pos[1]), 30, CV_RGB(255,255,255), 1 );

			// 0.020106 = 0.08m*0.08m*PI [m^2]
			cp1.EstimateDistanceToMarker( 0.020106, cp1.m_vContourStates[iIdxBig].area, cp1.m_vContourStates[iIdxBig].pos, g_TcpSensorData_Bearnav.attitude, dPosTarget );
			
			printf("X=%.3f Y=%.3f Z=%.3f\n", dPosTarget[0], dPosTarget[1], dPosTarget[2] );

			g_stEstPoseStates.dMeanPos[0] = dPosTarget[0];
			g_stEstPoseStates.dMeanPos[1] = dPosTarget[1];
			g_stEstPoseStates.dMeanPos[2] = dPosTarget[2];
				
			g_stEstPoseStates.dCovPos[0] = 0.06;
			g_stEstPoseStates.dCovPos[1] = 0.06;
			g_stEstPoseStates.dCovPos[2] = 0.06;
		}

		g_stEstPoseStates.flag = 1;

		imshow( "m_imgSrcColor", cam1.m_imgSrcColor );

		// video server
		if( g_States.RemoteComm_Video_On == 1 )
			cam1.m_imgSrcColor.copyTo( g_CameraImage1.imgSrc );

		cv::waitKey(30);
	}

	cam1.StopCamera();
	printf("FFMV camera disconnected\n");

#ifdef USE_POSIX_THREAD	
	pthread_exit( (void *)1 );
	return (void *)1;
#else
	return 1;
#endif
}

// ---------------------------Thread_LaserHokuyoSerial-----------------------------------
#ifdef USE_POSIX_THREAD	
void *Thread_LaserHokuyoSerial(void *arg)
#else
unsigned WINAPI Thread_LaserHokuyoSerial(void *arg)
#endif
{
	HS2_LaserHokuyo Laser1;
	Laser1.Initiate( &(g_States.LRF_On), &(g_LaserParam1), 14 );//14;//6;//10;//3;
	Laser1.Loop_GD( &(g_States.LRF_On), &(g_LaserData1) );
	Laser1.Terminate();
#ifdef USE_POSIX_THREAD	
	pthread_exit( (void *)1 );
	return (void *)1;
#else
	return 1;
#endif
}

// -----------------------------------------------------------------------------

int Console_Initial()
{
	memset( &g_States, 0, sizeof(g_States) );
/*
	g_States.IMU_On = 0;
	g_States.CAM_On = 0;
	g_States.LRF_On = 0;
	g_States.Motor_On = 0;
	g_States.GPS_On = 0;
	g_States.OpenGL_On = 0;

	g_States.FusionSLAM_On = 0;
	g_States.ReplayVideo_On = 0;
	g_States.VideoRecording_On = 0;

	g_States.TcpConnected = 0;


	g_States.LaserPointsViewMode  = 0;


	for( int i=0; i<4; i++ )
		g_TcpSensorData.SensorStatus[i] = 0;

	for( int i=0; i<12; i++ )
		g_TcpSensorData.EKFSLAM_MeanCurr[i] = 0.0f;

	g_TcpSensorData.NumEKFSLAMLMs = 0;

	for( int i=0; i<12; i++ )
		g_TcpSensorData.IMUdata[i] = 0.0;

	for( int i=0; i<3; i++ )
		g_TcpSensorData.ServoState[i] = 0.0;


	for( int i=0; i<20; i++ )
		g_TcpCommand.MotorControl[i] = 0;
*/


	return 1;
}


int main(int argc, char** argv)
{
	int key = 1;

	Console_Initial();

	printf( "------------------------------  HS2_CONSOLE  --------------------------------\n" );
	printf( "laser(l) cam(c) servo(v) imu(i) fslam(f) visionNav(n)\n" );
	printf( "tcpGround(t) tcpVideo(y) tcpBearnav(b) sendGPS(g)\n" );
	printf( "quit(q) help(h) opengl(o) pointcloud(p)\n" );
	printf( "record_video(s)  play<<start(r) stop(w) resume(e)>>\n" );
	printf( "servo_mode(m) servo_up(,) servo_down(.) servo_center(/)\n" );
	printf( "-----------------------------------------------------------------------------\n" );
	while( key )
	{
		switch( getchar() )
		{
			case 'h' : 
			{
				printf( "laser(l) cam(c) servo(v) imu(i) fslam(f) visionNav(n)\n" );
				printf( "tcpGround(t) tcpVideo(y) tcpBearnav(b) sendGPS(g)\n" );
				printf( "quit(q) help(h) opengl(o) pointcloud(p)\n" );
				printf( "record_video(s)  play<<start(r) stop(w) resume(e)>>\n" );
				printf( "servo_mode(m) servo_up(,) servo_down(.) servo_center(/)\n" );
				break;
			}

			case 'q' : 
			{
				key = 0;
				printf( "Exiting...\n" );
				break;
			}

/*					case 'o' : 
			{
				int iParam1=0;		unsigned int threadID1;
				HANDLE handle1 = (HANDLE)_beginthreadex(NULL, 0, Thread_OpenglView, (void*)&iParam1, 0, &threadID1);

				printf( "OpenGL\n" );
				break;
			}
*/
			case 'l' : 
			{
				if( g_States.LRF_On == 0 )
				{
					g_States.LRF_On = 1;
					int iParam1=0;		
					
#ifdef USE_POSIX_THREAD
					// Create a mutex
					//mutexCommand = new pthread_mutex_t;
					//pthread_mutex_init(mutexCommand, NULL);

					// Create a thread
					pthread_t pthreadLaserHokuyoSerial;
					if( pthread_create( &pthreadLaserHokuyoSerial, NULL, Thread_LaserHokuyoSerial, (void*)iParam1 ) != 0 ) 
					{
						return 0;
					}
#else
					unsigned int threadID1;
					HANDLE handle1 = (HANDLE)_beginthreadex(NULL, 0, Thread_LaserHokuyoSerial, (void*)&iParam1, 0, &threadID1);
#endif

					printf( "Thread_LaserHokuyoSerial\n" );
				}
				else
				{
					g_States.LRF_On = 0;
				}
				break;
			}

			case 'c' : 
			{
				// Camera fixed exposure
				if( g_States.CAM_On == 0 )
				{
					g_States.CAM_On = 1;
					int *iParam = new int;
					*iParam=2;

#ifdef USE_POSIX_THREAD
					// Create a mutex
					//mutexCommand = new pthread_mutex_t;
					//pthread_mutex_init(mutexCommand, NULL);

					// Create a thread
					pthread_t pthreadCameraPointgrey2;
					if( pthread_create( &pthreadCameraPointgrey2, NULL, Thread_CameraPointgrey2, (void*)iParam ) != 0 ) 
					{
						return 0;
					}
#else
					unsigned int threadID1;
					HANDLE handle1 = (HANDLE)_beginthreadex(NULL, 0, Thread_CameraPointgrey2, (void*)iParam, 0, &threadID1);
#endif
			
					printf( "Thread_CameraPointgrey: auto\n" );
				}
				else
				{
					g_States.CAM_On = 0;
				}
				break;
			}

			
/*
			case 'C' : 
			{
				int i=0;	// autosetting
				DWORD threadID1;
				HANDLE handle1 = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)Thread_FFMVCamera, (LPVOID)i, 0, &threadID1);

				printf( "FFMV Camera (auto setting)\n" );

				break;
			}

			case 's' : 
			{
				if( g_States.VideoRecording_On == 0 )
				{
					g_States.VideoRecording_On = 1;
					fprintf( stdout, "----------DatVideo Save Started----------\n" );
				}
				else if( g_States.VideoRecording_On == 2 )
				{
					g_States.VideoRecording_On = 3;
					fprintf( stdout, "----------DatVideo Save Ended----------\n" );
				}

				break;
			}

			case 'r' : 
			{
				fprintf( stdout, "DatVideo Replay\n" );
				int i=0;		DWORD threadID1;
				HANDLE handle1 = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)DatVideoReplay, (LPVOID)i, 0, &threadID1);

				break;
			}

			case 'w' : 
			{
				// replay stop
				g_States.ReplayVideo_On = 1;
				break;
			}

			case 'e' : 
			{
				// replay ++
				g_States.ReplayVideo_On = 10;

				break;
			}

			case 'v' : 
			{
				if( g_States.Motor_On == 0 )
				{
					int i=0;		DWORD threadID1;
					HANDLE handle1 = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)MotorThread, (LPVOID)i, 0, &threadID1);
					fprintf( stdout, "Servo On\n" );
				}
				else
				{
					g_States.Motor_On = 0;
					fprintf( stdout, "Servo Off\n" );
				}

				break;
			}

			case 'p' : 
			{
				// Laser Point Cloud
				// --- pusing button switches between two modes ---
				// cumulative map mode
				if(	g_States.LaserPointsViewMode == 0 )
				{
					g_States.LaserPointsViewMode = 1;	// --> 2
				}
				else //(	g_States.LaserPointsViewMode != 0 )
				{
					g_States.LaserPointsViewMode = 3;	// --> 4 --> 0
				}
				break;
			}

			case 'i' : 
			{
				if( g_States.IMU_On == 0 )
				{
					g_States.IMU_On = 1;
					int i=0;		DWORD threadID1;
					HANDLE handle1 = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)Thread_IMU, (LPVOID)i, 0, &threadID1);
					fprintf( stdout, "Thread IMU On\n" );
				}
				else
				{
					g_States.IMU_On = 0;
					fprintf( stdout, "Thread IMU Off\n" );
				}

				break;
			}

			case 'm' : 
			{
				if( g_MotorControl[0] == 1 )
					g_MotorControl[0] = 2;
				else
					g_MotorControl[0] = 1;

				break;
			}

			case '/' : 
			{
				g_MotorControl[1] = 500;
				break;
			}

			case ',' : 
			{
				g_MotorControl[1] += 30;
				break;
			}

			case '.' : 
			{
				g_MotorControl[1] -= 30;
				break;
			}
*/

			case 'n' : 
			{
				g_stEstPoseStates.dMeanPos[0] += 1.0;
				g_stEstPoseStates.dMeanPos[1] += 1.0;
				g_stEstPoseStates.dMeanPos[2] = -2.0;

				g_stEstPoseStates.dCovPos[0] = 2.0;
				g_stEstPoseStates.dCovPos[1] = 2.0;
				g_stEstPoseStates.dCovPos[2] = 2.0;
			
				g_stEstPoseStates.flag = 0;
				for( int i=0; i<3; i++ )
				{
					g_stEstPoseStates.dMeanVel[i] = 0.0;
					g_stEstPoseStates.dMeanAtt[i] = 0.0;

					g_stEstPoseStates.fLccPosCurr[i] = 0.f;
					g_stEstPoseStates.fLccPosPrev[i] = 0.f;
				}
				

				break;
			}


			case 't' : 
			{
				if( g_States.RemoteComm_States_On == 0 )
				{
					g_States.RemoteComm_States_On = 1;
					int iParam1=0;		
#ifdef USE_POSIX_THREAD					
					// Create a mutex
					//mutexCommand = new pthread_mutex_t;
					//pthread_mutex_init(mutexCommand, NULL);

					// Create a thread
					pthread_t pthreadRemoteCommServer_StatesDownlink;
					if( pthread_create( &pthreadRemoteCommServer_StatesDownlink, NULL, Thread_RemoteCommServer_StatesDownlink, (void*)iParam1 ) != 0 ) 
					{
						return 0;
					}
#else					
					unsigned int threadID1;
					HANDLE handle1 = (HANDLE)_beginthreadex(NULL, 0, Thread_RemoteCommServer_StatesDownlink, (void*)&iParam1, 0, &threadID1);
#endif
					printf( "Thread_RemoteCommServer_StatesDownlink\n" );
				}
				else
				{
					g_States.RemoteComm_States_On = 0;
				}
				break;
			}

			case 'y' : 
			{
				if( g_States.RemoteComm_Video_On == 0 )
				{
					g_States.RemoteComm_Video_On = 1;
					int iParam1=0;		
#ifdef USE_POSIX_THREAD
					// Create a mutex
					//mutexCommand = new pthread_mutex_t;
					//pthread_mutex_init(mutexCommand, NULL);

					// Create a thread
					pthread_t pthreadRemoteCommServer_VideoDownlink;
					if( pthread_create( &pthreadRemoteCommServer_VideoDownlink, NULL, Thread_RemoteCommServer_VideoDownlink, (void*)iParam1 ) != 0 ) 
					{
						return 0;
					}
#else					
					unsigned int threadID1;
					HANDLE handle1 = (HANDLE)_beginthreadex(NULL, 0, Thread_RemoteCommServer_VideoDownlink, (void*)&iParam1, 0, &threadID1);
#endif
					printf( "Thread_RemoteCommServer_VideoDownlink\n" );
				}
				else
				{
					g_States.RemoteComm_Video_On = 0;
				}
				break;
			}

			case 'b' : 
			{
				g_stEstPoseStates.dMeanPos[0] = 0.0;
				g_stEstPoseStates.dMeanPos[1] = 0.0;
				g_stEstPoseStates.dMeanPos[2] = 0.0;

				g_stEstPoseStates.dCovPos[0] = 2.0;
				g_stEstPoseStates.dCovPos[1] = 2.0;
				g_stEstPoseStates.dCovPos[2] = 2.0;
			
				g_stEstPoseStates.flag = 0;
				for( int i=0; i<3; i++ )
				{
					g_stEstPoseStates.dMeanVel[i] = 0.0;
					g_stEstPoseStates.dMeanAtt[i] = 0.0;

					g_stEstPoseStates.fLccPosCurr[i] = 0.f;
					g_stEstPoseStates.fLccPosPrev[i] = 0.f;
				}


				if( g_States.TcpCommBearnav_On == g_States.Off )
				{
					g_States.TcpCommBearnav_On = g_States.PartiallyActivated;
					int iParam1=0;		
#ifdef USE_POSIX_THREAD
					// Create a mutex
					//mutexCommand = new pthread_mutex_t;
					//pthread_mutex_init(mutexCommand, NULL);

					// Create a thread
					pthread_t pthreadCommClientRecv_Bearnav;
					if( pthread_create( &pthreadCommClientRecv_Bearnav, NULL, Thread_CommClientRecv_Bearnav, (void*)iParam1 ) != 0 ) 
					{
						return 0;
					}
#else					
					unsigned int threadID1;
					HANDLE handle1 = (HANDLE)_beginthreadex(NULL, 0, Thread_CommClientRecv_Bearnav, (void*)&iParam1, 0, &threadID1);
#endif
					printf( "Thread_CommClientRecv_Bearnav On\n" );
				}
				else
				{
					g_States.TcpCommBearnav_On = g_States.Off;
					printf( "Thread_CommClientRecv_Bearnav Off\n" );
				}
				break;
			}

/*
			case 'f' : 
			{
				if( g_States.FusionSLAM_On == 0 )
				{
					g_States.FusionSLAM_On = 1;
					fprintf( stdout, "F_SLAM On\n" );
				}
				else
				{
					g_States.FusionSLAM_On = 0;
					fprintf( stdout, "F_SLAM Off\n" );
				}

				break;
			}

			case 'g' : 
			{
				if( g_States.GPS_On == 0 )
				{
					g_States.GPS_On = 1;
					fprintf( stdout, "SLAM GPS On\n" );
					int i=0;		DWORD threadID1;
					HANDLE handle1 = CreateThread(0, 0, (LPTHREAD_START_ROUTINE)Thread_PseudoGps, (LPVOID)i, 0, &threadID1);
				}
				else
				{
					g_States.GPS_On = 0;
					fprintf( stdout, "SLAM GPS Off\n" );
				}

				break;
			}
	*/

		}

		Sleep(300);
	}

	printf( "Program Terminated...\n" );
		
	return 1;
}


