// Sungsik Huh
// 2013.10.23
// 

//#include "stdafx.h"
#include "HS2_LaserHokuyo.h"

#ifdef __linux__
#endif

#ifdef _WIN32
#endif

/*
unsigned WINAPI Thread_LaserHokuyoSerial_Example(void *arg)
{
	HS2_LaserHokuyo Laser1;
	Laser1.m_iComPort = 6;//14;//6;//10;//3;

	// connect serial port: 115200
	if( Laser1.OpenLaserScanner( Laser1.m_iComPort, 115200, 800 ) < 0 )
		return 0;

	Sleep(50);

	if( Laser1.FindAndSetBaudRate() < 0 )
		return 0;

	if( Laser1.GetLaserSpecs() < 0 )
		return 0;

	Laser1.SetLaserParameters();
	memcpy( &(g_LaserParam1), &(Laser1.LaserParam1), sizeof(LaserParameters) );
	Sleep(50);

	g_States.LRF_On = 1;

	if( Laser1.EnableMeasurement_BM() == 1 )
	{
		printf("EnableMeasurement_BM: OK\n");
	}
	else
	{
		printf("EnableMeasurement_BM: Failed\n");
		Laser1.DisableMeasurement_QT();
		Laser1.CloseLaserScanner();
		return 0;
	}

	Sleep(150);
	Laser1.HighSensitivity_HS();

	Sleep(150);
	Laser1.CaptureLaserRange( "MD", 2 );

	while( g_States.LRF_On )
	{
		//Laser1.GetLaserRangeData_MDMS( g_TcpSensorData.LRFdata );
		Laser1.GetLaserRangeData_MDMS( g_LaserData1.Laserdata, &(g_LaserData1.Timestamp) );
		memcpy( g_TcpSensorData.LRFdata, g_LaserData1.Laserdata, 1081*2 );
		//memcpy( g_LaserData1.Laserdata, g_TcpSensorData.LRFdata, 1081*2 );
		//printf("Timestamp %d\n", g_LaserData1.Timestamp );

		//Laser1.CaptureLaserRange( "GD", 2 );
		//Laser1.GetLaserRangeData_GDGS( &(g_TcpSensorData.LRFdata[0]) );

		Sleep(3);
	}
	Sleep(30);
	Laser1.DisableMeasurement_QT();
	Sleep(30);
	Laser1.CloseLaserScanner();
	Sleep(30);

	return 1;
}
*/ 
int HS2_LaserHokuyo::Initiate( int *Laser_On, StructLaserParameters *stLaserParam1, int iComPort )
{
	//Laser1.m_iComPort = 6;//14;//6;//10;//3;
	m_iComPort = iComPort;

	// connect serial port: 115200
	if( OpenLaserScanner( m_iComPort, 115200, 800 ) < 0 )
		return 0;

	Sleep(30);

	if( FindAndSetBaudRate() < 0 )
		return 0;

	if( GetLaserSpecs() < 0 )
		return 0;

	SetLaserParameters();
	memcpy( stLaserParam1, &(m_LaserParam1), sizeof(StructLaserParameters) );
	Sleep(30);

	*Laser_On = 1;

	if( EnableMeasurement_BM() == 1 )
	{
		printf("EnableMeasurement_BM: OK\r\n");
	}
	else
	{
		printf("EnableMeasurement_BM: Failed\r\n");
		DisableMeasurement_QT();
		CloseLaserScanner();
		return 0;
	}

	Sleep(150);
	HighSensitivity_HS();

	Sleep(150);

	return 1;
}


int HS2_LaserHokuyo::Loop_MD( int *Laser_On, StructLaserdataDownlink *stLaserData1 )
{
	CaptureLaserRange( "MD", 2 );

	while( *Laser_On == 1 )
	{
		//Laser1.GetLaserRangeData_MDMS( g_TcpSensorData.LRFdata );
		GetLaserRangeData_MDMS( stLaserData1->Laserdata, &(stLaserData1->Timestamp) );
		//memcpy( g_TcpSensorData.LRFdata, g_LaserData1.Laserdata, 1081*2 );
		//memcpy( g_LaserData1.Laserdata, g_TcpSensorData.LRFdata, 1081*2 );
		//printf("Timestamp %d\n", g_LaserData1.Timestamp );
		static int LaserCount = 0;
		if( LaserCount++ %10 == 0 )
			printf("L90=%d, L45=%d, C=%d, R45=%d, R90=%d\r\n", stLaserData1->Laserdata[m_LaserParam1.Left90DegStep+1], stLaserData1->Laserdata[m_LaserParam1.Left90DegStep+1 + 180], stLaserData1->Laserdata[m_LaserParam1.CenterFrontStep], stLaserData1->Laserdata[m_LaserParam1.Right90DegStep-1-180], stLaserData1->Laserdata[m_LaserParam1.Right90DegStep-1] );

		Sleep(3);
	}

	return 1;
}

int HS2_LaserHokuyo::Loop_GD( int *Laser_On, StructLaserdataDownlink *stLaserData1 )
{
	while( *Laser_On == 1 )
	{
		CaptureLaserRange( "GD", 2 );
		GetLaserRangeData_GDGS( stLaserData1->Laserdata, &(stLaserData1->Timestamp) );
		static int LaserCount = 0;
		if( LaserCount++ %10 == 0 )
			printf("L90=%d, L45=%d, C=%d, R45=%d, R90=%d\r\n", stLaserData1->Laserdata[m_LaserParam1.Left90DegStep+1], stLaserData1->Laserdata[m_LaserParam1.Left90DegStep+1 + 180], stLaserData1->Laserdata[m_LaserParam1.CenterFrontStep], stLaserData1->Laserdata[m_LaserParam1.Right90DegStep-1-180], stLaserData1->Laserdata[m_LaserParam1.Right90DegStep-1] );

		Sleep(3);
	}
	return 1;
}

int HS2_LaserHokuyo::Terminate()
{
	Sleep(30);
	DisableMeasurement_QT();
	Sleep(30);
	CloseLaserScanner();
	Sleep(30);

	return 1;
}


int HS2_LaserHokuyo::OpenLaserScanner( int iComPort, int nBaudRate, int nReadTimeout )
{
	m_nRemainByte = 0;

	if ( HS2_SerialComm::OpenSerialPort( iComPort, nBaudRate, nReadTimeout ) < 0 )
	{
		printf("OpenLaserScanner(COM%d Baudrate=%d) Failed!\r\n", iComPort, nBaudRate);
		return -1;
	}
	printf("LaserScanner(COM%d Baudrate=%d) Opened\r\n", iComPort, nBaudRate );
	return 1;
}


int HS2_LaserHokuyo::CloseLaserScanner(void)
{
	if ( HS2_SerialComm::CloseSerialPort() < 0 )
	{
		printf("CloseLaserScanner() Failed!\r\n");
		return -1;
	}

	printf("LaserScanner Disconnected\r\n" );
	return 1;
}



int HS2_LaserHokuyo::FindAndSetBaudRate(void)
{
	char cBufRead[100];
	Sleep(30);
	// read buffer remains
	if( HS2_SerialComm::ReadBufferAll( 100, 1 ) > 99 )
	{
		Sleep(30);
		// case: laser is sending MD response data
		HS2_SerialComm::WriteString( "QT\n", 3 );
		Sleep(30);
		if( HS2_SerialComm::ReadBufferAll( 100, 1 ) > 99 )
		{
			// case: baudrate is not matched
			// please take another action
			Sleep(150);
			CloseLaserScanner();
			Sleep(150);
			OpenLaserScanner( m_iComPort, 19200, 800 );	
			Sleep(150);
			HS2_SerialComm::WriteString( "QT\n", 3 );
			Sleep(30);
			if( HS2_SerialComm::ReadBufferAll( 100, 1 ) > 99 )
			{
				return -1;
			}

			// slient
			Sleep(500);
			// OK. now 19200.
		}
	}
	else
	{
		// case: just bluetooth message

		// slient

		Sleep(500);
		//HS2_SerialComm::ReadBufferAll( 1000, 1 );

		HS2_SerialComm::WriteString( "SCIP2.0\n", 8 );
		Sleep(30);
		HS2_SerialComm::ReadBufferAll( 100, cBufRead, 10, 1 );

		if( strncmp(cBufRead, "SCIP2.0", 7) == 0 )	// 0: identical
		{
			// OK. now 115200.
			return 1;
		}

		// not 115200? change baudrate to 19200
		CloseLaserScanner();
		Sleep(150);
		OpenLaserScanner( m_iComPort, 19200, 800 );	
		Sleep(150);
		// OK. now 19200.
	}

	// 

	HS2_SerialComm::WriteString( "SCIP2.0\n", 8 );
	Sleep(30);
	HS2_SerialComm::ReadBufferAll( 100, cBufRead, 10, 1 );
	Sleep(30);

	if( strncmp(cBufRead, "SCIP2.0", 7) == 0 )	// 0: identical
	{
		HS2_SerialComm::WriteString( "SS115200\n", 9 );
		Sleep(30);
		HS2_SerialComm::ReadBufferAll( 100, cBufRead, 10, 1 );
		Sleep(30);

		if( strncmp(cBufRead, "SS115200", 8) == 0 )	// 0: identical
		{
			CloseLaserScanner();
			Sleep(150);
			OpenLaserScanner( m_iComPort, 115200, 800 );	
			Sleep(150);
		}
		else
		{
			printf("Error: SS115200\r\n");
			return -1;
		}
	}
	else
	{
		printf("Error: SCIP2.0\r\n");
		return -1;
	}

	return 1;
}


int HS2_LaserHokuyo::GetLaserSpecs(void)
{
	//DisableMeasurement_QT();

	//HS2_SerialComm::ReadBufferAll( 100, 1 );

	//HS2_SerialComm::WriteString( "QT\n", 3 );

	//Sleep(500);
	//HS2_SerialComm::ReadBufferAll( 1000, 1 );

	//if( FindAndSetBaudRate() < 0 )
	//	return -1;

	Sleep(30);
	HS2_SerialComm::WriteString( "VV\n", 3 );
	Sleep(30);
	HS2_SerialComm::ReadBufferAll( 1000, 1 );

	Sleep(30);
	HS2_SerialComm::WriteString( "II\n", 3 );
	Sleep(30);
	HS2_SerialComm::ReadBufferAll( 1000, 1 );

	Sleep(30);
	HS2_SerialComm::WriteString( "PP\n", 3 );
	Sleep(30);
	GetLaserParams_PP();
	Sleep(30);
	
	return 1;
}



int HS2_LaserHokuyo::GetLaserParams_PP(void)
{
	unsigned char ucBuf[16 + 64 + 1 + 1 + 1];
	int iLineIndex = 0;

	enum 
	{
		MODL=0,	// Sensor Model
		DMIN,	// Minimum Measurement [mm]
		DMAX,	// Maximum Measurement [mm]
		ARES,	// Total Number of Steps in 360deg range
		AMIN,	// First Step of the Measurement Range
		AMAX,	// Last Step of the Measurement Range
		AFRT,	// Step number on the sensor¡¯s front axis
		SCAN,	// Standard motor speed [rpm]
	};

	int nLineLength;
	for( ; (nLineLength = HS2_SerialComm::ReadStoreLine(ucBuf, 300, 800, 0)) > 0; ++iLineIndex) 
	{
		if(iLineIndex == OtherLine + MODL)
		{
			ucBuf[nLineLength - 2] = '\0';	// for end of string
			for( int k=5; k <= nLineLength-2; k++ )
			{
				m_LaserParam1.Laser_Model[k-5] = ucBuf[k];
			}
		}
		else if(iLineIndex == OtherLine + DMIN) 
		{
			m_LaserParam1.Laser_MinDist = atoi((char*)&ucBuf[5]);	// atoi : ASCII --> integer
		} 
		else if(iLineIndex == OtherLine + DMAX) 
		{
			m_LaserParam1.Laser_MaxDist = atoi((char*)&ucBuf[5]);
		} 
		else if(iLineIndex == OtherLine + ARES) 
		{
			m_LaserParam1.Laser_TotalSteps = atoi((char*)&ucBuf[5]);
		} 
		else if(iLineIndex == OtherLine + AMIN) 
		{
			m_LaserParam1.Laser_FirstStep = atoi((char*)&ucBuf[5]);
		} 
		else if(iLineIndex == OtherLine + AMAX) 
		{
			m_LaserParam1.Laser_LastStep = atoi((char*)&ucBuf[5]);
		} 
		else if(iLineIndex == OtherLine + AFRT) 
		{
			m_LaserParam1.Laser_FrontStep = atoi((char*)&ucBuf[5]);
		} 
		else if(iLineIndex == OtherLine + SCAN) 
		{
			m_LaserParam1.Laser_ScanRPM = atoi((char*)&ucBuf[5]);
		}
	}

	if (iLineIndex <= OtherLine + SCAN) 
	{
		return -1;
	}
	
	m_nNumData = m_LaserParam1.Laser_TotalSteps +1;

	return 0;
}



int HS2_LaserHokuyo::SetLaserParameters(void)
{
	m_LaserParam1.NumTotalMeas = m_LaserParam1.Laser_LastStep -m_LaserParam1.Laser_FirstStep +1;
	m_LaserParam1.LeftEndStep = 0;//LaserParam1.Laser_FirstStep;
	m_LaserParam1.RightEndStep = m_LaserParam1.Laser_LastStep -m_LaserParam1.Laser_FirstStep;
	m_LaserParam1.CenterFrontStep = m_LaserParam1.Laser_FrontStep -m_LaserParam1.Laser_FirstStep;

	if( strncmp("URG-04LX", m_LaserParam1.Laser_Model, 8) == 0 )
	{
		// URG-04LX: 360/1024=0.3515625
		m_LaserParam1.AngleResolution = 0.3515625;	
		m_LaserParam1.LeftEndAngle = -120.0;
		m_LaserParam1.RightEndAngle = 120.0;
		m_LaserParam1.Left90DegStep = m_LaserParam1.LeftEndStep + 85;	// 0+85=85
		m_LaserParam1.Right90DegStep = m_LaserParam1.RightEndStep - 85;	// 681-85=596
	}
	else if( strncmp("UTM-30LX", m_LaserParam1.Laser_Model, 8) == 0 )
	{
		// UTM-30LX: 0.25
		m_LaserParam1.AngleResolution = 0.25;	
		m_LaserParam1.LeftEndAngle = -135.0;
		m_LaserParam1.RightEndAngle = 135.0;
		m_LaserParam1.Left90DegStep = 180;
		m_LaserParam1.Right90DegStep = 900;
	}

	for(int i=m_LaserParam1.LeftEndStep; i<=m_LaserParam1.RightEndStep; i++)
	{
		m_LaserParam1.LUT_CosScanAngle[i] = (float)( cos(D2R*( m_LaserParam1.LeftEndAngle+((double)(i-m_LaserParam1.LeftEndStep)*m_LaserParam1.AngleResolution) )) );
		m_LaserParam1.LUT_SinScanAngle[i] = (float)( sin(D2R*( m_LaserParam1.LeftEndAngle+((double)(i-m_LaserParam1.LeftEndStep)*m_LaserParam1.AngleResolution) )) );
	}
	return 1;
}


int HS2_LaserHokuyo::EnableMeasurement_BM()
{
	Sleep(30);
	HS2_SerialComm::WriteString( "BM\n", 3 );
	Sleep(30);
	int iError_BM = ReceiveEcho_CheckCommand( 100, "BM", 2, 1 );

	if( iError_BM == 0x00 )
		printf("BM:OK\r\n");
	else if( iError_BM == 0x01 )
		printf("BM:Unable to control due to laser malfunction.\r\n");
	else if( iError_BM == 0x02 )
		printf("BM:Laser is already on.\r\n");
	else
		printf("BM:Error code=%d\r\n", iError_BM );

	Sleep(30);
	return 1;
}


int HS2_LaserHokuyo::DisableMeasurement_QT()
{
	Sleep(30);
	HS2_SerialComm::WriteString( "QT\n", 3 );
	Sleep(30);
	int iError_QT = ReceiveEcho_CheckCommand( 100, "QT", 2, 1 );

	if( iError_QT == 0x00 )
		printf("QT:OK\n");
	else
		printf("QT:Error code=%d\n", iError_QT );

	Sleep(30);
	return 1;
}


int HS2_LaserHokuyo::HighSensitivity_HS()
{
	Sleep(30);
	HS2_SerialComm::WriteString( "HS1\n", 4 );
	Sleep(30);
	int iError_HS = ReceiveEcho_CheckCommand( 100, "HS1", 2, 1 );

	if( iError_HS == 0x00 )
		printf("HS:OK\r\n");
	else if( iError_HS == 0x01 )
		printf("HS:Parameter Error.\r\n");
	else if( iError_HS == 0x02 )
		printf("HS:Already running on the set mode.\r\n");
	else if( iError_HS == 0x03 )
		printf("HS:Incompatible with current sensor model.\r\n");
	else
		printf("HS:Error code=%d\r\n", iError_HS );

	Sleep(30);
	return 1;
}


int HS2_LaserHokuyo::ReceiveEcho_CheckCommand( int nMaxRead, const char* ucCmd, int nStringLength, int iPrintConsole )
{
	unsigned char ucBuf[16 + 64 + 1 + 1 + 1];
	int iLineIndex = 0;

	int nLineLength;

	int nStatus = 0;

	for( ; (nLineLength = HS2_SerialComm::ReadStoreLine(ucBuf, 100, 800, 0)) > 0; ++iLineIndex) 
	{
		if(iLineIndex == TagReplyLine)
		{
			if( strncmp((char*)ucBuf, ucCmd, nStringLength) != 0 )	// 0: identical
				break;
		}
		else if(iLineIndex == DataReplyLine) 
		{
			if( IsChecksumOkay(ucBuf, nLineLength) == 0 )
				return -1;	// checksum failed!

			ucBuf[nLineLength-1] = '\0';	// write '\0' at checksum place
			nStatus = atoi((char*)&ucBuf[0]);	// atoi : ASCII --> integer
		} 
		else if(iLineIndex == OtherLine) 
		{
			//	ucBuf[i] = '\0';	// write NULL(0x00) to buffer, and finish.
		}
	}

	return nStatus;
}


int HS2_LaserHokuyo::IsChecksumOkay( const unsigned char *ucBuf, int nLineLength )
{
	unsigned char ucSum = 0x00;
	for( int i=0; i<nLineLength-1; i++ )
	{
		ucSum += ucBuf[i];
	}

	ucSum &= 0x3f;
	ucSum += 0x30;

	if( ucBuf[nLineLength-1] == ucSum )
		return 1;
	else
		return 0;
}

int HS2_LaserHokuyo::CaptureLaserRange( const char *cDataType, int iClusterCount )
{
	char cCmd[20];

	if( cDataType[0] == 'G' )
	{
		sprintf( cCmd, "%c%c%04d%04d%02d\n", cDataType[0], cDataType[1], m_LaserParam1.Laser_FirstStep, m_LaserParam1.Laser_LastStep, iClusterCount);
		HS2_SerialComm::WriteString( cCmd, 13 );
#ifdef __linux__
		Sleep(22);	// need sleep time for linux
#endif
	}
	else if( cDataType[0] == 'M' )
	{
		char cScanInterval = 0;
		short sNumOfScans = 0;
		sprintf( cCmd, "%c%c%04d%04d%02d%01d%02d\n", cDataType[0], cDataType[1], m_LaserParam1.Laser_FirstStep, m_LaserParam1.Laser_LastStep, iClusterCount
			, cScanInterval, sNumOfScans );
		HS2_SerialComm::WriteString( cCmd, 16 );
	}

	return 1;
}


int HS2_LaserHokuyo::GetLaserRangeData_Loop( const char *cDataType, int iClusterCount, unsigned short *usLaserData )
{
	CaptureLaserRange( cDataType, iClusterCount );
	unsigned int uiTimestamp;
	GetLaserRangeData_GDGS( usLaserData, &uiTimestamp );

	return 1;
}



int HS2_LaserHokuyo::GetLaserRangeData_MDMS( unsigned short *usData, unsigned int *uiTimestamp )
{
	int nClusterCounted = 0;
	int nDataByte = 0;
	int nCount = 0;
	m_nRemainByte = 0;	// unless initialize m_nRemainByte=0 every step, laser gets garbage data when it delays.
	//for (int i = LaserParam1.Laser_FirstStep-1; i >= 0; --i) 
	{
	//	usData[nCount++] = 19;
	}

	unsigned char ucBuf[16 + 64 + 1 + 1 + 1];
	int iLineIndex = 0;

	int nLineLength;
	for( ; (nLineLength = HS2_SerialComm::ReadStoreLine(ucBuf, 83, 800, 0)) > 0; ++iLineIndex) 
	{
		if(iLineIndex == 0)
		{
			if( strncmp((char*)ucBuf, "MS", 2) == 0 )	// 0: identical
				nDataByte = 2;
			else if( strncmp((char*)ucBuf, "MD", 2) == 0 )	// 0: identical
				nDataByte = 3;
			else
			{
				printf("Error: Neither MS nor MD.\r\n");
				return -1;
			}

			ucBuf[12] = '\0';	// for end of string
			nClusterCounted = atoi((char*)&ucBuf[10]);
		}
		else if(iLineIndex == 1) 
		{
			if( strncmp((char*)ucBuf, "99b", 3) != 0 )	// 0: identical
			{
				printf("Error: Status %c%c\r\n", ucBuf[0], ucBuf[1]);
				return -1;
			}
		} 
		else if(iLineIndex == 2) 
		{
			*uiTimestamp = DecodeRangeDataUI( (char*)ucBuf, 4 );	// [ms]
			//printf("%x %x %x %x  ", ucBuf[0], ucBuf[1], ucBuf[2], ucBuf[3] );
			//ucBuf[4] = '\0';	// for end of string
		} 
		else if(iLineIndex >= 3) 
		{
			if (nLineLength > (64 + 1)) 
			{
				nLineLength = (64 + 1);
			}
			ucBuf[nLineLength - 1] = '\0';

			int iResult = AddReceivedData((char*)ucBuf, &(m_usDataTemp[0]), &nCount, nDataByte, nClusterCounted);
			if (iResult < 0) 
			{
				return iResult;
			}
		} 
	}

	// laser data order reverse
	for (int i=0; i<=m_LaserParam1.RightEndStep; i++)
	{
		memcpy( &(usData[m_LaserParam1.RightEndStep-i]), &(m_usDataTemp[i]), 2 );
	}
		
	return 0;
}


int HS2_LaserHokuyo::GetLaserRangeData_GDGS( unsigned short *usData, unsigned int *uiTimestamp )
{
	int nClusterCounted = 0;
	int nDataByte = 0;
	m_nRemainByte = 0;	// unless initialize m_nRemainByte=0 every step, laser gets garbage data when it delays.
	
	int nCount = 0;
	//for (int i = LaserParam1.Laser_FirstStep-1; i >= 0; --i) 
	{
	//	usData[nCount++] = 19;
	}

	unsigned char ucBuf[16 + 64 + 1 + 1 + 1];
	int iLineIndex = 0;

	int nLineLength;
	for( ; (nLineLength = HS2_SerialComm::ReadStoreLine(ucBuf, 83, 800, 0)) > 0; ++iLineIndex) 
	{
		if(iLineIndex == 0)
		{
			if( strncmp((char*)ucBuf, "GS", 2) == 0 )	// 0: identical
				nDataByte = 2;
			else if( strncmp((char*)ucBuf, "GD", 2) == 0 )	// 0: identical
				nDataByte = 3;
			else
			{
				printf("Error: Neither GS nor GD.\r\n");
				return -1;
			}

			ucBuf[12] = '\0';	// for end of string
			nClusterCounted = atoi((char*)&ucBuf[10]);
		}
		else if(iLineIndex == 1) 
		{
			if( strncmp((char*)ucBuf, "00P", 3) != 0 )	// 0: identical
			{
				printf("Error: Status %c%c\r\n", ucBuf[0], ucBuf[1]);
				return -1;
			}
		} 
		else if(iLineIndex == 2) 
		{
			*uiTimestamp = DecodeRangeDataUI( (char*)ucBuf, 4 );	// [ms]
			//printf("%x %x %x %x  ", ucBuf[0], ucBuf[1], ucBuf[2], ucBuf[3] );
			//ucBuf[4] = '\0';	// for end of string
		} 
		else if(iLineIndex >= 3) 
		{
			if (nLineLength > (64 + 1)) 
			{
				nLineLength = (64 + 1);
			}
			ucBuf[nLineLength - 1] = '\0';

			int iResult = AddReceivedData((char*)ucBuf, &(m_usDataTemp[0]), &nCount, nDataByte, nClusterCounted);
			if (iResult < 0) 
			{
				return iResult;
			}
		} 
	}

	// laser data order reverse
	for (int i=0; i<=m_LaserParam1.RightEndStep; i++)
	{
		memcpy( &(usData[m_LaserParam1.RightEndStep-i]), &(m_usDataTemp[i]), 2 );
	}

	return 0;
}



int HS2_LaserHokuyo::AddReceivedData( const char ucBuf[], unsigned short usData[], int* nCount, int nDataByte, int nClusterCounted ) 
{	
	//const int nDataByte = 2;
	const char* pPrevBuf = &(ucBuf[0]);
	const char* pCurrBuf = pPrevBuf;

	if(m_nRemainByte > 0) 
	{
		memcpy(&m_cRemainData[m_nRemainByte], ucBuf, nDataByte - m_nRemainByte);
		//usData[LaserParam1.NumTotalMeas-1-*nCount] = DecodeRangeData(m_cRemainData, nDataByte);
		usData[*nCount] = DecodeRangeData(m_cRemainData, nDataByte);
		for( int k=1; k<nClusterCounted; k++ )
		{
			++(*nCount);
			//usData[LaserParam1.NumTotalMeas-1-*nCount] = usData[LaserParam1.NumTotalMeas-*nCount];
			usData[*nCount] = usData[*nCount-1];
		}
		++(*nCount);
		pPrevBuf = &ucBuf[nDataByte - m_nRemainByte];
		pCurrBuf = pPrevBuf;
		m_nRemainByte = 0;
	}

	do 
	{
		++pCurrBuf;
		if( (pCurrBuf-pPrevBuf) >= static_cast<int>(nDataByte) ) 
		{
			//usData[LaserParam1.NumTotalMeas-1-*nCount] = DecodeRangeData(pPrevBuf, nDataByte);
			usData[*nCount] = DecodeRangeData(pPrevBuf, nDataByte);
			for( int k=1; k<nClusterCounted; k++ )
			{
				++(*nCount);
				//usData[LaserParam1.NumTotalMeas-1-*nCount] = usData[LaserParam1.NumTotalMeas-*nCount];
				usData[*nCount] = usData[*nCount-1];
			}
			++(*nCount);
			pPrevBuf = pCurrBuf;
		}
	}
	while (*pCurrBuf != '\0');

	m_nRemainByte = (int)(pCurrBuf - pPrevBuf);
	memcpy(m_cRemainData, pPrevBuf, m_nRemainByte);

	return 1;
}



unsigned short HS2_LaserHokuyo::DecodeRangeData(const char* cData, int nDataByte)	// data_byte = 2 or 3 (GS or GD)
{
	unsigned short usValue = 0;
	for (int i = 0; i < nDataByte; ++i) 
	{
		usValue <<= 6;	// shift 6 bits to left. 1 character
		usValue &= ~0x3f;	// consider only lower 1 character = 6 bits = 0b111111 = 0x3f = 63
		usValue |= cData[i] - 0x30;	// subtract 0x30 from 1 character
	}
	return usValue;
}

unsigned int HS2_LaserHokuyo::DecodeRangeDataUI(const char* cData, int nDataByte)	// data_byte = 4 (Timestamp)
{
	unsigned int uiValue = 0;
	for (int i = 0; i < nDataByte; ++i) 
	{
		uiValue <<= 6;	// shift 6 bits to left. 1 character
		uiValue &= ~0x3f;	// consider only lower 1 character = 6 bits = 0b111111 = 0x3f = 63
		uiValue |= cData[i] - 0x30;	// subtract 0x30 from 1 character
	}
	return uiValue;
}