#pragma once

#include <math.h>
#include "hs2def.h"
#include "HS2_SerialComm.h"

class HS2_LaserHokuyo : HS2_SerialComm
{
public:
	int m_nRemainByte;
	char m_cRemainData[3];

	int m_iComPort;

public:
	int Initiate( int *Laser_On, StructLaserParameters *stLaserParam1, int iComPort );
	int Loop_MD( int *Laser_On, StructLaserdataDownlink *stLaserData1 );
	int Loop_GD( int *Laser_On, StructLaserdataDownlink *stLaserData1 );
	int Terminate();

	int OpenLaserScanner(int iComPort, int nBaudRate, int nReadTimeout );
	int CloseLaserScanner(void);

	int GetLaserSpecs(void);

	int FindAndSetBaudRate(void);

	int GetLaserParams_PP(void);

	int SetLaserParameters(void);

	enum 
	{
		TagReplyLine = 0,
		DataReplyLine,
		OtherLine,
	};

	StructLaserParameters m_LaserParam1;
	int m_nNumData;	// total number of laser data = 726, 1081

	unsigned short m_usDataTemp[1082];	// 1081 original, but it needs 1082 when cluster=2 for duplication

	int EnableMeasurement_BM(void);
	int DisableMeasurement_QT(void);

	int HighSensitivity_HS(void);

	int ReceiveEcho_CheckCommand( int nMaxRead, const char *ucCmd, int nStringLength, int iPrintConsole=0 );

	int IsChecksumOkay( const unsigned char *ucBuf, int nLineLength );

	int GetLaserRangeData_Loop( const char *cDataType, int iClusterCount, unsigned short *usLaserData );
	int CaptureLaserRange( const char *cDataType, int iClusterCount );
	int GetLaserRangeData_GDGS( unsigned short *usData, unsigned int *uiTimestamp );
	int GetLaserRangeData_MDMS( unsigned short *usData, unsigned int *uiTimestamp );

	int AddReceivedData( const char ucBuf[], unsigned short usData[], int* nCount, int nDataByte, int nClusterCounted );

	unsigned short DecodeRangeData(const char* ucData, int nDataByte);	// data_byte = 2 or 3 (GS or GD)
	unsigned int DecodeRangeDataUI(const char* ucData, int nDataByte);	// data_byte = 4 (Timestamp)
};