#ifndef HS2_SERIALCOMM_H_
#define HS2_SERIALCOMM_H_
#endif /* HS2_SERIALCOMM_H_ */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __linux__
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <assert.h>
#endif

class HS2_SerialComm
{
public:
#ifdef _WIN32
	HANDLE m_hSerialComm;
#endif

#ifdef __linux__
	int m_hSerialComm;
	ssize_t ReadBufferAllOnce( int nMaxRead, char* cBufRead, int iPrintConsole=0 );
#endif

	int OpenSerialPort( const int iComPort, const int nBaudRate, const int nReadTimeout );
	int CloseSerialPort(void);

	int SetSerialCommParam( const int nReadTimeout );

	int WriteString( const char *ucCmd, int nLength );
	int ReadBufferAll( int nMaxRead, int iPrintConsole=0 );
	int ReadBufferAll( int nMaxRead, char* cBufRead, int nNumToRead, int iPrintConsole=0 );

	int ReadBufferToStore( unsigned char *unBuf, int nMaxRead, int nReadTimeout, int iPrintConsole=0 );

	int ReadStoreLine( unsigned char *ucBuf, int nMaxRead, int nReadTimeout, int iPrintConsole=0 );


};

