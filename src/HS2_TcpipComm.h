// Sungsik Huh
// 2013.10.19
// 
#ifndef HS2_TCPIPCOMM_H_
#define HS2_TCPIPCOMM_H_
#endif /* HS2_TCPIPCOMM_H_ */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#define _AFXDLL		// for HS2_Console
#include <afxsock.h>// for HS2_Console

#include <winsock2.h>
#endif 

#ifdef __linux__
#include <unistd.h>	
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define SOCKET	int

#endif


class HS2_TcpipComm
{
public:
// ----------------- Common -------------------
#ifdef _WIN32
	SOCKET m_hServSock;
	SOCKET m_hClntSock;
	int SetSendSocketBuffer( SOCKET hSock, int sndBuf );
	int SetRecvSocketBuffer( SOCKET hSock, int rcvBuf );
#endif 

#ifdef __linux__
	int m_hServSock;
	int m_hClntSock;
	int SetSendSocketBuffer( int hSock, int sndBuf );
	int SetRecvSocketBuffer( int hSock, int rcvBuf );
#endif 

	int CheckSocketBuffer(void);
	int ErrorHandling( char* message );
	char m_cIpAddress[20];
	unsigned short m_usPort;

// ----------------- Tcp Client -------------------
	int SetSocketIPandPort( const char* cIpAddress, const unsigned short usPort );
	int ConnectToTcpServer( const char* cIpAddress, const unsigned short usPort );
	int ReConnectToTcpServer( const char* cIpAddress, const unsigned short usPort );
	int CloseTcpConnection_Client(void);

	int SendDataToServer( const char *cBuf, const int nLength );
	int RecvDataFromServer( char *cBuf, const int nLength );
	int RecvAllBufferFromServer( char *cBuf, const int nLength );	// at CLIENT
	int RecvStoreLine( unsigned char *ucBuf, int nMaxRead, int nReadTimeout, int iPrintConsole=0 );
	int RecvLargeDataFromServer( const char *cBuf, const int nLength, int nSendSize );	// large data


// ----------------- Tcp Server -------------------
	int SetTcpServer( const unsigned short usPort );	
	int WaitForTcpClient(void);
	int SendDataToClient( const char *cBuf, const int nLength );
	int RecvDataFromClient( char *cBuf, const int nLength );
	int SendLargeDataToClient( const char *cBuf, const int nLength, int nSendSize );	// large data
	int CloseTcpServer(void);
	
};
