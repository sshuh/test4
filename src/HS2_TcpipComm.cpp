// Sungsik Huh
// 2013.10.19
// 

//#include "stdafx.h"

#include "HS2_TcpipComm.h"





#ifdef _WIN32

int Thread_TcpipClientExample(void)
{
	HS2_TcpipComm tcpip1;

	tcpip1.SetSocketIPandPort( "127.0.0.1", 9190 );

	tcpip1.ConnectToTcpServer( tcpip1.m_cIpAddress, tcpip1.m_usPort );

	while( 1 )
	{
		int nResult;
		char *cBuf = "abc\n";

		nResult = tcpip1.SendDataToServer( cBuf, 4 );

		nResult = tcpip1.RecvDataFromServer( cBuf, 4 );
		
		if( nResult == -1 )
		{
			do
			{
				tcpip1.CloseTcpConnection_Client();

				Sleep(3000);
				
				fprintf( stdout, "Try to ReConnectTcpSocket_Client\n" );
				nResult = tcpip1.ReConnectToTcpServer( tcpip1.m_cIpAddress, tcpip1.m_usPort );
			}
			while( nResult < 0 );
		}
	}

	tcpip1.CloseTcpConnection_Client();

	return 1;
}


int Thread_TcpipServerExample(void)
{
	HS2_TcpipComm tcpip1;

	tcpip1.SetTcpServer( 9190 );
	tcpip1.WaitForTcpClient();

	while( 1 )
	{
		int nResult;
		char *cBuf = "abc\n";

		nResult = tcpip1.SendDataToClient( cBuf, 4 );

		nResult = tcpip1.RecvDataFromClient( cBuf, 4 );
		
	}

	tcpip1.CloseTcpServer();

	return 1;
}
#endif 


#ifdef __linux__

#ifndef Sleep
#define Sleep(x)	usleep(x*1000)
#endif	


#endif



// ------------------------------------------------
// ----------------- Tcp Client -------------------
// ------------------------------------------------

int HS2_TcpipComm::SetSocketIPandPort( const char* cIpAddress, const unsigned short usPort )
{
	strcpy( m_cIpAddress, cIpAddress );
	m_usPort = usPort;

	return 1;
}

int HS2_TcpipComm::ConnectToTcpServer( const char* cIpAddress, const unsigned short usPort )
{
#ifdef _WIN32
	WSADATA wsaData;
	SOCKADDR_IN servAddr;

	if( WSAStartup(MAKEWORD(2,2), &wsaData) != 0 )
	{
		printf("WSAStartup() error!");
		return -1;
	}

	m_hClntSock = socket(PF_INET, SOCK_STREAM, 0);
	if(m_hClntSock == INVALID_SOCKET)
	{
		printf("socket() error");
		return -1;
	}

	memset(&servAddr, 0, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(cIpAddress);
	servAddr.sin_port = htons(usPort);

	if( connect(m_hClntSock, (sockaddr*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR )
	{
		printf("connect() error");
		return -1;
	}

	printf( "TCP Connected\n" );

	return 1;
#endif

#ifdef __linux__
	struct sockaddr_in servAddr;

	if( (m_hClntSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
	{
		perror("socket() error");
		return -1;
	}

	memset( &servAddr, 0, sizeof(servAddr) );
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(cIpAddress);
	servAddr.sin_port = htons( usPort );

	if( connect(m_hClntSock, (struct sockaddr*) &servAddr, sizeof(servAddr)) < 0 )
	{
		perror("connect() error");
		return -1;
	}
	printf("TCP Connected\n");
	return 1;
#endif
}


int HS2_TcpipComm::ReConnectToTcpServer( const char* cIpAddress, const unsigned short usPort )
{
#ifdef _WIN32
	WSADATA wsaData;
	SOCKADDR_IN servAddr;

	if( WSAStartup(MAKEWORD(2,2), &wsaData) != 0 )
	{
		printf("WSAStartup() error!");
		return -1;
	}

	m_hClntSock = socket(PF_INET, SOCK_STREAM, 0);
	if(m_hClntSock == INVALID_SOCKET)
	{
		printf("socket() error");
		return -1;
	}

	memset(&servAddr, 0, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(cIpAddress);
	servAddr.sin_port = htons(usPort);

	if( connect(m_hClntSock, (sockaddr*) &servAddr, sizeof(servAddr)) == SOCKET_ERROR )
	{
		printf("connect() error");
		return -1;
	}
	printf( "TCP Re-Connected\n" );
	return 1;
#endif

#ifdef __linux__
	struct sockaddr_in servAddr;

	if( (m_hClntSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
	{
		perror("socket() error");
		return -1;
	}

	memset( &servAddr, 0, sizeof(servAddr) );
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = inet_addr(cIpAddress);
	servAddr.sin_port = htons( usPort );

	if( connect(m_hClntSock, (struct sockaddr*) &servAddr, sizeof(servAddr)) < 0 )
	{
		perror("connect() error");
		return -1;
	}
	printf("TCP Re-Connected\n");
	return 1;
#endif
}




int HS2_TcpipComm::CloseTcpConnection_Client(void)
{
#ifdef _WIN32
	closesocket(m_hClntSock);
	WSACleanup();
	printf("Client is closed.\n" );
	return 1;
#endif

#ifdef __linux__
	close( m_hClntSock );
	printf("Client is closed.\n");
	return 1;
#endif
}


int HS2_TcpipComm::SendDataToServer( const char *cBuf, const int nLength )	// at CLIENT
{
	//fflush( stdout );
	int nResult = send( m_hClntSock, cBuf, nLength, 0 );
	//printf("(SendResult%d)", nResult );

	return nResult;
}


int HS2_TcpipComm::RecvDataFromServer( char *cBuf, const int nLength )	// at CLIENT
{
	int nResult = recv( m_hClntSock, cBuf, nLength, 0);

	return nResult;
}

int HS2_TcpipComm::RecvAllBufferFromServer( char *cBuf, const int nLength )	// at CLIENT
{
	int nResult = recv( m_hClntSock, cBuf, nLength, MSG_PEEK );

	nResult = recv( m_hClntSock, cBuf, nResult, 0 );

	return nResult;
}


int HS2_TcpipComm::RecvStoreLine( unsigned char *ucBuf, int nMaxRead, int nReadTimeout, int iPrintConsole )
{
	int nResult;
	int iEnd = nMaxRead;
	nResult = recv( m_hClntSock, (char*)ucBuf, nMaxRead, MSG_PEEK );

	if( nResult == 1 )
	{
		recv( m_hClntSock, (char*)ucBuf, 1, 0 );
		return -1;
	}

	for( int i=0; i<nResult; i++ )
	{
		if( (ucBuf[i] == '\r') || (ucBuf[i] == '\n') ) // If read CR or LF, write NULL to buffer, and finish.
		{	
			iEnd = i;
			break;
		}
	}

	nResult = recv( m_hClntSock, (char*)ucBuf, iEnd+1, 0 );
	ucBuf[iEnd] = '\0';	// write NULL(0x00) to buffer, and finish.

	return nResult;	// return amount of data read
}


int HS2_TcpipComm::RecvLargeDataFromServer( const char *cBuf, const int nLength, int nSendSize )	// large data
{
	//int nSendSize = 3000;

	int nResult;//, nTotalLength;
	
	//nResult = recv( m_hClntSock, (char*)(&nTotalLength), sizeof(int), 0);
	//printf("recv1 %d ", nResult);
	//Sleep(1);//Sleep(5);

	unsigned char *iMemPos = (unsigned char*)cBuf;
		
	int nBufRemain = nLength;//nTotalLength;
	int nBufRead;

	while( nBufRemain != 0 )
	{
		int nBufRemain2 = nSendSize;
			
		if( nBufRemain < nSendSize )
			nBufRemain2 = nBufRemain;

		int nBufRead2 = nBufRemain2;
		while( nBufRemain2 != 0 )
		{
			nBufRead = recv( m_hClntSock, (char*)(iMemPos), nBufRemain2, 0);
			nBufRemain2 -= nBufRead;
			iMemPos += nBufRead;
			//Sleep(2);
			//printf("recv2 %d nBufRemain2 %d ", nBufRead, nBufRemain2);
		}

		nBufRemain -= nBufRead2;
		//iMemPos += nBufRead;
		nResult = send( m_hClntSock, (char*)(&nBufRead2), sizeof(int), 0 );
		//printf("Send %d ", nResult);
	}

	return nResult;
}



// ------------------------------------------------
// ----------------- Tcp Server -------------------
// ------------------------------------------------


int HS2_TcpipComm::SetTcpServer( const unsigned short usPort )
{
#ifdef _WIN32
	int nResult;
	//u_short port = 9190;
	WSADATA wsaData;
	SOCKADDR_IN servAddr;
	//int szClntAddr;

	if( WSAStartup(MAKEWORD(2,2), &wsaData) != 0 )
	{
		printf("WSAStartup() error!");
		return -1;
	}

	m_hServSock = socket(PF_INET, SOCK_STREAM, 0);
	if(m_hServSock == INVALID_SOCKET)
	{
		printf("socket() error");
		return -1;
	}

	memset(&servAddr, 0, sizeof(servAddr));
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servAddr.sin_port = htons( usPort );

	int nOptVal;
	nResult = setsockopt( m_hServSock, SOL_SOCKET, SO_REUSEADDR, (char*)&nOptVal, sizeof(nOptVal) );
	printf("(SO_REUSEADDR %d)", nResult );

	if(bind(m_hServSock, (sockaddr*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR )
	{	
		printf("bind() error");
		return -1;
	}

	nResult = listen(m_hServSock, 5);
	if(nResult == SOCKET_ERROR)
	{
		printf("listen() error");
		return -1;
	} 

	nResult = WSAGetLastError();
	printf("(listen%d)", nResult );
	return 1;
#endif

#ifdef __linux__
	int nResult;
	struct sockaddr_in servAddr;

	if( (m_hServSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
	{
		perror("socket() error");
		return -1;
	}

	memset( &servAddr, 0, sizeof(servAddr) );
	servAddr.sin_family = AF_INET;
	servAddr.sin_addr.s_addr = htonl( INADDR_ANY );
	servAddr.sin_port = htons( usPort );

	int nOptVal;
	nResult = setsockopt( m_hServSock, SOL_SOCKET, SO_REUSEADDR, (char*)&nOptVal, sizeof(nOptVal) );
	printf("(SO_REUSEADDR %d)", nResult );

	if( bind(m_hServSock, (struct sockaddr*) &servAddr, sizeof(servAddr)) < 0 )
	{
		perror("bind() error");
		return -1;
	}

	if( listen( m_hServSock, 3 ) < 0 )
	{
		perror("listen() error");
		return -1;
	}
	printf("(listen)" );
	return 1;
#endif
}



int HS2_TcpipComm::WaitForTcpClient(void)
{
#ifdef _WIN32
	int nResult;
	SOCKADDR_IN clntAddr;
	int szClntAddr;

	szClntAddr = sizeof(clntAddr);
	m_hClntSock = accept(m_hServSock, (sockaddr*)&clntAddr, &szClntAddr);
	if( m_hClntSock == INVALID_SOCKET )
	{
		printf("accept() error");
		return -1;
	}

	nResult = WSAGetLastError();
	printf( "TCP Accepted %d\n", nResult );
	return 1;
#endif

#ifdef __linux__
	struct sockaddr_in clntAddr;

	unsigned int len = sizeof( clntAddr );
	if( (m_hClntSock = accept( m_hServSock, (struct sockaddr*)&clntAddr, &len )) < 0 )
	{
		perror("accept() error");
		return -1;
	}
	printf( "TCP Accepted\n" );
#endif
}


int HS2_TcpipComm::SendDataToClient( const char *cBuf, const int nLength )
{
	//fflush( stdout );
	int nResult = send( m_hClntSock, cBuf, nLength, 0 );
	//printf("(SendResult%d)", nResult );

	return nResult;
}


int HS2_TcpipComm::RecvDataFromClient( char *cBuf, const int nLength )
{
	int nResult = recv( m_hClntSock, cBuf, nLength, 0);

	return nResult;
}


int HS2_TcpipComm::SendLargeDataToClient( const char *cBuf, const int nLength, int nSendSize )	// large data
{
	int nTotalLength = nLength;
	unsigned char *iMemPos = (unsigned char*)cBuf;

	//int nSendSize = 3000;
	int nResult;
	nResult = send( m_hClntSock, (char*)(&nTotalLength), sizeof(int), 0 );
	//if( nResult < 0 )
	//	return -1;

	//printf("nTotalLength %d", nTotalLength);
	//printf("Send1 %d ", nResult);
	Sleep(3);

	int nBufRemain = nTotalLength;
	int nBufSent, nBufRead;

	do
	{
		if( nBufRemain < nSendSize )
			nBufSent = send( m_hClntSock, (char*)(iMemPos), nBufRemain, 0 );
		else
			nBufSent = send( m_hClntSock, (char*)(iMemPos), nSendSize, 0 );

		//printf("Send2 %d ", nBufSent);

		//Sleep(10);

		nResult = recv( m_hClntSock, (char*)(&nBufRead), sizeof(int), 0);
		iMemPos += nBufRead;
		nBufRemain -= nBufRead;
		//printf("recv %d nBufRead %d nBufRemain %d ", nResult, nBufRead, nBufRemain);
		Sleep(3);
	}
	while( nBufRemain != 0 );

	return 1;
}


int HS2_TcpipComm::CloseTcpServer(void)
{
#ifdef _WIN32
	closesocket(m_hClntSock);
	closesocket(m_hServSock);
	WSACleanup();
	printf("Server is closed.\n" );
	return 1;
#endif

#ifdef __linux__
	close( m_hClntSock );
	close( m_hServSock );
	printf("Server is closed.\n");
	return 1;
#endif
}


int HS2_TcpipComm::ErrorHandling( char *message )
{
	fputs(message, stderr);
	fputc('\n', stderr);

	return 1;
}


int HS2_TcpipComm::CheckSocketBuffer(void)
{
#ifdef _WIN32
	int sndBuf, rcvBuf;
	int len = sizeof(rcvBuf);
	int state = getsockopt( m_hServSock, SOL_SOCKET, SO_SNDBUF, (char*)&sndBuf, &len );
	if( state==SOCKET_ERROR ) // SOCKET_ERROR==-1
		printf("getsockopt() error");

	state = getsockopt( m_hServSock, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBuf, &len );
	if( state==SOCKET_ERROR )
		printf("getsockopt() error");

	printf("Input buffer size : %d \n", rcvBuf );
	printf("Output buffer size : %d \n", sndBuf );
	return 1;
#endif

#ifdef __linux__
	int sndBuf, rcvBuf;
	unsigned int len = sizeof(rcvBuf);
	int state = getsockopt( m_hServSock, SOL_SOCKET, SO_SNDBUF, (void*)&sndBuf, &len );
	if( state==-1 ) // SOCKET_ERROR==-1
		printf("getsockopt() error");

	state = getsockopt( m_hServSock, SOL_SOCKET, SO_RCVBUF, (void*)&rcvBuf, &len );
	if( state==-1 )
		printf("getsockopt() error");

	printf("Input buffer size : %d \n", rcvBuf );
	printf("Output buffer size : %d \n", sndBuf );
	return 1;
#endif
}


int HS2_TcpipComm::SetSendSocketBuffer( SOCKET hSock, int sndBuf )
{
#ifdef _WIN32
	//int sndBuf = 65535;
	int len = sizeof(sndBuf);
	int state = setsockopt( hSock, SOL_SOCKET, SO_SNDBUF, (char*)&sndBuf, len );
	if( state==SOCKET_ERROR )
		printf("setsockopt() error");

	len = sizeof(sndBuf);
	state = getsockopt( hSock, SOL_SOCKET, SO_SNDBUF, (char*)&sndBuf, &len );
	if( state==SOCKET_ERROR )
		printf("getsockopt() error");

	printf("Input buffer size : %d \n", sndBuf );
	return 1;
#endif

#ifdef __linux__
	//int sndBuf = 65535;
	unsigned int len = sizeof(sndBuf);
	int state = setsockopt( hSock, SOL_SOCKET, SO_SNDBUF, (void*)&sndBuf, len );
	if( state==-1 )
		printf("setsockopt() error");

	len = sizeof(sndBuf);
	state = getsockopt( hSock, SOL_SOCKET, SO_SNDBUF, (void*)&sndBuf, &len );
	if( state==-1 )
		printf("getsockopt() error");

	printf("Input buffer size : %d \n", sndBuf );
	return 1;
#endif
}

int HS2_TcpipComm::SetRecvSocketBuffer( SOCKET hSock, int rcvBuf )
{
#ifdef _WIN32
	//int rcvBuf = 65535;
	int len = sizeof(rcvBuf);
	int state = setsockopt( hSock, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBuf, len );
	if( state==SOCKET_ERROR )
		printf("setsockopt() error");

	len = sizeof(rcvBuf);
	state = getsockopt( hSock, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBuf, &len );
	if( state==SOCKET_ERROR )
		printf("getsockopt() error");

	printf("Input buffer size : %d \n", rcvBuf );

	return 1;
#endif

#ifdef __linux__
	//int rcvBuf = 65535;
	unsigned int len = sizeof(rcvBuf);
	int state = setsockopt( hSock, SOL_SOCKET, SO_RCVBUF, (void*)&rcvBuf, len );
	if( state==-1 )
		printf("setsockopt() error");

	len = sizeof(rcvBuf);
	state = getsockopt( hSock, SOL_SOCKET, SO_RCVBUF, (void*)&rcvBuf, &len );
	if( state==-1 )
		printf("getsockopt() error");

	printf("Input buffer size : %d \n", rcvBuf );

	return 1;
#endif
}





