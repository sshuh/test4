// Sungsik Huh
// 2013.09.23
// 

//#include "stdafx.h"

#include "HS2_SerialComm.h"


int HS2_SerialComm::OpenSerialPort( const int iComPort, const int nBaudRate, const int nReadTimeout )
{
#ifdef _WIN32
	char szSerialPort[30];
	sprintf(szSerialPort, "\\\\.\\COM%d", iComPort);	// \\\\.\\ for COM10~COMXX
	HANDLE hSer;

	// Open Serial Port
	hSer=CreateFile(TEXT(szSerialPort), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hSer==INVALID_HANDLE_VALUE)
	{
		//printf("Init Laser Comm: failed to open COM port\n");
		return -1;
	}

	DCB dcb;
	dcb.DCBlength=sizeof(dcb);
	GetCommState(hSer,&dcb);	// get DCB

	// DCB setting
	dcb.BaudRate=(DWORD)(nBaudRate);
	dcb.fParity=FALSE;
	dcb.fNull=FALSE;
	dcb.StopBits=ONESTOPBIT;
	dcb.Parity=NOPARITY;
	dcb.ByteSize=8;
	SetCommState(hSer,&dcb);
	//SetupComm( m_hSerial, 4096, 4096 ); 

	PurgeComm( m_hSerialComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );

	// set the port to wait indefinitely until a data is available...
	// set up for overlapped I/O
	COMMTIMEOUTS cto;
	GetCommTimeouts(hSer, &cto);

	// Read operation TimeOut
	cto.ReadIntervalTimeout = 0;
	cto.ReadTotalTimeoutConstant = nReadTimeout;
	cto.ReadTotalTimeoutMultiplier = 0;

	// CBR_9600 is approximately 1byte/ms. For our purposes, allow
	// double the expected time per character for a fudge factor.
	cto.WriteTotalTimeoutConstant = 0;
	cto.WriteTotalTimeoutMultiplier = 0;
	
	SetCommTimeouts(hSer, &cto);
	m_hSerialComm = hSer;

	PurgeComm( m_hSerialComm, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );
	return 1;
#endif 

#ifdef __linux__
	char szSerialPort[30] = "/dev/ttyACM0";
	int hSerial;
	struct termios newtio;

	// if you are not superuser, serial port cannot be opened!! try to "sudo su"
	hSerial = open(szSerialPort, O_RDWR | O_NOCTTY | O_NONBLOCK );
	if( hSerial < 0 )
	{
		printf("Init Laser Comm: failed to open COM port %d\r\n", hSerial);
		return 0;
	}
	else
	{
		printf("Init Laser Comm: opened successfully\r\n");
	}

	m_hSerialComm = hSerial;

	//int speed = nBaudRate;
	int nBaudRateDef;
	switch( nBaudRateDef )
	{
	default:
	case 115200:
		nBaudRateDef = B115200;
		break;
	case 19200:
		nBaudRateDef = B19200;
		break;
	case 38400:
		nBaudRateDef = B38400;
		break;
	}

	int parity = 0;
	int handle;
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	newtio.c_cflag = nBaudRateDef  | CS8 | CLOCAL | CREAD | parity;
	newtio.c_iflag = IGNPAR; // | ICRNL; ->IGNORE CARRIGE RETURN, WHICH IS A BIG NO-NO FOR BINARY STREAM
	newtio.c_oflag = ONLCR;//0;
	newtio.c_lflag = 0;//ICANON; // DISABLED ANYTHING SUSPICIOUS AND SEEM UNNECESSARY
	newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
	newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
	newtio.c_cc[VERASE]   = 0;     /* del */
	newtio.c_cc[VKILL]    = 0;     /* @ */
	newtio.c_cc[VEOF]     = 0;     /* Ctrl-d */
	newtio.c_cc[VTIME]    = 8;     /* inter-character timer unused */
	newtio.c_cc[VMIN]     = 0;     /* blocking read until 1 character arrives */
	newtio.c_cc[VSWTC]    = 0;     /* '\0' */
	newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
	newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	newtio.c_cc[VEOL]     = 0;     /* '\0' */
	newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	newtio.c_cc[VEOL2]    = 0;     /* '\0' */
	tcflush(handle, TCIOFLUSH);
	tcsetattr(handle,TCSANOW,&newtio);
	//printf("(s3)");	
	return 1;
#endif 
}


int HS2_SerialComm::CloseSerialPort(void)
{
#ifdef _WIN32
	if( CloseHandle(m_hSerialComm) == true )
	{
		return 1;
	}
	else
	{
		return -1;
	}
#endif 

#ifdef __linux__
	if( close(m_hSerialComm) == 0 )
	{
		return 1;
	}
	else
	{
		return -1;
	}
#endif 
}


int HS2_SerialComm::SetSerialCommParam( const int nReadTimeout )
{
#ifdef _WIN32
	COMMTIMEOUTS cto;
	GetCommTimeouts(m_hSerialComm, &cto);

	// Read doperation TimeOut
	cto.ReadIntervalTimeout = 0;
	cto.ReadTotalTimeoutConstant = nReadTimeout;
	cto.ReadTotalTimeoutMultiplier = 0;

	// CBR_9600 is approximately 1byte/ms. For our purposes, allow
	// double the expected time per character for a fudge factor.
	cto.WriteTotalTimeoutConstant = 0;
	cto.WriteTotalTimeoutMultiplier = 0;
	
	SetCommTimeouts(m_hSerialComm, &cto);
#endif
	return 1;
}


int HS2_SerialComm::WriteString( const char *cCmd, int nLength )
{
#ifdef _WIN32
	//unsigned char ucCmd[] = "SCIP2.0\n";
	DWORD nBytesWritten;
	if( WriteFile(HS2_SerialComm::m_hSerialComm, cCmd, (DWORD)nLength, &nBytesWritten, NULL) == true )
	{
		//	printf("Sent\n");
		return 1;
	}
	else
		return -1;
#endif

#ifdef __linux__
	ssize_t nBytesWritten;
	nBytesWritten = write(m_hSerialComm, cCmd, nLength);
	if( nBytesWritten > 0 )
	{
		return 1;
	}
	else
		return -1;
#endif
}


int HS2_SerialComm::ReadBufferAll( int nMaxRead, int iPrintConsole )
{
#ifdef _WIN32
	DWORD nBytesRead;
	unsigned char ucBuf;
	int iCount;
	for( iCount=0; iCount<nMaxRead; iCount++ )
	{
		ReadFile(HS2_SerialComm::m_hSerialComm, &ucBuf, 1, &nBytesRead, NULL);
		if( nBytesRead < 1 )
			break;

		if( iPrintConsole == 1 )
			printf("%c", ucBuf );
	}
	return iCount;	// returns amount of buffer read
#endif

#ifdef __linux__
	ssize_t nBytesRead;
	unsigned char ucBuf;
	int iCount;
	for( iCount=0; iCount<nMaxRead; iCount++ )
	{
		nBytesRead = read(m_hSerialComm, &ucBuf, 1);
		//ReadFile(HS2_SerialComm::m_hSerialComm, &ucBuf, 1, &nBytesRead, NULL);
		if( nBytesRead < 1 )
			break;

		if( iPrintConsole == 1 )
			printf("%c", ucBuf );
	}
	return iCount;	// returns amount of buffer read
#endif
}


int HS2_SerialComm::ReadBufferAll( int nMaxRead, char* cBufRead, int nNumToRead, int iPrintConsole )
{
#ifdef _WIN32
	DWORD nBytesRead;
	char cBuf;

	int iCount;
	for( iCount=0; iCount<nMaxRead; iCount++ )
	{
		ReadFile(HS2_SerialComm::m_hSerialComm, &cBuf, 1, &nBytesRead, NULL);
		if( nBytesRead < 1 )
			break;

		if( iCount < nNumToRead )
			cBufRead[iCount] = cBuf;

		if( iPrintConsole == 1 )
			printf("%c", cBuf );
	}
	return iCount;	// returns amount of buffer read
#endif

#ifdef __linux__
	ssize_t nBytesRead;
	char cBuf;

	int iCount;
	for( iCount=0; iCount<nMaxRead; iCount++ )
	{
		nBytesRead = read(m_hSerialComm, &cBuf, 1);
		//ReadFile(HS2_SerialComm::m_hSerialComm, &cBuf, 1, &nBytesRead, NULL);
		if( nBytesRead < 1 )
			break;

		if( iCount < nNumToRead )
			cBufRead[iCount] = cBuf;

		if( iPrintConsole == 1 )
			printf("%c", cBuf );
	}
	return iCount;	// returns amount of buffer read
#endif
}



int HS2_SerialComm::ReadBufferToStore( unsigned char *ucBuf, int nMaxRead, int nReadTimeout, int iPrintConsole )
{
#ifdef _WIN32
	int nReadTotal = 0;
	int nReadableInQue = 0;
	
	do // read until read readable_size
	{
		DWORD dwErrors;
		COMSTAT ComStat;
		ClearCommError(m_hSerialComm, &dwErrors, &ComStat);	// retrieves information about serial status
		nReadableInQue = (int)ComStat.cbInQue;	// received, but not yet read by ReadFile()
		int nToRead = min(nMaxRead,nReadableInQue);	// smaller one
		
		DWORD nBytesRead;
		ReadFile(m_hSerialComm, &(ucBuf[nReadTotal]), nToRead, &nBytesRead, NULL);
		nReadTotal += nBytesRead;
		nReadableInQue -= nBytesRead;

		if(nReadTotal >= nMaxRead) 
		{
			return nReadTotal;
		}
	} 
	while (nReadableInQue > 0);

	return nReadTotal;
#endif

#ifdef __linux__
	int nReadTotal = 0;
	int nReadableInQue = 0;
/*
	do // read until read readable_size
	{
		ssize_t dwErrors;
		COMSTAT ComStat;
		ClearCommError(m_hSerialComm, &dwErrors, &ComStat);	// retrieves information about serial status
		nReadableInQue = (int)ComStat.cbInQue;	// received, but not yet read by ReadFile()
		int nToRead = min(nMaxRead,nReadableInQue);	// smaller one
		
		ssize_t nBytesRead;
		nBytesRead = read(m_hSerialComm, &(ucBuf[nReadTotal]), nToRead);
		//ReadFile(m_hSerialComm, &(ucBuf[nReadTotal]), nToRead, &nBytesRead, NULL);
		nReadTotal += nBytesRead;
		nReadableInQue -= nBytesRead;

		if(nReadTotal >= nMaxRead) 
		{
			return nReadTotal;
		}
	} 
	while (nReadableInQue > 0);
*/
	return nReadTotal;
#endif
}



int HS2_SerialComm::ReadStoreLine( unsigned char *ucBuf, int nMaxRead, int nReadTimeout, int iPrintConsole )
{
#ifdef _WIN32
	int i;
	for( i=0; i<(nMaxRead-1); ++i)
	{
		unsigned char ucCharRecv;
		DWORD nBytesRead;
		ReadFile( m_hSerialComm, &(ucCharRecv), 1, &nBytesRead, NULL );
		//int n = com_recv(&recv_ch, 1, Timeout);		// ReadFile from COM Port
		if( nBytesRead <= 0 ) 
		{
			if(i == 0)
			{
				return -1;              // no data to read
			}
			break;
		}
		if( (ucCharRecv == '\r') || (ucCharRecv == '\n') ) // If read CR or LF, write NULL to buffer, and finish.
		{	
			break;
		}
		ucBuf[i] = ucCharRecv;	// If read a character, write it to buffer[i]
	}
	ucBuf[i] = '\0';	// write NULL(0x00) to buffer, and finish.
	return i;	// return amount of data read
#endif

#ifdef __linux__
	int i;
	for( i=0; i<(nMaxRead-1); ++i)
	{
		unsigned char ucCharRecv;
		ssize_t nBytesRead;
		nBytesRead = read(m_hSerialComm, &(ucCharRecv), 1);
		//int n = com_recv(&recv_ch, 1, Timeout);		// ReadFile from COM Port
		if( nBytesRead <= 0 ) 
		{
			if(i == 0)
			{
				return -1;              // no data to read
			}
			break;
		}
		if( (ucCharRecv == '\r') || (ucCharRecv == '\n') ) // If read CR or LF, write NULL to buffer, and finish.
		{	
			break;
		}
		ucBuf[i] = ucCharRecv;	// If read a character, write it to buffer[i]
	}
	ucBuf[i] = '\0';	// write NULL(0x00) to buffer, and finish.
	return i;	// return amount of data read
#endif
}


#ifdef __linux__
ssize_t HS2_SerialComm::ReadBufferAllOnce( int nMaxRead, char* cBufRead, int iPrintConsole )
{
	ssize_t nBytesRead;
	char cBufRead2[300];
	nBytesRead = read(m_hSerialComm, cBufRead2, nMaxRead);
	if( iPrintConsole > 0 )
		printf("%s", cBufRead2);

	return nBytesRead;	// returns amount of buffer read
}
#endif