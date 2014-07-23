//#include "stdafx.h"
#include "HS2_Utils.h"

//----------------------------------------------------------------------------------

int ClassDataProcess::ClearData()
{
	m_usData.clear();
	return 1;
}


int ClassDataProcess::InputData( unsigned short usInput )
{
	int k = m_usData.size();
	for( int i=m_usData.size()-1; i>=0; i-- )
	{
		if( m_usData[i] < usInput )
		{
			break;
		}
		k = i;
	}

	m_usData.insert( m_usData.begin()+k, usInput );
	return 1;
}


int ClassDataProcess::GetMedian( unsigned short *usMedian )
{
	if( m_usData.size() == 0 )
	{
		return 0;
	}
	else if( m_usData.size() == 1 )
	{
		*usMedian = m_usData[0];
	}
	else
	{
		*usMedian = m_usData[m_usData.size()/2];
	}

	return 1;
}


//----------------------------------------------------------------------------------

void PrintMat( char* str, cv::Mat matPrint, cv::Range rowRange, cv::Range colRange )
{
	printf("%s=\n", str);
	int nRows, nCols;

	nRows = matPrint.rows;
	nCols = matPrint.cols;

	if( matPrint.depth() == CV_32F )
	{
		for(int i=rowRange.start; i<rowRange.end; i++ )
		{
			for(int j=colRange.start; j<colRange.end; j++ )
				printf("%3.3f ",matPrint.at<float>(i,j) );

			printf("\n");
		}
	}
	else if( matPrint.depth() == CV_64F )
	{
		for(int i=rowRange.start; i<rowRange.end; i++ )
		{
			for(int j=colRange.start; j<colRange.end; j++ )
				printf("%3.3f ",matPrint.at<double>(i,j) );

			printf("\n");
		}
	}
}


void PrintMat( char* str, cv::Mat matPrint )
{
	printf("%s=\n", str);
	int nRows, nCols;
	nRows = matPrint.rows;
	nCols = matPrint.cols;

	if( matPrint.depth() == CV_32F )
	{
		for(int i=0; i<nRows; i++ )
		{
			for(int j=0; j<nCols; j++ )
				printf("%3.3f ",matPrint.at<float>(i,j) );

			printf("\n");
		}
	}
	else if( matPrint.depth() == CV_64F )
	{
		for(int i=0; i<nRows; i++ )
		{
			for(int j=0; j<nCols; j++ )
				printf("%3.3f ",matPrint.at<double>(i,j) );

			printf("\n");
		}
	}
}


void PrintVecInt( char* str, std::vector <int> vecIntPrint, int nNum )
{
	int nSize = vecIntPrint.size();
	if( nNum >= 0 )
		printf("%s(%d){%d}  ", str, nSize, nNum );
	else
		printf("%s(%d)  ", str, nSize);

	for( int i=0; i<nSize; i++ ) 
		printf("%d ", vecIntPrint[i] );

	printf("\n");
}

//----------------------------------------------------------------------------------

int ClassFastRandomNumberLUT::InitializeUniformLUT( float fMin, float fMax, int nNumDataUniform )
{
	m_arrUniform = new float [nNumDataUniform];
	m_nNumDataUniform = nNumDataUniform;
	m_nIdxUniform = 0;

	cv::RNG RandGenerator;
	for( int i=0; i<nNumDataUniform; i++ )
	{
		m_arrUniform[i] = RandGenerator.uniform( fMin, fMax );
	}
	return 1;
}

int ClassFastRandomNumberLUT::InitializeGaussianLUT( double dSigma, int nNumDataGaussian )
{
	m_arrGaussian = new float [nNumDataGaussian];
	m_nNumDataGaussian = nNumDataGaussian;
	m_nIdxGaussian = 0;

	cv::RNG RandGenerator;
	for( int i=0; i<nNumDataGaussian; i++ )
	{
		m_arrGaussian[i] = RandGenerator.gaussian( dSigma );
	}
	return 1;
}

int ClassFastRandomNumberLUT::TerminateUniformLUT()
{
	delete [] m_arrUniform;
	return 1;
}

int ClassFastRandomNumberLUT::TerminateGaussianLUT()
{
	delete [] m_arrGaussian;
	return 1;
}


float ClassFastRandomNumberLUT::GetUniformRand()
{
	if( (++m_nIdxUniform) == m_nNumDataUniform )
		m_nIdxUniform = 0;

	return m_arrUniform[m_nIdxUniform];
}

float ClassFastRandomNumberLUT::GetGaussianRand()
{
	if( (++m_nIdxGaussian) == m_nNumDataGaussian )
		m_nIdxGaussian = 0;

	return m_arrGaussian[m_nIdxGaussian];
}






//----------------------------------------------------------------------------------
#ifdef _WIN32
/*
int ClassProcessTimeChecker::Initialize( int nNumDataForFreq )
{
	m_arrData = new int [nNumDataForFreq];
	m_nNumDataForFreq = nNumDataForFreq;
	m_nTimePrev = GetTickCount();
	m_nIdx = 0;
	m_CountTimeToPrint = 0;
	return 1;
}

int ClassProcessTimeChecker::Terminate()
{
	delete [] m_arrData;
	return 1;
}

int ClassProcessTimeChecker::GetProcessTime()
{
	m_nTimeCurr = GetTickCount();
	m_nElapsedTime = (int)(m_nTimeCurr-m_nTimePrev);
	m_nTimePrev = m_nTimeCurr;

	//printf("MCL Time: %dms\n", m_nTimeCurr-m_nTimePrev );
	return m_nElapsedTime;
}

float ClassProcessTimeChecker::GetProcessFrequency()
{
	m_nTimeCurr = GetTickCount();
	m_nElapsedTime = (int)(m_nTimeCurr-m_nTimePrev);
	m_nTimePrev = m_nTimeCurr;

	if( (++m_nIdx) == m_nNumDataForFreq )
		m_nIdx = 0;

	m_arrData[m_nIdx] = m_nElapsedTime;

	int nSumOfTime = 0;
	for( int i=0; i<m_nNumDataForFreq; i++ )
		nSumOfTime += m_arrData[i];

	m_fProcessFrequency = 1000.f * ((float)m_nNumDataForFreq) / ((float)nSumOfTime);

	return m_fProcessFrequency;
}
*/
#endif
//----------------------------------------------------------------------------------

int DataPlot::DataPlot_Loop( float fData1, float fData2, float fData3 )
{
	// clear window and redraw axes when the time exceeds the window
	if( m_pDataPlotPrev[0] >= m_imgDataPlot.cols )
	{
		DataPlot_DrawAxes();
		m_pDataPlotPrev[0] = m_nPlotOriginX;
	}

	m_pDataPlotCurr[0] = m_pDataPlotPrev[0] + 2;
	m_pDataPlotCurr[1] = m_nPlotOriginY -(int)(fData1*m_fDataScaleY);
	m_pDataPlotCurr[2] = m_nPlotOriginY -(int)(fData2*m_fDataScaleY);
	m_pDataPlotCurr[3] = m_nPlotOriginY -(int)(fData3*m_fDataScaleY);

	// draw data
	line( m_imgDataPlot, cv::Point(m_pDataPlotPrev[0],m_pDataPlotPrev[1]), cv::Point(m_pDataPlotCurr[0],m_pDataPlotCurr[1]), CV_RGB(255,0,0), 1 );
	line( m_imgDataPlot, cv::Point(m_pDataPlotPrev[0],m_pDataPlotPrev[2]), cv::Point(m_pDataPlotCurr[0],m_pDataPlotCurr[2]), CV_RGB(0,255,0), 1 );
	line( m_imgDataPlot, cv::Point(m_pDataPlotPrev[0],m_pDataPlotPrev[3]), cv::Point(m_pDataPlotCurr[0],m_pDataPlotCurr[3]), CV_RGB(0,0,255), 1 );

	// store current data
	for( int i=0; i<4; i++ )
		m_pDataPlotPrev[i] = m_pDataPlotCurr[i];

	imshow( m_strLabelY, m_imgDataPlot );

	return 1;
}


int DataPlot::DataPlot_Initial( int nWndSizeX, int nWndSizeY, int nPlotOriginX, int nPlotOriginY, float fDataHalfRangeY, char* strLabelY, int iNeedR2D )
{
	// set window size
	m_nWndSizeX = nWndSizeX;
	m_nWndSizeY = nWndSizeY;

	m_imgDataPlot.create( m_nWndSizeY, m_nWndSizeX, CV_8UC3 );
	//for( int i=0; i<4; i++ )
	//	m_pDataPlotPrev[i] = 0;

	m_nPlotOriginX = nPlotOriginX;	// hor origin : time-axis origin
	m_nPlotOriginY = m_nWndSizeY - m_nWndSizeY/2;	// ver origin : data-axis origin

	m_nPlotHalfRangeY = m_nWndSizeY/2 - 40;	// half range of data plot	

	m_pDataPlotPrev[0] = m_nPlotOriginX;	// initial time
	m_pDataPlotPrev[1] = m_nPlotOriginY;	// initial data1
	m_pDataPlotPrev[2] = m_nPlotOriginY;	// initial data2
	m_pDataPlotPrev[3] = m_nPlotOriginY;	// initial data3

	// set x,y labels
	sprintf( m_strLabelX, "Time" );
	sprintf( m_strLabelY, strLabelY );
	sprintf( m_strGridY0, "0" );
	sprintf( m_strGridYp, " %d", (int)(fDataHalfRangeY) );
	sprintf( m_strGridYn, "%d", -(int)(fDataHalfRangeY) );

	m_fDataScaleY = ((float)m_nPlotHalfRangeY/fDataHalfRangeY);	// compute scale to multiply with data
	if( iNeedR2D )
		m_fDataScaleY = m_fDataScaleY * R2DF;	// if the data needs R2D	// for angles

	DataPlot_DrawAxes();

	return 1;
}


int DataPlot::DataPlot_DrawAxes()
{
	m_imgDataPlot.setTo( cv::Scalar(255,255,255) );	// draw background white
	
	// draw x,y labels
	putText( m_imgDataPlot, m_strLabelX, cv::Point(m_nWndSizeX/2,m_nWndSizeY-20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,0), 1 );
	putText( m_imgDataPlot, m_strLabelY, cv::Point(10,20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,0), 1 );

	// draw y-axis scale numbers
	putText( m_imgDataPlot, m_strGridY0, cv::Point(10,m_nPlotOriginY), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,0,0), 1 );
	putText( m_imgDataPlot, m_strGridYp, cv::Point(10,m_nPlotOriginY-m_nPlotHalfRangeY), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,0,0), 1 );
	putText( m_imgDataPlot, m_strGridYn, cv::Point(10,m_nPlotOriginY+m_nPlotHalfRangeY), CV_FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,0,0), 1 );

	// draw x-axis lines
	line( m_imgDataPlot, cv::Point(m_nPlotOriginX,m_nPlotOriginY), cv::Point(m_nWndSizeX-10,m_nPlotOriginY), CV_RGB(180,180,180), 1 );
	line( m_imgDataPlot, cv::Point(m_nPlotOriginX,m_nPlotOriginY-m_nPlotHalfRangeY), cv::Point(m_nWndSizeX-10,m_nPlotOriginY-m_nPlotHalfRangeY), CV_RGB(180,180,180), 1 );
	line( m_imgDataPlot, cv::Point(m_nPlotOriginX,m_nPlotOriginY+m_nPlotHalfRangeY), cv::Point(m_nWndSizeX-10,m_nPlotOriginY+m_nPlotHalfRangeY), CV_RGB(180,180,180), 1 );

	// draw y-axis lines
	line( m_imgDataPlot, cv::Point(m_nPlotOriginX,m_nPlotOriginY+m_nPlotHalfRangeY), cv::Point(m_nPlotOriginX,m_nPlotOriginY-m_nPlotHalfRangeY), CV_RGB(180,180,180), 1 );

	return 1;
}


//----------------------------------------------------------------------------------

int KalmanFilter_Nav3D::Initialize( float fP, float fQ, float fR )
{
	for( int i=0; i<3; i++ )
	{
		m_arrStatePrev[i] = 0.f;
		m_arrStateCurr[i] = 0.f;
		m_fMatP[i] = fP;
	}

	m_fQ = fQ;
	m_fR = fR;

	m_fDt = 0.02f;

	return 1;
}

int KalmanFilter_Nav3D::Prediction( float fMeasAccB[3], float fEulerAtt[3] )
{
	// F = Identity, B = zero

	// Convert Acc in body frame to ned frame
	float sin_phi = sin(fEulerAtt[0]);	
	float cos_phi = cos(fEulerAtt[0]);
	float sin_theta = sin(fEulerAtt[1]);	
	float cos_theta = cos(fEulerAtt[1]);	
	float sin_psi = sin(fEulerAtt[2]);
	float cos_psi = cos(fEulerAtt[2]);
	float arrRot[3][3];

	arrRot[0][0] = cos_theta*cos_psi;							arrRot[0][1] = cos_theta*sin_psi;							arrRot[0][2] = -sin_theta;
	arrRot[1][0] = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;	arrRot[1][1] = sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;	arrRot[1][2] = sin_phi*cos_theta;
	arrRot[2][0] = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;	arrRot[2][1] = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;	arrRot[2][2] = cos_phi*cos_theta;

	float fMeasAccNed[3];
	for( int i=0; i<3; i++ )
		fMeasAccNed[i] = arrRot[0][i]*fMeasAccB[0] +arrRot[1][i]*fMeasAccB[1] +arrRot[2][i]*fMeasAccB[2];
/*
	asdfasdfasdfasdfasdf
*/

	// x = F*x + B*u
	for( int i=0; i<3; i++ )
	{
		m_arrStateCurr[i] = m_arrStatePrev[i] + m_arrStatePrev[i+3]*m_fDt;
	}



	for( int i=0; i<3; i++ )
	{
		m_arrStateCurr[i+3] = m_arrStatePrev[i+3] + m_arrStatePrev[i+3];
	}

	// P = F*P*Ft + Q
	for( int i=0; i<3; i++ )
		m_fMatP[i] = m_fMatP[i] + m_fQ;

	return 1;
}

int KalmanFilter_Nav3D::MeasurementUpdate( float fMeasAccB[3], float fEulerAtt[3] )
{
/*	float fMeasAccNed[3];

	// Convert Acc in body frame to ned frame
	float sin_phi = sin(fEulerAtt[0]);	
	float cos_phi = cos(fEulerAtt[0]);
	float sin_theta = sin(fEulerAtt[1]);	
	float cos_theta = cos(fEulerAtt[1]);	
	float sin_psi = sin(fEulerAtt[2]);
	float cos_psi = cos(fEulerAtt[2]);
	float arrRot[3][3];

	arrRot[0][0] = cos_theta*cos_psi;							arrRot[0][1] = cos_theta*sin_psi;							arrRot[0][2] = -sin_theta;
	arrRot[1][0] = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;	arrRot[1][1] = sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;	arrRot[1][2] = sin_phi*cos_theta;
	arrRot[2][0] = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;	arrRot[2][1] = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;	arrRot[2][2] = cos_phi*cos_theta;

	for( int i=0; i<3; i++ )
		fMeasAccNed[i] = arrRot[0][i]*fMeasAccB[0] +arrRot[1][i]*fMeasAccB[1] +arrRot[2][i]*fMeasAccB[2];

	//fMeasAccNed[0] = fMeasAccB[0];
	//fMeasAccNed[1] = fMeasAccB[1];
	//fMeasAccNed[2] = fMeasAccB[2];

	// gravity compensation
	fMeasAccNed[2] = fMeasAccNed[2] + 9.81f;

	float fMatK[3];
	// H = Identity mat
	float fMatSinv[3];


	// S = H*P*Ht + R
	// K = P*Ht*Sinv
	for( int i=0; i<3; i++ )
	{
		fMatSinv[i] = 1.f/(m_fMatP[i]+m_fR);
		fMatK[i] = m_fMatP[i]*fMatSinv[i];
	}

	// x = x + K*(z-H*x)
	for( int i=0; i<3; i++ )
		m_fStateAcc[i] = m_fStateAcc[i] + fMatK[i]*(fMeasAccNed[i] -m_fStateAcc[i]);

	// P = (I-K*H)*P
	for( int i=0; i<3; i++ )
		m_fMatP[i] = (1.f -fMatK[i])*m_fMatP[i];
*/
	return 1;
}
//----------------------------------------------------------------------------------




//----------------------------------------------------------------------------------

int KalmanFilter_Accel3D::Initialize( float fP, float fQ, float fR )
{
	for( int i=0; i<3; i++ )
	{
		m_fStateAcc[i] = 0.f;
		m_fMatP[i] = fP;
	}

	m_fQ = fQ;
	m_fR = fR;

	return 1;
}

int KalmanFilter_Accel3D::Prediction()
{
	// F = Identity, B = zero

	// x = F*x + B*u
	//for( int i=0; i<3; i++ )
	//	m_fStateAcc[i] = m_fStateAcc[i];

	// P = F*P*Ft + Q
	for( int i=0; i<3; i++ )
		m_fMatP[i] = m_fMatP[i] + m_fQ;

	return 1;
}


int KalmanFilter_Accel3D::MeasurementUpdate( float fMeasAccB[3], float fEulerAtt[3] )
{
	float fMeasAccNed[3];

	// Convert Acc in body frame to ned frame
	float sin_phi = sin(fEulerAtt[0]);	
	float cos_phi = cos(fEulerAtt[0]);
	float sin_theta = sin(fEulerAtt[1]);	
	float cos_theta = cos(fEulerAtt[1]);	
	float sin_psi = sin(fEulerAtt[2]);
	float cos_psi = cos(fEulerAtt[2]);
	float arrRot[3][3];

	arrRot[0][0] = cos_theta*cos_psi;							arrRot[0][1] = cos_theta*sin_psi;							arrRot[0][2] = -sin_theta;
	arrRot[1][0] = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;	arrRot[1][1] = sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;	arrRot[1][2] = sin_phi*cos_theta;
	arrRot[2][0] = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;	arrRot[2][1] = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;	arrRot[2][2] = cos_phi*cos_theta;

	for( int i=0; i<3; i++ )
		fMeasAccNed[i] = arrRot[0][i]*fMeasAccB[0] +arrRot[1][i]*fMeasAccB[1] +arrRot[2][i]*fMeasAccB[2];

	//fMeasAccNed[0] = fMeasAccB[0];
	//fMeasAccNed[1] = fMeasAccB[1];
	//fMeasAccNed[2] = fMeasAccB[2];

	// gravity compensation
	fMeasAccNed[2] = fMeasAccNed[2] + 9.81f;

	float fMatK[3];
	// H = Identity mat
	float fMatSinv[3];


	// S = H*P*Ht + R
	// K = P*Ht*Sinv
	for( int i=0; i<3; i++ )
	{
		fMatSinv[i] = 1.f/(m_fMatP[i]+m_fR);
		fMatK[i] = m_fMatP[i]*fMatSinv[i];
	}

	// x = x + K*(z-H*x)
	for( int i=0; i<3; i++ )
		m_fStateAcc[i] = m_fStateAcc[i] + fMatK[i]*(fMeasAccNed[i] -m_fStateAcc[i]);

	// P = (I-K*H)*P
	for( int i=0; i<3; i++ )
		m_fMatP[i] = (1.f -fMatK[i])*m_fMatP[i];

	return 1;
}
//----------------------------------------------------------------------------------

SinCosEuler::SinCosEuler( float fRoll, float fPitch, float fYaw )
{
	sin_phi = sin(fRoll);	sin_theta = sin(fPitch);	sin_psi = sin(fYaw);
	cos_phi = cos(fRoll);	cos_theta = cos(fPitch);	cos_psi = cos(fYaw);

	tan_theta = tan(fPitch);
}

SinCosEuler::SinCosEuler( float fRoll, float fPitch, float fYaw, float fX, float fY, float fZ )
{
	sin_phi = sin(fRoll);	sin_theta = sin(fPitch);	sin_psi = sin(fYaw);
	cos_phi = cos(fRoll);	cos_theta = cos(fPitch);	cos_psi = cos(fYaw);

	tan_theta = tan(fPitch);

	trans[0] = fX;	trans[1] = fY;	trans[2] = fZ;
}

SinCosEuler::SinCosEuler( double dRoll, double dPitch, double dYaw, double dX, double dY, double dZ )
{
	sin_phi = sin((float)dRoll);	sin_theta = sin((float)dPitch);	sin_psi = sin((float)dYaw);
	cos_phi = cos((float)dRoll);	cos_theta = cos((float)dPitch);	cos_psi = cos((float)dYaw);

	tan_theta = tan((float)dPitch);

	trans[0] = (float)dX;	trans[1] = (float)dY;	trans[2] = (float)dZ;
}


void SinCosEuler::Euler2Rot( cv::Mat* matRot )
{

	matRot->at<float>(0,0) = cos_theta*cos_psi;								matRot->at<float>(0,1) = cos_theta*sin_psi;								matRot->at<float>(0,2) = -sin_theta;
	matRot->at<float>(1,0) = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;	matRot->at<float>(1,1) = sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;	matRot->at<float>(1,2) = sin_phi*cos_theta;
	matRot->at<float>(2,0) = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;	matRot->at<float>(2,1) = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;	matRot->at<float>(2,2) = cos_phi*cos_theta;
}


void SinCosEuler::Euler2Rot( float *arrRot[3][3] )
{
	*arrRot[0][0] = cos_theta*cos_psi;								*arrRot[0][1] = cos_theta*sin_psi;								*arrRot[0][2] = -sin_theta;
	*arrRot[1][0] = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;	*arrRot[1][1] = sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;	*arrRot[1][2] = sin_phi*cos_theta;
	*arrRot[2][0] = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;	*arrRot[2][1] = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;	*arrRot[2][2] = cos_phi*cos_theta;
}




void SinCosEuler::Euler2TfmD( cv::Mat* matRot )	// Euler to transformation matrix // double CV_64FC1
{
	matRot->at<double>(0,0) = cos_theta*cos_psi;							matRot->at<double>(0,1) = cos_theta*sin_psi;							matRot->at<double>(0,2) = -sin_theta;
	matRot->at<double>(1,0) = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;	matRot->at<double>(1,1) = sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;	matRot->at<double>(1,2) = sin_phi*cos_theta;
	matRot->at<double>(2,0) = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;	matRot->at<double>(2,1) = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;	matRot->at<double>(2,2) = cos_phi*cos_theta;

	matRot->at<double>(3,0) = 0.0;	matRot->at<double>(3,1) = 0.0;	matRot->at<double>(3,2) = 0.0;	matRot->at<double>(3,3) = 1.0;

	for( int i=0; i<3; i++ )
	{
		matRot->at<double>(i,3) = 0.0;
		for( int k=0; k<3; k++ )
			matRot->at<double>(i,3) -= matRot->at<double>(i,k)*trans[k];
	}
}


void SinCosEuler::Euler2InvTfmD( cv::Mat* matRot )	// Euler to inv. transformation matrix // double CV_64FC1
{
	matRot->at<double>(0,0) = cos_theta*cos_psi;	matRot->at<double>(0,1) = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;	matRot->at<double>(0,2) = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;
	matRot->at<double>(1,0) = cos_theta*sin_psi;	matRot->at<double>(1,1) = sin_phi*sin_theta*sin_psi + cos_phi*cos_psi;	matRot->at<double>(1,2) = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;
	matRot->at<double>(2,0) = -sin_theta;			matRot->at<double>(2,1) = sin_phi*cos_theta;								matRot->at<double>(2,2) = cos_phi*cos_theta;

	matRot->at<double>(3,0) = 0.0;	matRot->at<double>(3,1) = 0.0;	matRot->at<double>(3,2) = 0.0;	matRot->at<double>(3,3) = 1.0;

	matRot->at<double>(0,3) = trans[0];		matRot->at<double>(1,3) = trans[1];		matRot->at<double>(2,3) = trans[2];		
}


void SinCosEuler::Euler2Rot_dEuler( cv::Mat* matRot_dphi, cv::Mat* matRot_dtheta, cv::Mat* matRot_dpsi )
{
	matRot_dphi->at<float>(0,0) = 0.0f;											matRot_dphi->at<float>(0,1) = 0.0f;											matRot_dphi->at<float>(0,2) = 0.0f;
	matRot_dphi->at<float>(1,0) = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;	matRot_dphi->at<float>(1,1) = cos_phi*sin_theta*sin_psi - sin_phi*cos_psi;	matRot_dphi->at<float>(1,2) = cos_phi*cos_theta;
	matRot_dphi->at<float>(2,0) = -sin_phi*sin_theta*cos_psi + cos_phi*sin_psi;	matRot_dphi->at<float>(2,1) = -sin_phi*sin_theta*sin_psi - cos_phi*cos_psi;	matRot_dphi->at<float>(2,2) = -sin_phi*cos_theta;

	matRot_dtheta->at<float>(0,0) = -sin_theta*cos_psi;				matRot_dtheta->at<float>(0,1) = -sin_theta*sin_psi;				matRot_dtheta->at<float>(0,2) = -cos_theta;
	matRot_dtheta->at<float>(1,0) = sin_phi*cos_theta*cos_psi;		matRot_dtheta->at<float>(1,1) = sin_phi*cos_theta*sin_psi;		matRot_dtheta->at<float>(1,2) = -sin_phi*sin_theta;
	matRot_dtheta->at<float>(2,0) = cos_phi*cos_theta*cos_psi;		matRot_dtheta->at<float>(2,1) = cos_phi*cos_theta*sin_psi;		matRot_dtheta->at<float>(2,2) = -cos_phi*sin_theta;

	matRot_dpsi->at<float>(0,0) = -cos_theta*sin_psi;								matRot_dpsi->at<float>(0,1) = cos_theta*cos_psi;								matRot_dpsi->at<float>(0,2) = 0.0f;
	matRot_dpsi->at<float>(1,0) = -sin_phi*sin_theta*sin_psi - cos_phi*cos_psi;		matRot_dpsi->at<float>(1,1) = sin_phi*sin_theta*cos_psi - cos_phi*sin_psi;		matRot_dpsi->at<float>(1,2) = 0.0f;
	matRot_dpsi->at<float>(2,0) = -cos_phi*sin_theta*sin_psi + sin_phi*cos_psi;		matRot_dpsi->at<float>(2,1) = cos_phi*sin_theta*cos_psi + sin_phi*sin_psi;		matRot_dpsi->at<float>(2,2) = 0.0f;
}


void SinCosEuler::Euler2AngRateMat( cv::Mat* matRotRate )
{
	matRotRate->at<float>(0,0) = 1.0f;		matRotRate->at<float>(0,1) = sin_phi*tan_theta;		matRotRate->at<float>(0,2) = cos_phi*tan_theta;
	matRotRate->at<float>(1,0) = 0.0f;		matRotRate->at<float>(1,1) = cos_phi;				matRotRate->at<float>(1,2) = -sin_phi;
	matRotRate->at<float>(2,0) = 0.0f;		matRotRate->at<float>(2,1) = sin_phi/cos_theta;		matRotRate->at<float>(2,2) = cos_phi/cos_theta;
}

//----------------------------------------------------------------------------------

#ifdef _WIN32
/*
char* GetDATfromFileDialog()
{
	char *readFileName = (char *) NULL;
	
	char szFilter[] = "DAT file (*.dat)|*.dat|";  // view selected file extension through szfilter 

	CFileDialog fileDlg(TRUE,	// read
							  "dat",  // file extension(확장자)
							  NULL,
							  OFN_EXPLORER|OFN_HIDEREADONLY,
							  szFilter
							  );

	if( fileDlg.DoModal() == IDOK )
	{
		//CString to char *
		readFileName = (char *) strdup ((char *) (LPCTSTR) fileDlg.GetPathName());
	}

	return readFileName;
}

char* GetAVIfromFileDialog()
{
	char *readFileName = (char *) NULL;
	
	char szFilter[] = "AVI file (*.avi)|*.avi|";  // view selected file extension through szfilter 

	CFileDialog fileDlg(TRUE,	// read
							  "avi",  // file extension(확장자)
							  NULL,
							  OFN_EXPLORER|OFN_HIDEREADONLY,
							  szFilter
							  );

	if( fileDlg.DoModal() == IDOK )
	{
		//CString to char *
		readFileName = (char *) strdup ((char *) (LPCTSTR) fileDlg.GetPathName());
	}

	return readFileName;
}
*/
#endif