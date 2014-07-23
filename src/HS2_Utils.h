#pragma once

#include <stdio.h>

#ifdef _WIN32
#include "OpenCV2.4.3/include/opencv2/opencv.hpp"
#endif

#ifdef __linux__
#include "opencv2/opencv.hpp"
#endif

#ifndef PI
	#define PI		3.14159265358979
#endif
#ifndef PIF
	#define PIF		3.14159265358979f
#endif
#ifndef D2R
	#define D2R	0.01745329251994327	//(PI/180.)
#endif
#ifndef R2D
	#define R2D	57.2957795130823799	//(180./PI)
#endif
#ifndef D2RF
	#define D2RF	0.01745329251994327f	//(PIf/180.f)
#endif
#ifndef R2DF
	#define R2DF	57.2957795130823799f	//(180.f/PIf)
#endif

#define AT_D( matX, i, j )		(*(double*)(&((matX).data[(i)*(matX).cols*8+(j)*8])))
#define AT_F( matX, i, j )		(*(float*)(&((matX).data[(i)*(matX).cols*4+(j)*4])))
#define AT_UC( matX, i, j )		(*(&((matX).data[(i)*(matX).cols+(j)])))
#define AT_COLOR_R( imgX, i, j )		(*(&((imgX).data[(i)*(imgX).cols*3+(j)*3+0])))
#define AT_COLOR_G( imgX, i, j )		(*(&((imgX).data[(i)*(imgX).cols*3+(j)*3+1])))
#define AT_COLOR_B( imgX, i, j )		(*(&((imgX).data[(i)*(imgX).cols*3+(j)*3+2])))

//----------------------------------------------------------------------------------
class ClassDataProcess
{
public:
	std::vector <unsigned short> m_usData;

	int ClearData();
	int InputData( unsigned short usInput );
	int GetMedian( unsigned short *usMedian );
};



//----------------------------------------------------------------------------------
void PrintMat( char* str, cv::Mat matPrint, cv::Range rowRange, cv::Range colRange );
void PrintMat( char* str, cv::Mat matPrint );
//void PrintMat( char* str, Mat matPrint, bool IsThisInteger, int bytes );
void PrintVecInt( char* str, std::vector <int> vecIntPrint, int nNum=-1 );


//----------------------------------------------------------------------------------
class ClassFastRandomNumberLUT
{
public:
	int InitializeUniformLUT( float fMin=-1.0f, float fMax=1.0f, int nNumDataUniform=300 );
	int InitializeGaussianLUT( double dSigma=1.0, int nNumDataGaussian=300 );
	int TerminateUniformLUT();
	int TerminateGaussianLUT();

	float GetUniformRand();
	float GetGaussianRand();
protected:
	float *m_arrUniform;
	float *m_arrGaussian;
	
	int m_nIdxUniform;
	int m_nIdxGaussian;
	int m_nNumDataUniform;
	int m_nNumDataGaussian;

};


//----------------------------------------------------------------------------------
class ClassProcessTimeChecker
{
public:
	int Initialize( int nNumDataForFreq=10 );
	int Terminate();

	int GetProcessTime();
	float GetProcessFrequency();
	int m_CountTimeToPrint;
protected:
	int m_nIdx;
	int m_nNumDataForFreq;
	int *m_arrData;
	int m_nElapsedTime;
	unsigned long m_nTimePrev, m_nTimeCurr;
	//float m_fTimeItv;
	float m_fProcessFrequency;

};

//----------------------------------------------------------------------------------

class DataPlot
{
public:
	int DataPlot_Initial( int nWndSizeX, int nWndSizeY, int nPlotOriginX, int nPlotOriginY, float fDataScaleY, char* strLabelY, int iNeedR2D=0 );
	int DataPlot_Loop( float fData1, float fData2, float fData3 );
	int DataPlot_DrawAxes();// Mat imgDataPlot, char* strLabelX, char* strLabelY, char* strGridYp, char* strGridYn, int nPlotOffsetX, int nPlotOffsetY );

	cv::Mat m_imgDataPlot;

	int m_nWndSizeX;
	int m_nWndSizeY;
	int m_pDataPlotPrev[4];
	int m_pDataPlotCurr[4];
	int m_nPlotOriginX;// hor
	int m_nPlotOriginY;// ver1

	int m_nPlotHalfRangeY;

/*	std::string m_strLabelX;
	std::string m_strLabelY;
	std::string m_strGridYp;
	std::string m_strGridY0;
	std::string m_strGridYn;
*/
	char m_strLabelX[10];
	char m_strLabelY[10];
	char m_strGridYp[10];
	char m_strGridY0[10];
	char m_strGridYn[10];

	float m_fDataScaleY;
	int m_iR2D;
};


//----------------------------------------------------------------------------------

class KalmanFilter_Nav3D
{
public:
	int Initialize( float fP, float fQ, float fR );
	int Prediction( float fMeasAccB[3], float fEulerAtt[3] );
	int MeasurementUpdate( float fMeasAcc[3], float fEulerAtt[3] );

	float m_fQ;
	float m_fR;
	float m_fMatP[3];

	float m_arrStatePrev[6];
	float m_arrStateCurr[6];
	float m_fDt;

};
//----------------------------------------------------------------------------------

class KalmanFilter_Accel3D
{
public:
	int Initialize( float fP, float fQ, float fR );
	int Prediction();
	int MeasurementUpdate( float fMeasAcc[3], float fEulerAtt[3] );

	float m_fQ;
	float m_fR;
	float m_fMatP[3];

	float m_fStateAcc[3];

};

//----------------------------------------------------------------------------------

class SinCosEuler
{
public:
	float sin_phi, sin_theta, sin_psi;
	float cos_phi, cos_theta, cos_psi;
	float tan_theta;
	float trans[3];

	SinCosEuler( float fRoll, float fPitch, float fYaw );
	SinCosEuler( float fRoll, float fPitch, float fYaw, float fX, float fY, float fZ );
	SinCosEuler( double dRoll, double dPitch, double dYaw, double dX, double dY, double dZ );

	void Euler2Rot( cv::Mat* matRot );
	void Euler2Rot( float *arrRot[3][3] );

	void Euler2Rot_dEuler( cv::Mat* matRot_dphi, cv::Mat* matRot_dtheta, cv::Mat* matRot_dpsi );

	void Euler2TfmD( cv::Mat* matRot );	// double CV_64FC1
	void Euler2InvTfmD( cv::Mat* matRot );	// double CV_64FC1

	void Euler2AngRateMat( cv::Mat* matRotRate );
};

//----------------------------------------------------------------------------------


char* GetDATfromFileDialog();
char* GetAVIfromFileDialog();
