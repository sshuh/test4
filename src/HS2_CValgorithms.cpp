#ifdef _WIN32
//#include "stdafx.h"
#endif

#include "HS2_CValgorithms.h"


int ClassExtractColor::Initiate( cv::Mat imgSrcColor )
{
	m_imgBGR[0].create( imgSrcColor.size(), CV_8UC1 );
	m_imgBGR[1].create( imgSrcColor.size(), CV_8UC1 );
	m_imgBGR[2].create( imgSrcColor.size(), CV_8UC1 );

	m_imgTemp.create( imgSrcColor.size(), CV_8UC1 );
	m_imgRofB.create( imgSrcColor.size(), CV_8UC1 );
	m_imgGofB.create( imgSrcColor.size(), CV_8UC1 );
	m_imgRofS.create( imgSrcColor.size(), CV_8UC1 );
	m_imgDilated.create( imgSrcColor.size(), CV_8UC1 );
	m_imgThresholded.create( imgSrcColor.size(), CV_8UC1 );

	return 1;
}


int ClassExtractColor::ExtractColor_Red( cv::Mat imgSrcColor, cv::Mat imgResultBinary )
{
	cv::split( imgSrcColor, m_imgBGR );

//	imshow( "R", imgBGR[2] );
//	imshow( "G", imgBGR[1] );
//	imshow( "B", imgBGR[0] );
/*
	// R >= B*1.5
	cv::multiply( m_imgBGR[2], 0.5, m_imgTemp );
	cv::compare( m_imgTemp, m_imgBGR[0], m_imgRofB, cv::CMP_GE );
	imshow( "m_imgRofB", m_imgRofB );

	// B >= G*1.5
	cv::multiply( m_imgBGR[0], 1.2, m_imgTemp );
	cv::compare( m_imgTemp, m_imgBGR[1], m_imgGofB, cv::CMP_GE );
	imshow( "m_imgGofB", m_imgGofB );

	// R >= 20
	cv::compare( m_imgBGR[2], 20.0, m_imgRofS, cv::CMP_GE );
	imshow( "m_imgRofS", m_imgRofS );
*/

	// R >= G*1.5
	cv::multiply( m_imgBGR[2], 0.7, m_imgTemp );
	cv::compare( m_imgTemp, m_imgBGR[1], m_imgRofB, cv::CMP_GE );
	imshow( "m_imgRofB", m_imgRofB );

	// G >= B*1.5
	cv::multiply( m_imgBGR[1], 0.7, m_imgTemp );
	cv::compare( m_imgTemp, m_imgBGR[0], m_imgGofB, cv::CMP_GE );
	imshow( "m_imgGofB", m_imgGofB );

	// R >= 20
	cv::compare( m_imgBGR[2], 50.0, m_imgRofS, cv::CMP_GE );
	imshow( "m_imgRofS", m_imgRofS );
	

	cv::bitwise_and( m_imgRofB, m_imgGofB, m_imgTemp );
	cv::bitwise_and( m_imgTemp, m_imgRofS, m_imgThresholded );
	imshow( "m_imgThresholded", m_imgThresholded );

	int dilation_size = 1;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );
	//dilate( m_imgThresholded, m_imgDilated, element );
	//erode(m_imgDilated, m_imgThresholded, element );
	
	cv::erode( m_imgThresholded, m_imgDilated, element );
	cv::dilate( m_imgDilated, imgResultBinary, element );

	//m_imgThresholded2 = Mat::zeros( m_imgThresholded2.size(), m_imgThresholded2.type() );
	imshow( "imgResultBinary", imgResultBinary );

	return 1;
}



// -------------------------------------------------------------------------------------------------

int ClassContourProcess::Initiate( cv::Mat imgSrc )
{
	m_imgContours.create( imgSrc.size(), CV_8UC1 );

	return 1;
}


int ClassContourProcess::ExtractContours( cv::Mat imgSrcBinary )
{
	double dMinArea = 15.0, dMaxArea = 500.0*500.0;
	CvRect rectContour;
	int nNumContours=0;
	double dContourArea, dAspectRatio, dCx, dCy;
	int iMarker=0;
	
	m_imgContours = cv::Mat::zeros( m_imgContours.size(), m_imgContours.type() );

	std::vector< std::vector<cv::Point> > vContours;
	std::vector<cv::Vec4i> vHierarchy;

	//  ----- Contour Processing ----- 
	cv::findContours( imgSrcBinary, vContours, vHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0) );
	cv::drawContours( m_imgContours, vContours, -1, CV_RGB(255,255,255) );
	nNumContours = vContours.size();

	m_vContourStates.clear();

	for( int i=0; i<nNumContours; i++ )
	{
		dContourArea = cv::contourArea( vContours[i] ); 

		if( (dMinArea<dContourArea) && (dContourArea<dMaxArea) )
		{
			m_ContourMoments = moments( vContours[i], true );
			dCx = m_ContourMoments.m10/m_ContourMoments.m00;
			dCy = m_ContourMoments.m01/m_ContourMoments.m00;
		
			dAspectRatio = m_ContourMoments.mu02/m_ContourMoments.mu20;
			if( (0.05<dAspectRatio) && (dAspectRatio<20.0) && (m_ContourMoments.m00>1.0) )
			{
				rectContour = boundingRect(vContours[i]);

				//if( (rectContour.width<200) && (rectContour.height<100) )
				{
					rectangle( m_imgContours, rectContour, CV_RGB(255,0,0), 1 );

					ContourStates tempContour;
					tempContour.pos[0] = dCx;
					tempContour.pos[1] = dCy;
					tempContour.area = m_ContourMoments.m00;
					tempContour.mu[0] = m_ContourMoments.mu02;
					tempContour.mu[1] = m_ContourMoments.mu20;
					tempContour.aspectratio = dAspectRatio;

					m_vContourStates.push_back(tempContour);
				}
				//else
				{
	//				printf("\n" );
				}
			}
		}
	}

	imshow( "m_imgContours", m_imgContours );
	return 1;
}


int ClassContourProcess::FindTheBiggestMarker( std::vector <ContourStates> vContourStates )
{
	int iIdxBiggest = 0;

	if( vContourStates.size() == 0 )
		return -1;

	if( vContourStates.size() == 1 )
		return 0;

	double dMaxArea = vContourStates[0].area;
	for( int i=1; i<vContourStates.size(); i++ )
	{
		if( vContourStates[i].area > dMaxArea )
		{
			dMaxArea = vContourStates[i].area;
			iIdxBiggest = i;
		}
	}

	return iIdxBiggest;
}


int ClassContourProcess::EstimateDistanceToMarker( double dAreaActual, double dAreaMeasured, double *dPosImg, float *fCameraAtt, double *dDistResult )
{
	float sin_phi = sin(fCameraAtt[0]);
	float sin_theta = sin(fCameraAtt[1]);
	float sin_psi = sin(fCameraAtt[2]);
	float cos_phi = cos(fCameraAtt[0]);
	float cos_theta = cos(fCameraAtt[1]);
	float cos_psi = cos(fCameraAtt[2]);

	float fRotI2C[3][3];
	fRotI2C[0][0] = cos_theta*cos_psi;							fRotI2C[0][1] = cos_theta*sin_psi;							fRotI2C[0][2] = -sin_theta;
	fRotI2C[1][0] = sin_phi*sin_theta*cos_psi -cos_phi*sin_psi;	fRotI2C[1][1] = sin_phi*sin_theta*sin_psi +cos_phi*cos_psi;	fRotI2C[1][2] = sin_phi*cos_theta;
	fRotI2C[2][0] = cos_phi*sin_theta*cos_psi +sin_phi*sin_psi;	fRotI2C[2][1] = cos_phi*sin_theta*sin_psi -sin_phi*cos_psi;	
	fRotI2C[2][2] = cos_phi*cos_theta;


	double dAreaMeasCorrected = dAreaMeasured/fRotI2C[2][2];

	dDistResult[2] = 340.0 * sqrt(dAreaActual/dAreaMeasCorrected);

	dDistResult[0] = (240.0-dPosImg[1]) * dDistResult[2] / 340.0;
	dDistResult[1] = (dPosImg[0]-320.0) * dDistResult[2] / 340.0;

	return 1;
}
