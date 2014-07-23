//#include "stdafx.h"
#include "OpenGLView.h"
#include <iostream>
#include <fstream>
#include <iomanip>


// visual show options
#define PCD_COLOR_SHOW	1
//#define HS2_OCTREE_RELATED	1
//#define HS2_MCL3D_RELATED	1
//#define HS2_FSLAM_RELATED	1
//#define HS2_LASERCAMSLAM_RELATED	1
// ---------------

// communication result show options
//#define COMM_BEARNAV_RELATED	1
// ----------------


extern SharedStates g_States;

extern StructSensorsDownlink	g_TcpSensorData;

#ifdef COMM_BEARNAV_RELATED
	extern PCK_SENSORDATA_BEARNAV	g_TcpSensorData_Bearnav;
	extern PCK_COMMAND_BEARNAV		g_TcpCommand_Bearnav;
#endif

extern LaserParameters g_LaserParam1;

#ifdef HS2_OCTREE_RELATED
	#include "HS2_Octree.h"
	extern HS2_Octree g_Octree1;
#endif

#ifdef HS2_MCL3D_RELATED
	extern MCL3dStates g_Mcl3dStates;
#endif

#ifdef HS2_FSLAM_RELATED
	#if( FSLAM_VERSION == 1 )
		#include "F_SLAM.h"
	#endif
	#if( FSLAM_VERSION == 2 )
		#include "F_SLAM2.h"
	#endif
	#if( FSLAM_VERSION == 3 ) 
		#include "F_SLAM3.h"
	#endif
	#if( FSLAM_VERSION == 4 )
		#include "F_SLAM4.h"
	#endif
	float g_fEKFSLAM_MeanCurr[NUM_FULLSTATEVECTOR];
	float g_fEKFSLAM_CovCurr[NUM_FULLSTATEVECTOR][NUM_FULLSTATEVECTOR];
	int g_nNumEKFSLAMstates = 0;
	int g_nNumEKFSLAMLMs = 0;
	float g_fEKFSLAM_ExLMpos[100][3];
	int g_nNumEKFSLAMexLMs = 0;
#endif

#ifdef HS2_LASERCAMSLAM_RELATED
	#include "HS2_LaserCamSLAM.h"
	extern LaserCamSlamStates g_LaserCamSlam;
#endif

void MyMouseMove( int x, int y );
void MyMouseButton( int button, int state, int x, int y );
void MyDisplay();
void MyReshape(int x, int y);
void MyTimer(int value);
void MyMainMenu(int entryID);
void MySubMenu(int entryID);
void MySubMenu2(int entryID);
void MyKeyboard(int Key, int x, int y);


COpenglView glview1;

unsigned WINAPI Thread_OpenglView( void *arg )
{
	glview1.GLInitialize();

	int pargc = 0;
	glutInit(&pargc, NULL);
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
	glutInitWindowSize(glview1.m_cx,glview1.m_cy);
	glutInitWindowPosition(900,100);
	glutCreateWindow("3D View");
	glClearColor(0.0,0.0,0.0,1.0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.0,1.0,-1.0,1.0,-1.0,1.0);

#ifdef USE_NVIDIA_GPU_FOR_OPENGL	// GPU related
	glview1.GPU_SetBuffer();
#endif

/*
	// menu
	GLint MySubMenuID = glutCreateMenu(MySubMenu);
	glutAddMenuEntry("Day",1);
	glutAddMenuEntry("Night",2);
	glutAddMenuEntry("Wire Frame",3);
	GLint MyMainMenuID = glutCreateMenu(MyMainMenu);
	glutAddSubMenu("Shape",MySubMenuID);
	glutAddMenuEntry("Light On/Off",1);
	glutAddMenuEntry("Animation",2);
	glutAddMenuEntry("Exit",3);
	glutAttachMenu(GLUT_RIGHT_BUTTON);
*/
	// menu
	GLint MyMainMenuID = glutCreateMenu(MyMainMenu);
	//GLint MySubMenuID = glutCreateMenu(MySubMenu);
	glutAddMenuEntry("Birdeye View: Origin",1);
	glutAddMenuEntry("Birdeye View: Vehicle",2);
	glutAddMenuEntry("Chase View: ",3);
	glutAddMenuEntry("Pilot View: ",4);
	
	glutAddMenuEntry("Laser: Latest",5);
	glutAddMenuEntry("Laser: Point Cloud",6);

/*	glutAddSubMenu("ViewPoint",MySubMenuID);
	GLint MySubMenuID2 = glutCreateMenu(MySubMenu2);
	glutAddSubMenu("Laser Type",MySubMenuID2);
*/

	glutAddMenuEntry("Light On/Off",7);
	glutAddMenuEntry("Light On/Off",8);
	//glutAddMenuEntry("Animation",2);
	//glutAddMenuEntry("Exit",3);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

//	LoadGLTextures();
//	InitLight();
	glutDisplayFunc(MyDisplay);
	glutReshapeFunc(MyReshape);
	glutTimerFunc(10, MyTimer, 1);
	glutMouseFunc( MyMouseButton );
	glutMotionFunc( MyMouseMove );
	//glutSpecialFunc( MyKeyboard );
	glutMainLoop();

	return 1;
}





void MyDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);	
	glLoadIdentity();
/*	
	gluLookAt(0.0, 100.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 1.0);
 
	glRotatef(eye1,1.0,0.0,0.0);
	glRotatef(eye2,0.0,0.0,1.0);
	
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);	
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);	
	glLightfv(GL_LIGHT1, GL_POSITION,LightPosition);
	
	if(lof==0)
		glEnable(GL_LIGHT1);
	else
		glDisable(GL_LIGHT1);
	
	glEnable(GL_LIGHT0);	
	glPushMatrix();
	glRotatef(0, 0.0,1.0,0.0);
	glPushMatrix();
    glRotatef(eye3,0,0,1);

	glDisable(GL_TEXTURE_2D);
*/

//	glPopMatrix();
//	glPopMatrix();
	
	glview1.RenderScene();

	glFlush();
	glutSwapBuffers();
}

void MyReshape(int x, int y)
{
	glViewport(0, 0, (GLsizei) x, (GLsizei) y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20, (GLdouble)x/(GLdouble)y,1.0,50.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glutPostRedisplay();
}



void MyTimer(int value)
{
	if( glview1.m_iTimerOn )
	{
		glutPostRedisplay();
		glutTimerFunc(130, MyTimer, 1);
	}
}


void COpenglView::GLInitialize()
{
	g_States.OpenGL_On = 1;

	m_iTimerOn = 1;


	m_cx = 1200;	// opengl screen size
	m_cy = 900;	// opengl screen size

	m_dViewScale[0] = 5225.f;//-0.0002f;
	m_dViewScale[1] = 5225.f;//-0.0002f;
	m_dViewScale[2] = 15000.f;//-0.0002f;

	m_dViewRotate[0] = 45;
	m_dViewRotate[1] = 0;//180;
	m_dViewRotate[2] = 0;//-90;

	for( int ii=0; ii<3; ii++ )
	{
		m_dLookAtCenter[ii] = 0.0;
		m_dRobotPos[ii] = 0.0;
		m_dRobotAtt[ii] = 0.0;
	}

	m_Rotate = FALSE;
	m_LeftButtonDown = FALSE;
	m_RightButtonDown = FALSE;

	m_iViewpoint = 0;

	m_dTurnProp = 0.0;

//	BuildFont(hdc);
	glEnable(GL_DEPTH_TEST);	// 은면제거 활성화 : 깊이 비교를 통해서 깊이 버퍼를 자동으로 업데이트 해주기 때문에 뒤쪽의 선은 안보이게 된다

	//glEnable(GL_CULL_FACE);	//	후면제거모드 활성화
	//glFrontFace(GL_CCW);	//	default 후면제거
	//glCullFace(GL_BACK);	//	후면을 제거한다

	// ----- Vehicle Trace related -----
	m_nMaxVehicleTrace = 0;
	m_nCurrentVehicleTrace = 0;
	// ------------------------------------


}

void COpenglView::GLTerminate()
{
	
}

void COpenglView::RenderText()
{
/*	char strSensorState[300];

	glRasterPos2f(-0.490f,0.350f);
	glColor3ub(255,255,255);
	//sprintf_s( strSensorState, 299, "Bearnav Nav %.2f [Hz]", g_States.BearnavDataFreq );
	sprintf_s( strSensorState, 299, "Bearnav Nav %d [ms]", //  AccFromBearnav %.1f %.1f %.1f [m/s2]", 
		g_States.BearnavDataTime );//, g_States.AccFromBearnav[0]*0.02f, g_States.AccFromBearnav[1]*0.02f, g_States.AccFromBearnav[2]*0.02f );
	glutBitmapString(GLUT_BITMAP_HELVETICA_12, (unsigned char*)strSensorState);


	glRasterPos2f(-0.490f,-0.370f);
	glColor3ub(255,255,255);
	sprintf_s( strSensorState, 299, "IMU roll %.2f  pitch %.2f [deg] P %.2f  Q %.2f  Acc %.2f %.2f %.2f [m/s2]"
		, g_TcpSensorData.IMUdata[0]*R2D, g_TcpSensorData.IMUdata[1]*R2D, g_TcpSensorData.IMUdata[3], g_TcpSensorData.IMUdata[4], g_TcpSensorData.IMUdata[6], g_TcpSensorData.IMUdata[7], g_TcpSensorData.IMUdata[8] );
	glutBitmapString(GLUT_BITMAP_HELVETICA_12, (unsigned char*)strSensorState);

	glRasterPos2f(-0.490f,-0.355f);
	glColor3ub(255,255,255);
	sprintf_s( strSensorState, 299, "State roll %.2f  pitch %.2f  yaw %.2f [deg] P %.2f  Q %.2f"
		, g_fEKFSLAM_MeanCurr[6]*R2D, g_fEKFSLAM_MeanCurr[7]*R2D, g_fEKFSLAM_MeanCurr[8]*R2D, g_fEKFSLAM_MeanCurr[9], g_TcpSensorData.IMUdata[10] );
	glutBitmapString(GLUT_BITMAP_HELVETICA_12, (unsigned char*)strSensorState);

	glRasterPos2f(-0.490f,-0.340f);
	glColor3ub(255,255,255);
	sprintf_s( strSensorState, 299, "g_dProcRobotPos[m] %.2f %.2f %.2f  Att[deg] %.1f %.1f %.1f  PosCov %.2f %.2f %.2f "
		, g_dProcRobotPos[0], g_dProcRobotPos[1], g_dProcRobotPos[2], g_dProcRobotAtt[0]*R2D, g_dProcRobotAtt[1]*R2D, g_dProcRobotAtt[2]*R2D, g_dProcRobotPosCov[0], g_dProcRobotPosCov[1], g_dProcRobotPosCov[2] );
	glutBitmapString(GLUT_BITMAP_HELVETICA_12, (unsigned char*)strSensorState);

	glRasterPos2f(-0.490f,-0.325f);
	glColor3ub(255,255,255);
	sprintf_s( strSensorState, 299, "g_TcpCommand_Bearnav.t %.1f llhSlam %.6f %.6f %.2f  PosCov %.2f"
		, g_TcpCommand_Bearnav.TimeTag, g_TcpCommand_Bearnav.llhSlam[0]*R2D, g_TcpCommand_Bearnav.llhSlam[1]*R2D, g_TcpCommand_Bearnav.llhSlam[2]*R2D
		, g_TcpCommand_Bearnav.covSlam[0] );
	glutBitmapString(GLUT_BITMAP_HELVETICA_12, (unsigned char*)strSensorState);

	glRasterPos2f(-0.490f,-0.310f);
	glColor3ub(255,255,255);
	sprintf_s( strSensorState, 299, "g_TcpSensorData_Bearnav.GPSTime %.2f llh %.6f %.6f %.2f  LLC[m] %.2f %.2f %.2f"
		, g_TcpSensorData_Bearnav.GPSTime, g_TcpSensorData_Bearnav.llh[0]*R2D, g_TcpSensorData_Bearnav.llh[1]*R2D, g_TcpSensorData_Bearnav.llh[2]*R2D
		, g_TcpSensorData_Bearnav.lcc[0], g_TcpSensorData_Bearnav.lcc[1], g_TcpSensorData_Bearnav.lcc[2] );
	glutBitmapString(GLUT_BITMAP_HELVETICA_12, (unsigned char*)strSensorState);
*/
}

#ifndef STRUCT_OBSTACLE_INFO
typedef struct
{
	int iBearing;
	unsigned short usRange;
} STRUCT_OBSTACLE_INFO;
#endif

int COpenglView::DetectObstacles()
{
	vector< vector<STRUCT_OBSTACLE_INFO> > vLabeledObstacle;

	vector <STRUCT_OBSTACLE_INFO> vObstacle;

	STRUCT_OBSTACLE_INFO stObsTemp;

	unsigned short usRangeDiff;
	unsigned short usRangeThreshold = 80; // [mm]
	int nCountPointsInObst = 0;
	int nPointsInObstThreshold = 20;

	for(int i=g_LaserParam1.Left90DegStep; i<=g_LaserParam1.Right90DegStep; i++)
	{
		if( g_TcpSensorData.LRFdata[i] < 300 )	// [mm]
		{
			if( nCountPointsInObst > nPointsInObstThreshold )
				vLabeledObstacle.push_back( vObstacle );	// Register vObstacle to vLabeledObstacle

			// reset vObstacle
			vObstacle.clear();
			nCountPointsInObst = 0;
			continue;
		}
			
		if( g_TcpSensorData.LRFdata[i-1] > g_TcpSensorData.LRFdata[i] )
			usRangeDiff = g_TcpSensorData.LRFdata[i-1] -g_TcpSensorData.LRFdata[i];
		else
			usRangeDiff = g_TcpSensorData.LRFdata[i] -g_TcpSensorData.LRFdata[i-1];

		if( usRangeDiff < usRangeThreshold )	// near points
		{
			stObsTemp.iBearing = i;
			stObsTemp.usRange = g_TcpSensorData.LRFdata[i];

			vObstacle.push_back( stObsTemp );
			nCountPointsInObst += 1;

		}
		else	// not near points
		{
			if( nCountPointsInObst > nPointsInObstThreshold )
				vLabeledObstacle.push_back( vObstacle );	// Register vObstacle to vLabeledObstacle

			// reset vObstacle
			vObstacle.clear();
			nCountPointsInObst = 0;

		}
	}

	// draw in opengl

	float XYZ[3], XYZ_3D[3]; //[mm]
	unsigned char ucRGBdefault[3] = {0,0,0};
	glColor3ub(255, 0, 0); 
	glLineWidth(3.0f);
	
	XYZ[2] = -100.f;
	for( int m=0; m<vLabeledObstacle.size(); m++ )
	{
		glColor3ub(255, 255, 255);
		glBegin(GL_LINE_STRIP);	

		for( int n=0; n<vLabeledObstacle[m].size(); n++ )
		{
			XYZ[0] = (float)((vLabeledObstacle[m])[n].usRange) * g_LaserParam1.LUT_CosScanAngle[(vLabeledObstacle[m])[n].iBearing];
			XYZ[1] = (float)((vLabeledObstacle[m])[n].usRange) * g_LaserParam1.LUT_SinScanAngle[(vLabeledObstacle[m])[n].iBearing];
	
			for( int k=0; k<3; k++ )
				XYZ_3D[k] = (m_Rot_I2L[0][k]*XYZ[0] + m_Rot_I2L[1][k]*XYZ[1] + m_Rot_I2L[2][k]*XYZ[2]) + m_dRobotPos[k];

			glVertex3fv( XYZ_3D );
		}
		glEnd();
	}

	return 1;
}

void COpenglView::RenderScene()
{
	GLsizei width,height;
	//GLdouble aspect;

	width = m_cx*2;	//640 1280
	height = m_cy*2;//460 920

//	double dRobotPos[3], dRobotAtt[3];
	// 현재 처리되고 있는 robot state 출력
/*	
	if( g_States.TcpComm_On == 1 )
	{
		for(int i=0; i<12; i++)
		{
			g_fEKFSLAM_MeanCurr[i] = g_TcpSensorData.EKFSLAM_MeanCurr[i]; 
			//dRobotPos[i] = g_dProcRobotPos[i];
			//dRobotAtt[i] = g_dProcRobotAtt[i];
		}
	}
*/
	glEnable(GL_DEPTH_TEST);	// 은면제거 활성화 : 깊이 비교를 통해서 깊이 버퍼를 자동으로 업데이트 해주기 때문에 뒤쪽의 선은 안보이게 된다
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// 은면제거를 위해 깊이정보 초기화 Clear The Screen And The Depth Buffer

	glViewport(-(width-m_cx)/2,-(width-m_cy)/2,width, width);//width,height);	// Reset The Current Viewport  
	
	glMatrixMode(GL_PROJECTION);	// Select The Projection Matrix
 
	glLoadIdentity();	// Reset The Projection Matrix 행렬을 단위행렬로 만든다

	glPushMatrix();

	RenderText();

		gluPerspective(90, 1.0, 500.0, 150000.0);

		glMatrixMode(GL_MODELVIEW);	
		
		// ------- Inertial Coordinate System <<
		glPushMatrix();

			double dLookAtEye[3];
			double dLookAtCenter[3];
			double dLookAtUp[3] = {0.0, 0.0, -1.0};

#ifdef HS2_LASERCAMSLAM_RELATED
			if( g_States.LaserCamSLAM_On > 0 )
			{
				for( int i=0; i<3; i++ )
				{
					m_dRobotPos[i] = (double)(g_LaserCamSlam.fMeanRobot[i])*1000.0;
					m_dRobotAtt[i] = (double)(g_LaserCamSlam.fMeanRobot[i+6]);
				}
			}
#endif

#ifdef HS2_MCL3D_RELATED
			if( g_States.MCL3D_On > 0 )
			{
				for( int i=0; i<3; i++ )
				{
					m_dRobotPos[i] = g_Mcl3dStates.dMeanPos[i]*1000.0;
					m_dRobotAtt[i] = g_Mcl3dStates.dMeanAtt[i];
				}
			}
#endif

#ifdef HS2_FSLAM_RELATED
			if( g_States.FusionSLAM_On > 0 )
			{
				for( int i=0; i<3; i++ )
				{
					m_dRobotPos[i] = (double)g_fEKFSLAM_MeanCurr[i]*1000.0;
					m_dRobotAtt[i] = (double)g_fEKFSLAM_MeanCurr[i+6];
				}
			}
#endif

			switch(m_iViewpoint)
			{
				case 0:// ------- Birdeye View : Looking at origin -------
					dLookAtEye[0] = -glview1.m_dViewScale[2]*cos(-glview1.m_dViewRotate[2]*D2R)*cos(glview1.m_dViewRotate[0]*D2R);
					dLookAtEye[1] = glview1.m_dViewScale[2]*sin(-glview1.m_dViewRotate[2]*D2R)*cos(glview1.m_dViewRotate[0]*D2R);
					dLookAtEye[2] = -glview1.m_dViewScale[2]*sin(glview1.m_dViewRotate[0]*D2R);
					dLookAtCenter[0] = m_dLookAtCenter[0];
					dLookAtCenter[1] = m_dLookAtCenter[1];
					dLookAtCenter[2] = m_dLookAtCenter[2];					
					break;
				case 1:// ------- Birdeye View : Looking at robot -------
					dLookAtEye[0] = -glview1.m_dViewScale[2]*cos(-glview1.m_dViewRotate[2]*D2R)*cos(glview1.m_dViewRotate[0]*D2R) + m_dRobotPos[0];
					dLookAtEye[1] = glview1.m_dViewScale[2]*sin(-glview1.m_dViewRotate[2]*D2R)*cos(glview1.m_dViewRotate[0]*D2R) + m_dRobotPos[1];
					dLookAtEye[2] = -glview1.m_dViewScale[2]*sin(glview1.m_dViewRotate[0]*D2R) + m_dRobotPos[2];
					dLookAtCenter[0] =  m_dRobotPos[0];
					dLookAtCenter[1] =  m_dRobotPos[1];
					dLookAtCenter[2] =  m_dRobotPos[2];
					break;
				case 2:// ------- Chase View -------
					dLookAtEye[0] = m_dRobotPos[0] - glview1.m_dViewScale[2]*cos(m_dRobotAtt[2]);
					dLookAtEye[1] = m_dRobotPos[1] - glview1.m_dViewScale[2]*sin(m_dRobotAtt[2]);
					dLookAtEye[2] = m_dRobotPos[2] - glview1.m_dViewScale[2]/4.0;
					dLookAtCenter[0] = m_dRobotPos[0];
					dLookAtCenter[1] = m_dRobotPos[1];
					dLookAtCenter[2] = m_dRobotPos[2];
					break;
				case 3:// ------- Pilot View -------
					dLookAtEye[0] = m_dRobotPos[0];
					dLookAtEye[1] = m_dRobotPos[1];
					dLookAtEye[2] = m_dRobotPos[2] +150.0;
					dLookAtCenter[0] = m_dRobotPos[0] + 5000.0*cos(m_dRobotAtt[2]);
					dLookAtCenter[1] = m_dRobotPos[1] + 5000.0*sin(m_dRobotAtt[2]);
					dLookAtCenter[2] = m_dRobotPos[2]+200.0 - 5000.0*sin(m_dRobotAtt[1]);
					//dLookAtUp[0] = sin(1.0);//sin(g_dEKFSLAM_MeanCurr[7]);
					//dLookAtUp[1] = sin(1.0);//sin(g_dEKFSLAM_MeanCurr[6]);
					//dLookAtUp[2] = -cos(1.0);//-cos(g_dEKFSLAM_MeanCurr[6]);
					break;
			}

			gluLookAt( dLookAtEye[0], dLookAtEye[1], dLookAtEye[2], dLookAtCenter[0],  dLookAtCenter[1], dLookAtCenter[2],  dLookAtUp[0], dLookAtUp[1], dLookAtUp[2]);
			

			// ------- Grid, Axis -------
			Draw3dAxis();
			Draw3dGrid( 2000, 50000 );


			GetTransformationMatrix();

			DrawVehicleTrace();

			if( g_States.LRF_On > 0 )
			{
				if( g_States.LaserPointsViewMode > 0 )	
					DrawLRF_PointCloud();	
				else
				{
					DrawLRF_LatestData();
					DetectObstacles();
				}
			}

#ifdef HS2_OCTREE_RELATED
			// load octree once
			if( g_States.Octree_On == 23 )
			{
				ReadAllPointCloudsToMakeOctree(1);
				printf("Load Octree\n");
				g_States.Octree_On = 3;
			}


			// show octree
			if( g_States.Octree_On == 1 )
			{
				g_Octree1.InitializeOctree( 50, 9, 0 );	//64, 7, 0
				//g_Octree1.InitializeOctree( 200, 9, 30000 );	//64, 7, 0
				g_States.Octree_On = 2;
			}
			else if( g_States.Octree_On > 1 )
			{
				glPushMatrix();
					glTranslatef(g_Octree1.m_fOffsetX, 0.f, 0.f);
					DrawOctree();
					//show_OctreeRayCasting();
				glPopMatrix();
			}

			// save octree once
			if( g_States.Octree_On == 13 )
			{
				StoreAllOctreeNodeAsPoints();
				printf("StoreAllOctreeNodeAsPoints\n");
				g_States.Octree_On = 3;
			}
#endif

#ifdef HS2_MCL3D_RELATED
			if( g_States.MCL3D_On == 1 )
			{
				DrawMcl3dParticles();
			}
#endif

			// ------- Vehicle Body Coordinate System <<
			glPushMatrix();

#ifdef HS2_FSLAM_RELATED
				if( g_States.FusionSLAM_On > 0 )
				{
					DrawRobotCovariance();

#if( FSLAM_VERSION == 2 )
					DrawLandmarkCovariance();
#endif
#if( FSLAM_VERSION == 3 )
					DrawLandmarkCovariance_InverseDepth();
#endif
#if( FSLAM_VERSION == 4 )
					DrawLandmarkCovariance();
#endif

					DrawExLandmark();
					
					glTranslatef( g_fEKFSLAM_MeanCurr[0]*1000.0f, g_fEKFSLAM_MeanCurr[1]*1000.0f, g_fEKFSLAM_MeanCurr[2]*1000.0f);
					glRotatef(g_fEKFSLAM_MeanCurr[8]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
					glRotatef(g_fEKFSLAM_MeanCurr[7]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch
					glRotatef(g_fEKFSLAM_MeanCurr[6]*R2Df, 1.0f, 0.0f, 0.0f);	// roll

					DrawVehicle_Quadrotor();
					DrawCameraFOV();
				}
#endif
				if( g_States.TcpComm_On > 0 )
				{
					//drawRobotCovariance();
					//drawLandmarkCovariance();
					//drawExLandmark();
					glTranslatef( m_dRobotPos[0], m_dRobotPos[1], m_dRobotPos[2]);
					glRotatef(m_dRobotAtt[2]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
					glRotatef(m_dRobotAtt[1]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch
					glRotatef(m_dRobotAtt[0]*R2Df, 1.0f, 0.0f, 0.0f);	// roll
					
					DrawVehicle_Quadrotor();
				}

#ifdef HS2_LASERCAMSLAM_RELATED
				if( g_States.LaserCamSLAM_On > 0 )
				{
					DrawRobotCovariance_LaserCamSlam();
					DrawLandmarkCovariance_LaserCamSlam();

					glTranslatef( g_LaserCamSlam.fMeanRobot[0]*1000.0f, g_LaserCamSlam.fMeanRobot[1]*1000.0f, g_LaserCamSlam.fMeanRobot[2]*1000.0f);
					glRotatef(g_LaserCamSlam.fMeanRobot[8]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
					glRotatef(g_LaserCamSlam.fMeanRobot[7]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch
					glRotatef(g_LaserCamSlam.fMeanRobot[6]*R2Df, 1.0f, 0.0f, 0.0f);	// roll
					
					DrawVehicle_Quadrotor();
				}
#endif

#ifdef HS2_MCL3D_RELATED
				if( g_States.MCL3D_On > 0 )
				{
					glTranslated( g_Mcl3dStates.dMeanPos[0]*1000.0, g_Mcl3dStates.dMeanPos[1]*1000.0, g_Mcl3dStates.dMeanPos[2]*1000.0);
					glRotated(g_Mcl3dStates.dMeanAtt[2]*R2D, 0.0, 0.0, 1.0);	// yaw
					glRotated(g_Mcl3dStates.dMeanAtt[1]*R2D, 0.0, 1.0, 0.0);	// pitch
					glRotated(g_Mcl3dStates.dMeanAtt[0]*R2D, 1.0, 0.0, 0.0);	// roll

					DrawVehicle_Quadrotor();
				}
#endif

				//DrawVehicle_AutoCar();

				// ------- Laser mount Coordinate System <<
				/*
				glPushMatrix(); 
					if( g_States.LaserPointsViewMode == 0 )	
						DrawLRFdata();
				glPopMatrix();
				*/
				//  >> Laser mount Coordinate System -------

			glPopMatrix();
			// >> Vehicle Body Coordinate System -------

		glPopMatrix();
		// >> Inertial Coordinate System -------

	glPopMatrix();
	

	glMatrixMode(GL_PROJECTION);	// Select The Projection Matrix
	//glMatrixMode(GL_MODELVIEW);	

	return;
}

/*
AUX_RGBImageRec* LoadBMP( char* szFilename)
{
	FILE* pFile = NULL;
	if( !szFilename )
		return NULL;

	pFile = fopen( szFilename, "r" );
	if( pFile )
	{
		fclose( pFile );
		return auxDIBImageLoad( szFilename );
	}
	return NULL;
}
*/

int COpenglView::LoadGLTextures()
{
/*	int Status = FALSE;
	glClearColor(0.0, 0.0, 0.0, 0.0);
	memset(m_pTextureImage, 0, sizeof(void *)*1);	// 포인터를 Null로 초기화

	if( m_pTextureImage[0]  = LoadBMP("map1_2011_0805_16h11m39s.bmp") )	// bitmap을 load하고 오류확인map_100425.bmp
	{
		Status = TRUE;
		glGenTextures( 1, &m_TextureObject[0] );	// texture 생성
		glBindTexture( GL_TEXTURE_2D, m_TextureObject[0] );
		glTexImage2D( GL_TEXTURE_2D, 0, 3, m_pTextureImage[0]->sizeX, m_pTextureImage[0]->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTextureImage[0]->data);
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glEnable( GL_TEXTURE_2D );
	}

	if( m_pTextureImage[0] )
	{
		if( m_pTextureImage[0]->data )
		{
			free( m_pTextureImage[0]->data );
		}
		free( m_pTextureImage[0] );
	}
*/
	return 0;
}


GLvoid COpenglView::draw_Texture()
{
/*	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// 은면제거를 위해 깊이정보 초기화 Clear The Screen And The Depth Buffer
	glColor3d(255,0,255);  // 컬러 지정
	glBegin(GL_LINE_LOOP);
*/
	glColor3f(1.0f,1.0f,1.0f);	// X-axis color RED

	glBindTexture( GL_TEXTURE_2D, m_TextureObject[0] );
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-10000.0f, -7500.0f, 5.0f);   //glVertex3f(-7500.0f, 10000.0f, 5.0f);   
    glTexCoord2f(0.0f, 1.0f); glVertex3f(10000.0f, -7500.0f, 5.0f);   //glVertex3f(-7500.0f, -10000.0f, 5.0f);   
    glTexCoord2f(1.0f, 1.0f); glVertex3f(10000.0f, 7500.0f, 5.0f);   //glVertex3f(7500.0f, -10000.0f, 5.0f);   
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-10000.0f, 7500.0f, 5.0f);    //glVertex3f(7500.0f, 10000.0f, 5.0f);    
/*	glTexCoord2s(0, 0); glVertex3s(-10000, 10000, 10);   
    glTexCoord2s(0, 400); glVertex3s(-10000, -10000, 10);   
    glTexCoord2s(400, 400); glVertex3s(10000, -10000, 10);   
    glTexCoord2s(400, 0); glVertex3s(10000, 10000, 10);    
*/    
	glEnd();   
}



GLvoid COpenglView::Draw3dAxis()
{
	int nScale = 3000;
	float fScale = 10.f;

	// ------------------------ draw X-Y-Z axis --------------------------
	glLineWidth(3.f);
	glColor3ub(255,0,0);	// X-axis color RED
	glBegin(GL_LINES);
	//glNormal3d(3.0,0.0,0.0);	
	glVertex3i(0, 0, 0);	glVertex3i(nScale, 0, 0);
	glEnd();

	glColor3ub(0,255,0);	// Y-axis color GREEN
	glBegin(GL_LINES);
	//glNormal3d(0.0,3.0,0.0);	
	glVertex3i(0, 0, 0);	glVertex3i(0, nScale, 0);
	glEnd();

	glColor3ub(0,0,255);	// Z-axis color BLUE
	glBegin(GL_LINES);
	//glNormal3d(0.0,0.0,3.0);	
	glVertex3i(0, 0, 0);	glVertex3i(0, 0, nScale);
	glEnd();

}



GLvoid COpenglView::Draw3dGrid( int nItv, int nScale )
{
	//int nScale = 50000;	// [mm]
	//int nItv = 2000;

	//int nOriginX = 10000;
	//int nOriginY = 10000;

	// ------------------------ draw grid --------------------------
	glLineWidth(1.f);
	glColor3ub(75,75,75);
	glBegin(GL_LINES);

	for( int i=-nScale; i<nScale+1; i=i+nItv )
	{
		glVertex3i(-nScale, i, 0);	glVertex3i(nScale, i, 0);
		glVertex3i(i, -nScale, 0);	glVertex3i(i, nScale, 0);
	}

	glEnd();
}


GLvoid COpenglView::DrawVehicle_Quadrotor()
{
/*	glTranslated( 1000.0, 1000.0, 1000.0 );

	glRotated( 1.0*R2D, 0.0, 0.0, 1.0);	// yaw
	glRotated( 0.0*R2D, 0.0, 1.0, 0.0);	// pitch
	glRotated( 0.0*R2D, 1.0, 0.0, 0.0);	// roll
*/
/*
	glTranslated( g_TcpSensorData.EKFSLAM_MeanCurr[0]*1000.0, g_TcpSensorData.EKFSLAM_MeanCurr[1]*1000.0, g_TcpSensorData.EKFSLAM_MeanCurr[2]*1000.0 );

	glRotated( g_TcpSensorData.EKFSLAM_MeanCurr[8]*R2D, 0.0, 0.0, 1.0);	// yaw
	glRotated( g_TcpSensorData.EKFSLAM_MeanCurr[7]*R2D, 0.0, 1.0, 0.0);	// pitch
	glRotated( g_TcpSensorData.EKFSLAM_MeanCurr[6]*R2D, 1.0, 0.0, 0.0);	// roll
*/
	// quadrotor body
	glPushMatrix();
		//glTranslated(0.0, 0.0, 0.0);	// [mm]
		glScaled(5.0, 100.0, 2.0);	// [cm]
		glColor3ub(100, 160, 210); 
		glutSolidCube(10.0);
		glColor3ub(250, 250, 250); 
		glutWireCube(10.0);
	glPopMatrix();

	glPushMatrix();
		glTranslated(237.5, 0.0, 0.0);	// [mm]
		glScaled(52.5, 5.0, 2.0);	// [cm]
		glColor3ub(100, 160, 210); 
		glutSolidCube(10.0);
		glColor3ub(250, 250, 250); 
		glutWireCube(10.0);
	glPopMatrix();

	// yellow back 
	glPushMatrix();
		glTranslated(-262.5, 0.0, 0.0);	// [mm]
		glScaled(47.5, 5.0, 2.0);	// [cm]
		glColor3ub(255, 255, 10); 
		glutSolidCube(10.0);
		glColor3ub(250, 250, 250); 
		glutWireCube(10.0);
	glPopMatrix();


	// quadrotor props
	// front
	glPushMatrix();
		glTranslated(470.0, 0.0, -20.0);	// [mm]
		glRotated(m_dTurnProp, 0.0, 0.0, 1.0);	// yaw
		glScaled(3.0, 35.0, 1.0);	// [cm]
		glColor3ub(220, 220, 220); 
		glutSolidCube(10.0);
	glPopMatrix();
	glPushMatrix();// back
		glTranslated(-470.0, 0.0, -20.0);	// [mm]
		glRotated(m_dTurnProp+170.0, 0.0, 0.0, 1.0);	// yaw
		glScaled(3.0, 35.0, 1.0);	// [cm]
		//glColor3ub(255, 255, 100); 
		glutSolidCube(10.0);
	glPopMatrix();
	glPushMatrix();// left
		glTranslated(0.0, -470.0, -20.0);	// [mm]
		glRotated(m_dTurnProp+70.0, 0.0, 0.0, -1.0);	// yaw
		glScaled(3.0, 35.0, 1.0);	// [cm]
		//glColor3ub(220, 220, 220); 
		glutSolidCube(10.0);
	glPopMatrix();
	glPushMatrix();// right
		glTranslated(0.0, 470.0, -20.0);	// [mm]
		glRotated(m_dTurnProp-110.0, 0.0, 0.0, -1.0);	// yaw
		glScaled(3.0, 35.0, 1.0);	// [cm]
		//glColor3ub(200, 200, 200); 
		glutSolidCube(10.0);
	glPopMatrix();

	m_dTurnProp += 35.0;

}



GLvoid COpenglView::DrawCameraFOV()
{
	float fBound[3];
	fBound[0] = 5000.0f;
	fBound[1] = fBound[0]*320.0f/340.0f;
	fBound[2] = fBound[0]*240.0f/340.0f;


	glPushMatrix();
		glRotated(-12.0, 0.0, 1.0, 0.0);	// yaw

		glLineWidth(2);
		glColor3ub(0,255,0);
		glBegin(GL_LINES);
		glVertex3f(0.0f, 0.0f, 0.0f);	glVertex3f(fBound[0], fBound[1], fBound[2]);
		glVertex3f(0.0f, 0.0f, 0.0f);	glVertex3f(fBound[0], fBound[1], -fBound[2]);
		glVertex3f(0.0f, 0.0f, 0.0f);	glVertex3f(fBound[0], -fBound[1], -fBound[2]);
		glVertex3f(0.0f, 0.0f, 0.0f);	glVertex3f(fBound[0], -fBound[1], fBound[2]);
		glVertex3f(fBound[0], fBound[1], fBound[2]);	glVertex3f(fBound[0], fBound[1], -fBound[2]);
		glVertex3f(fBound[0], fBound[1], -fBound[2]);	glVertex3f(fBound[0], -fBound[1], -fBound[2]);
		glVertex3f(fBound[0], -fBound[1], -fBound[2]);	glVertex3f(fBound[0], -fBound[1], fBound[2]);
		glVertex3f(fBound[0], -fBound[1], fBound[2]);	glVertex3f(fBound[0], fBound[1], fBound[2]);
		glEnd();

	glPopMatrix();

}

GLvoid COpenglView::DrawVehicle()
{
	// ---- just a box -----
	// body
	glPushMatrix();
		//glTranslated(0.0, 0.0, 0.0);	// [mm]
		glScaled(30.0, 30.0, 40.0);	// [cm]
		glColor3ub(100, 160, 210); 
		glutSolidCube(10.0);
		glColor3ub(250, 250, 250); 
		glutWireCube(10.0);
	glPopMatrix();
}


GLvoid COpenglView::DrawVehicle_AutoCar()
{

	glColor3ub(255, 255, 255);
	glPushMatrix();
		glTranslatef( -600.f, 0.f, 1300.f );
		glScalef(4.0f, 1.7f, 0.8f);
//		glColor3ub(255, 255, 255);
//		glutSolidCube(1000.f);
		glutWireCube(1000.f);
	glPopMatrix();

	glPushMatrix();
		glTranslatef( -1200.f, 0.f, 500.f );
		glScalef(2.8f, 1.7f, 0.8f);
		glutWireCube(1000.f);
	glPopMatrix();

	glColor3ub(255, 255, 0);
	glPushMatrix();
		glTranslatef(0.0f, 0.0f, 0.0f);
		glutWireCube(200.f);
	glPopMatrix();
}

GLvoid COpenglView::GetTransformationMatrix()
{
	float roll=0.f, pitch=0.f, yaw=0.f;
	float sinRoll, sinPitch, sinYaw;
	float cosRoll, cosPitch, cosYaw;

	// vehicle attitude
	if( (g_States.FusionSLAM_On > 0) || (g_States.TcpComm_On > 0) )
	{
		roll = m_dRobotAtt[0];	//0.f;
		pitch = m_dRobotAtt[1];	//0.f;
		yaw = m_dRobotAtt[2];	//0.f;
	}
	else if( g_States.IMU_On )
	{
		roll = (float)g_TcpSensorData.IMUdata[0];
		pitch = (float)g_TcpSensorData.IMUdata[1];
		yaw = (float)g_TcpSensorData.IMUdata[2];
	}
#ifdef COMM_BEARNAV_RELATED
	else if( g_States.TcpCommBearnav_On > 0 )
	{
		roll = (float)g_TcpSensorData_Bearnav.attitude[0];
		pitch = (float)g_TcpSensorData_Bearnav.attitude[1];
		yaw = (float)g_TcpSensorData_Bearnav.attitude[2];
	}
#endif

	sinRoll = sin(roll);	sinPitch = sin(pitch);	sinYaw = sin(yaw);
	cosRoll = cos(roll);	cosPitch = cos(pitch);	cosYaw = cos(yaw);

	float Rot_I2Imu[3][3];
	Rot_I2Imu[0][0] = cosYaw*cosPitch;	
	Rot_I2Imu[0][1] = sinYaw*cosPitch;	
	Rot_I2Imu[0][2] = -sinPitch;
	Rot_I2Imu[1][0] = -sinYaw*cosRoll+cosYaw*sinPitch*sinRoll;	
	Rot_I2Imu[1][1] = cosYaw*cosRoll+sinYaw*sinPitch*sinRoll;	
	Rot_I2Imu[1][2] = cosPitch*sinRoll;	
	Rot_I2Imu[2][0] = sinYaw*sinRoll+cosYaw*sinPitch*cosRoll;	
	Rot_I2Imu[2][1] = -cosYaw*sinRoll+sinYaw*sinPitch*cosRoll;	
	Rot_I2Imu[2][2] = cosPitch*cosRoll;

	// servo attitude
	roll = 0.f;
	pitch = (float)(g_TcpSensorData.ServoState[1])*D2Rf;
	yaw = 0.f;

	sinRoll = sin(roll);	sinPitch = sin(pitch);	sinYaw = sin(yaw);
	cosRoll = cos(roll);	cosPitch = cos(pitch);	cosYaw = cos(yaw);

	float Rot_Imu2L[3][3];
	Rot_Imu2L[0][0] = cosYaw*cosPitch;	
	Rot_Imu2L[0][1] = sinYaw*cosPitch;	
	Rot_Imu2L[0][2] = -sinPitch;
	Rot_Imu2L[1][0] = -sinYaw*cosRoll+cosYaw*sinPitch*sinRoll;	
	Rot_Imu2L[1][1] = cosYaw*cosRoll+sinYaw*sinPitch*sinRoll;	
	Rot_Imu2L[1][2] = cosPitch*sinRoll;	
	Rot_Imu2L[2][0] = sinYaw*sinRoll+cosYaw*sinPitch*cosRoll;	
	Rot_Imu2L[2][1] = -cosYaw*sinRoll+sinYaw*sinPitch*cosRoll;	
	Rot_Imu2L[2][2] = cosPitch*cosRoll;


	for (int ii=0; ii<3; ii++)
	{
		for (int jj=0; jj<3; jj++)
		{
			m_Rot_I2L[ii][jj]=0.f;
			for(int kk=0; kk<3; kk++)
				m_Rot_I2L[ii][jj] += Rot_Imu2L[ii][kk]*Rot_I2Imu[kk][jj];	// AB = A*B
		}
	}		

}


#ifdef LASER_UPDOWN_MIRROR_ON				
GLvoid COpenglView::DrawLRF_LatestData()
{
	float XYZ[3], XYZ_3D[3]; //[mm]
	unsigned char ucRGBdefault[3] = {0,0,0};
	glColor3ub(255, 0, 0); 
	glPointSize(3.0f);

	glBegin(GL_POINTS);

	// Left-side
	glColor3ub(255, 255, 0);
	float fDownDir = 0.f;
	for(int i=29; i<=48; i++)	// URG-04LX left side-downward dir. LRFdata[29~48]
	{
		fDownDir += 10.f;
		XYZ[0] = -10.f -fDownDir;
		XYZ[1] = -10.f;
		XYZ[2] = (float)(g_TcpSensorData.LRFdata[i]);

		for( int k=0; k<3; k++ )
			XYZ_3D[k] = (m_Rot_I2L[0][k]*XYZ[0] + m_Rot_I2L[1][k]*XYZ[1] + m_Rot_I2L[2][k]*XYZ[2]) + m_dRobotPos[k];

		glVertex3f(XYZ_3D[0], XYZ_3D[1], XYZ_3D[2] );
	}

	// Right-side
	glColor3ub(255, 0, 255);
	float fUpDir = 0.f;
	for(int i=615; i<=662; i++)	// URG-04LX right side-upward dir. LRFdata[615~662]
	{
		fUpDir += 10.f;
		XYZ[0] = -10.f -fUpDir;
		XYZ[1] = 10.f;
		XYZ[2] = -(float)(g_TcpSensorData.LRFdata[i]);

		for( int k=0; k<3; k++ )
			XYZ_3D[k] = (m_Rot_I2L[0][k]*XYZ[0] + m_Rot_I2L[1][k]*XYZ[1] + m_Rot_I2L[2][k]*XYZ[2]) + m_dRobotPos[k];

		glVertex3f(XYZ_3D[0], XYZ_3D[1], XYZ_3D[2] );
	}

	// Front
	glColor3ub(255, 0, 0);
	XYZ[2] = 0.f;
	for(int i=g_LaserParam1.Left90DegStep; i<=g_LaserParam1.Right90DegStep; i++)
	{
		XYZ[0] = (float)(g_TcpSensorData.LRFdata[i]) * g_LaserParam1.LUT_CosScanAngle[i];
		XYZ[1] = (float)(g_TcpSensorData.LRFdata[i]) * g_LaserParam1.LUT_SinScanAngle[i];
	
		for( int k=0; k<3; k++ )
			XYZ_3D[k] = (m_Rot_I2L[0][k]*XYZ[0] + m_Rot_I2L[1][k]*XYZ[1] + m_Rot_I2L[2][k]*XYZ[2]) + m_dRobotPos[k];

		glVertex3f(XYZ_3D[0], XYZ_3D[1], XYZ_3D[2] );
	}
	glEnd();
}

#else

GLvoid COpenglView::DrawLRF_LatestData()
{
	float XYZ[3], XYZ_3D[3];
	unsigned char ucRGBdefault[3] = {0,0,0};
	glColor3ub(255, 0, 0);  // 컬러 지정 
	glPointSize(3.0f);

	glBegin(GL_POINTS);
	XYZ[2] = 0.f;
	for(int i=g_LaserParam1.LeftEndStep; i<=g_LaserParam1.RightEndStep; i++)
	{
		XYZ[0] = (float)(g_TcpSensorData.LRFdata[i]) * g_LaserParam1.LUT_CosScanAngle[i];
		XYZ[1] = (float)(g_TcpSensorData.LRFdata[i]) * g_LaserParam1.LUT_SinScanAngle[i];
	
		for( int k=0; k<3; k++ )
			XYZ_3D[k] = (m_Rot_I2L[0][k]*XYZ[0] + m_Rot_I2L[1][k]*XYZ[1] + m_Rot_I2L[2][k]*XYZ[2]) + m_dRobotPos[k];

		glVertex3f(XYZ_3D[0], XYZ_3D[1], XYZ_3D[2] );

		// ----- generating octree node -----
		if( g_States.Octree_On == 2 )
			if( (g_TcpSensorData.LRFdata[i]>300) && (g_LaserParam1.LeftEndStep<=i) && (i<=g_LaserParam1.RightEndStep) )
				g_Octree1.InputData2(XYZ_3D, ucRGBdefault);
		// ----------------------------------
	}
	glEnd();
}

#endif

GLvoid COpenglView::DrawVehicleTrace()
{
	// storing Point Cloud Data 
	for( int k=0; k<3; k++ )
	{
		m_VehicleTrace[m_nCurrentVehicleTrace][k] = (m_dRobotPos[k]);
	}
		
	// draw point cloud
	glColor3ub(195, 195, 195);  // 컬러 지정 
	glPointSize(5.0f);
	glBegin(GL_POINTS);
	for(int j=0; j<m_nMaxVehicleTrace; j++)
	{
		glVertex3f(m_VehicleTrace[j][0], m_VehicleTrace[j][1], m_VehicleTrace[j][2] );
	}
	glEnd();

	if( m_nMaxVehicleTrace < 100 )
		m_nMaxVehicleTrace++;
	
	if( ++m_nCurrentVehicleTrace == 100 )
		m_nCurrentVehicleTrace = 0;

}




GLvoid COpenglView::DrawLRF_PointCloud()
{
	// initialize
	if( g_States.LaserPointsViewMode == 1 )
	{
		m_LRF_PCD = new short**[400];
		for(int i=0; i<400; i++)
		{
			m_LRF_PCD[i] = new short*[R_MEAS_STEP+1-L_MEAS_STEP];
			for(int j=0; j<(R_MEAS_STEP+1-L_MEAS_STEP); j++)
			{
				m_LRF_PCD[i][j] = new short[3];
			}
		}

		m_LRF_PCDcolor = new unsigned char**[400];
		for(int i=0; i<400; i++)
		{
			m_LRF_PCDcolor[i] = new unsigned char*[(R_MEAS_STEP+1-L_MEAS_STEP)];
			for(int j=0; j<(R_MEAS_STEP+1-L_MEAS_STEP); j++)
			{
				m_LRF_PCDcolor[i][j] = new unsigned char[3];
			}
		}

		m_nMaxPCD = 0;
		m_nCurrentPCD = 0;
		g_States.LaserPointsViewMode = 2;
		return;
	}

	// terminate
	if( g_States.LaserPointsViewMode == 3 )
	{
		for(int i=0; i<400; i++)
		{
			for(int j=0; j<(R_MEAS_STEP+1-L_MEAS_STEP); j++)
			{
				delete[] m_LRF_PCD[i][j];
			}
			delete[] m_LRF_PCD[i];
		}
		delete[] m_LRF_PCD;

		for(int i=0; i<400; i++)
		{
			for(int j=0; j<(R_MEAS_STEP+1-L_MEAS_STEP); j++)
			{
				delete[] m_LRF_PCDcolor[i][j];
			}
			delete[] m_LRF_PCDcolor[i];
		}
		delete[] m_LRF_PCDcolor;
		g_States.LaserPointsViewMode = 0;
		return;
	}

	// Draw laser point clouds
	float XYZ[3], XYZ_3D[3];

	unsigned char rgb_default[3] = {0,0,0};

	int iStepPCD = 0;

	//glBegin(GL_POINTS);
	for(int i=g_LaserParam1.Left90DegStep; i<=g_LaserParam1.Right90DegStep; i++)
	{
		XYZ[0] = (float)(g_TcpSensorData.LRFdata[i]) * g_LaserParam1.LUT_CosScanAngle[i];
		XYZ[1] = (float)(g_TcpSensorData.LRFdata[i]) * g_LaserParam1.LUT_SinScanAngle[i];
		XYZ[2] = 0.f;

		for( int k=0; k<3; k++ )
			XYZ_3D[k] = (m_Rot_I2L[0][k]*XYZ[0] + m_Rot_I2L[1][k]*XYZ[1] + m_Rot_I2L[2][k]*XYZ[2]) + m_dRobotPos[k];
			//XYZ_3D[k] = (m_Rot_I2L[0][k]*XYZ[0] + m_Rot_I2L[1][k]*XYZ[1] + m_Rot_I2L[2][k]*XYZ[2]) + (float)(1000.0*g_dEKFSLAM_MeanCurr[k]);

		// storing Point Cloud Data 
		if( g_States.LaserPointsViewMode == 2 )
		{
			if( (L_MEAS_STEP<=i) && (i<=R_MEAS_STEP) )
			{
				for( int k=0; k<3; k++ )
				{
					m_LRF_PCD[m_nCurrentPCD][iStepPCD][k] = (short)XYZ_3D[k];
#if( PCD_COLOR_SHOW )
					{
						m_LRF_PCDcolor[m_nCurrentPCD][iStepPCD][2] = 255;
						m_LRF_PCDcolor[m_nCurrentPCD][iStepPCD][1] = m_LRF_PCD[m_nCurrentPCD][iStepPCD][2]/10;
						m_LRF_PCDcolor[m_nCurrentPCD][iStepPCD][0] = 128-m_LRF_PCD[m_nCurrentPCD][iStepPCD][2]/10;
					}
#endif
				}
		
				iStepPCD++;
			}
		}
	}
	//glEnd();

	// draw point cloud
	glColor3ub(155, 0, 0);  // 컬러 지정 
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	for(int j=0; j<m_nMaxPCD; j++)
	{
		for(int i=0; i<(R_MEAS_STEP+1-L_MEAS_STEP); i++)
		{
#if( PCD_COLOR_SHOW )
			glColor3ub(m_LRF_PCDcolor[j][i][2], m_LRF_PCDcolor[j][i][1], m_LRF_PCDcolor[j][i][0] );
#endif
			glVertex3s(m_LRF_PCD[j][i][0], m_LRF_PCD[j][i][1], m_LRF_PCD[j][i][2] );
		}
	}
	glEnd();

	if( m_nMaxPCD < 400 )
		m_nMaxPCD++;
	
	if( ++m_nCurrentPCD == 400 )
		m_nCurrentPCD = 0;

}






#ifdef HS2_LASERCAMSLAM_RELATED
GLvoid COpenglView::DrawRobotCovariance_LaserCamSlam()
{
	float fRot[3], fEllipsePos[3], fEllipseAxis[3];
	float fItvRad = 15.0f*D2Rf;
	float fRobotScale = 1000.0f;


	glLineWidth(1.0f); 
	glColor3ub(255, 255, 0);  // 컬러 지정 

/*	Mat matA( 3, 3, CV_32FC1 );
	cv::SVD svd1;

	for( int ii=0; ii<3; ii++ )
		for( int jj=0; jj<3; jj++ )
			matA.at<float>(ii,jj) = g_fEKFSLAM_CovCurr[ii][jj];

	svd1.operator()( matA, SVD::MODIFY_A|SVD::NO_UV );

	// slightly bigger, but exact
	fEllipseAxis[0] = fRobotScale*sqrt( svd1.w.at<float>(0,0) );
	fEllipseAxis[1] = fRobotScale*sqrt( svd1.w.at<float>(1,0) );
	fEllipseAxis[2] = fRobotScale*sqrt( svd1.w.at<float>(2,0) );
*/
	// simple version
	fEllipseAxis[0] = fRobotScale*g_LaserCamSlam.fCovRobot[0][0];
	fEllipseAxis[1] = fRobotScale*g_LaserCamSlam.fCovRobot[1][1];
	fEllipseAxis[2] = fRobotScale*g_LaserCamSlam.fCovRobot[2][2];

	fRot[2] = 0.5f*atan2( 2.0f*g_LaserCamSlam.fCovRobot[0][1], (g_LaserCamSlam.fCovRobot[0][0]-g_LaserCamSlam.fCovRobot[1][1]) );	// yaw : between X- and Y-axis
	fRot[1] = -0.5f*atan2( 2.0f*g_LaserCamSlam.fCovRobot[0][2], (g_LaserCamSlam.fCovRobot[0][0]-g_LaserCamSlam.fCovRobot[2][2]) );	// pitch : between X- and Z-axis
	fRot[0] = 0.5f*atan2( 2.0f*g_LaserCamSlam.fCovRobot[1][2], (g_LaserCamSlam.fCovRobot[1][1]-g_LaserCamSlam.fCovRobot[2][2]) );	// roll : between Y- and Z-axis

	glPushMatrix();

		glTranslatef(1000.0f*g_LaserCamSlam.fMeanRobot[0], 1000.0f*g_LaserCamSlam.fMeanRobot[1], 1000.0f*g_LaserCamSlam.fMeanRobot[2]);

		glRotatef(fRot[2]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
		glRotatef(fRot[1]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch 
		glRotatef(fRot[0]*R2Df, 1.0f, 0.0f, 0.0f);	// roll

		fEllipsePos[0] = fEllipseAxis[0];
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = 0.0f;

		//x0=a+posX; y0=posY; z0=posZ;


		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*cos(i);
			fEllipsePos[1] = fEllipseAxis[1]*sin(i);
			//dEllipsePos[2] = 0.0;
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=posX; y0=b+posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = fEllipseAxis[1];
		fEllipsePos[2] = 0.0f;

		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			//dEllipsePos[0] = 0.0;
			fEllipsePos[1] = fEllipseAxis[1]*cos(i);
			fEllipsePos[2] = fEllipseAxis[2]*sin(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=a+posX; y0=posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = fEllipseAxis[2];
		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*sin(i);
			//dEllipsePos[1] = 0.0;
			fEllipsePos[2] = fEllipseAxis[2]*cos(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

	glPopMatrix();

}



GLvoid COpenglView::DrawLandmarkCovariance_LaserCamSlam()
{
// http://www.iph.ufrgs.br/corpodocente/marques/cd/multivar/eigen.htm
// Cov_xx, yy, zz는 ellipse의 major axis방향벡터
// Cov_xy, yz, xz는 각방향간의 관련성이므로 Cov가 0이면 서로 연관이 없으므로 구 형태가 된다.
// Cov_xy는 xy평면에 projection해볼때, 퍼져있는 정도를 나타내므로, xx,yy,zz벡터에 의해 회전이 들어간다면, 간단하게 xy를 간단하게 고려해 그릴수는 없고
// 회전에 맞게 euler transform을 반대로 넣어서 xy,yz,xz(ellipse굵기)를 표현해야 한다.

	// ellipsoid : x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
	// x = a*cos(lat)*cos(lon)
	// y = b*cos(lat)*sin(lon)
	// z = c*sin(lat)

	float fRot[3], fEllipsePos[3], fEllipseAxis[3];

	int iLM3;
	float fItvRad = 15.0f*D2Rf;
	float fLMScale = 500.0f;

	Mat matA( 3, 3, CV_32FC1 );
	cv::SVD svd1;

	glLineWidth(1.0f); 
	glColor3ub(255, 255, 0);  // 컬러 지정 

	//int nThisLMstate[3] = {0, 0, 0};
	for( int iLM=0; iLM<g_LaserCamSlam.nNumAllLMs; iLM++ )
	{
		iLM3 = iLM*3;

		for( int ii=0; ii<3; ii++ )
			for( int jj=0; jj<3; jj++ )
				AT_F(matA,ii,jj) = g_LaserCamSlam.fCovLM[iLM3+ii][jj];

		svd1.operator()( matA, SVD::MODIFY_A|SVD::NO_UV );

		fEllipseAxis[0] = fLMScale*( AT_F(svd1.w,0,0) );	//svd1.w.at<float>(0,0) );
		fEllipseAxis[1] = fLMScale*( AT_F(svd1.w,1,0) );	//svd1.w.at<float>(1,0) );
		fEllipseAxis[2] = fLMScale*( AT_F(svd1.w,2,0) );	//svd1.w.at<float>(2,0) );

		fRot[2] = 0.5f*atan2( 2.0f*AT_F(matA,0,1), (AT_F(matA,0,0)-AT_F(matA,1,1)) );	// yaw : between X- and Y-axis
		fRot[1] = -0.5f*atan( 2.0f*(AT_F(matA,0,2)+AT_F(matA,1,2))/(AT_F(matA,0,0)+AT_F(matA,1,1)-AT_F(matA,2,2)) );	// pitch : between X- and Z-axis
		//fRot[2] = 0.5f*atan2( 2.0f*g_LaserCamSlam.fCovLM[iLM3][1], (g_LaserCamSlam.fCovLM[iLM3][0]-g_LaserCamSlam.fCovLM[iLM3+1][1]) );	// yaw : between X- and Y-axis
		//fRot[1] = -0.5f*atan( 2.0f*(g_LaserCamSlam.fCovLM[iLM3][2]+g_LaserCamSlam.fCovLM[iLM3+1][2])/(g_LaserCamSlam.fCovLM[iLM3][0]+g_LaserCamSlam.fCovLM[iLM3+1][1]-g_LaserCamSlam.fCovLM[iLM3+2][2]) );	// pitch : between X- and Z-axis

		if( g_LaserCamSlam.fMeanLM[iLM3+2] > g_LaserCamSlam.fMeanRobot[2] )
			fRot[1] = -abs(fRot[1]);
		else
			fRot[1] = abs(fRot[1]);

		glPushMatrix();

		glTranslatef(1000.0f*g_LaserCamSlam.fMeanLM[iLM3], 1000.0f*g_LaserCamSlam.fMeanLM[iLM3+1], 1000.0f*g_LaserCamSlam.fMeanLM[iLM3+2]);
			
		glRotatef(fRot[2]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
		glRotatef(fRot[1]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch 
		//glRotated(dRot[0]*R2D, 1.0, 0.0, 0.0);	// roll

	
		fEllipsePos[0] = fEllipseAxis[0];
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = 0.0f;

		//x0=a+posX; y0=posY; z0=posZ;


		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*cos(i);
			fEllipsePos[1] = fEllipseAxis[1]*sin(i);
			//dEllipsePos[2] = 0.0;
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=posX; y0=b+posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = fEllipseAxis[1];
		fEllipsePos[2] = 0.0f;

		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			//dEllipsePos[0] = 0.0;
			fEllipsePos[1] = fEllipseAxis[1]*cos(i);
			fEllipsePos[2] = fEllipseAxis[2]*sin(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=a+posX; y0=posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = fEllipseAxis[2];
		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*sin(i);
			//dEllipsePos[1] = 0.0;
			fEllipsePos[2] = fEllipseAxis[2]*cos(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		glPopMatrix();
	}
}
#endif


#ifdef HS2_FSLAM_RELATED

GLvoid COpenglView::DrawExLandmark()
{
	glLineWidth(2.0f);
	glColor3ub(100, 255, 100); 
	for( int i=0; i<g_nNumEKFSLAMexLMs; i++ )
	{
		//g_fEKFSLAM_ExLMpos[i][3];

		glPushMatrix();
			glTranslatef(1000.0f*g_fEKFSLAM_ExLMpos[i][0], 1000.0f*g_fEKFSLAM_ExLMpos[i][1], 1000.0f*g_fEKFSLAM_ExLMpos[i][2]);// [mm]
			//glScaled(5.0, 100.0, 2.0);
			glutWireSphere( 100.0, 8, 4 );// [mm]
		glPopMatrix();
	}

}

GLvoid COpenglView::DrawLandmarkCovariance()
{
// http://www.iph.ufrgs.br/corpodocente/marques/cd/multivar/eigen.htm
// Cov_xx, yy, zz는 ellipse의 major axis방향벡터
// Cov_xy, yz, xz는 각방향간의 관련성이므로 Cov가 0이면 서로 연관이 없으므로 구 형태가 된다.
// Cov_xy는 xy평면에 projection해볼때, 퍼져있는 정도를 나타내므로, xx,yy,zz벡터에 의해 회전이 들어간다면, 간단하게 xy를 간단하게 고려해 그릴수는 없고
// 회전에 맞게 euler transform을 반대로 넣어서 xy,yz,xz(ellipse굵기)를 표현해야 한다.

	// ellipsoid : x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
	// x = a*cos(lat)*cos(lon)
	// y = b*cos(lat)*sin(lon)
	// z = c*sin(lat)

	float fRot[3], fEllipsePos[3], fEllipseAxis[3];

	int iLM_3;
	float fItvRad = 15.0f*D2Rf;
	float fLMScale = 500.0f;

	Mat matA( 3, 3, CV_32FC1 );
	cv::SVD svd1;

	glLineWidth(1.0f); 
	glColor3ub(255, 255, 0);  // 컬러 지정 

	int nThisLMstate[3] = {0, 0, 0};
	for( int iLM=0; iLM<g_nNumEKFSLAMLMs; iLM++ )
	{
		iLM_3 = iLM*3;
		nThisLMstate[0] = NUM_ROBOTSTATES +iLM_3;
		nThisLMstate[1] = NUM_ROBOTSTATES +1 +iLM_3;
		nThisLMstate[2] = NUM_ROBOTSTATES +2 +iLM_3;

		for( int ii=0; ii<3; ii++ )
			for( int jj=0; jj<3; jj++ )
				matA.at<float>(ii,jj) = g_fEKFSLAM_CovCurr[nThisLMstate[0]+ii][nThisLMstate[0]+jj];

		svd1.operator()( matA, SVD::MODIFY_A|SVD::NO_UV );


		fEllipseAxis[0] = fLMScale*( svd1.w.at<float>(0,0) );
		fEllipseAxis[1] = fLMScale*( svd1.w.at<float>(1,0) );
		fEllipseAxis[2] = fLMScale*( svd1.w.at<float>(2,0) );

		fRot[2] = 0.5f*atan2( 2.0f*g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[1]], (g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[0]]-g_fEKFSLAM_CovCurr[nThisLMstate[1]][nThisLMstate[1]]) );	// yaw : between X- and Y-axis
		fRot[1] = -0.5f*atan( 2.0f*(g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[2]]+g_fEKFSLAM_CovCurr[nThisLMstate[1]][nThisLMstate[2]])/(g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[0]]+g_fEKFSLAM_CovCurr[nThisLMstate[1]][nThisLMstate[1]]-g_fEKFSLAM_CovCurr[nThisLMstate[2]][nThisLMstate[2]]) );	// pitch : between X- and Z-axis
		//dRot[0] = 0.5*atan2( 2.0*g_dEKFSLAM_CovCurr[10+iLM_3][11+iLM_3], (g_dEKFSLAM_CovCurr[10+iLM_3][10+iLM_3]-g_dEKFSLAM_CovCurr[11+iLM_3][11+iLM_3]) );	// roll : between Y- and Z-axis


		if( g_fEKFSLAM_MeanCurr[nThisLMstate[2]] > g_fEKFSLAM_MeanCurr[2] )
			fRot[1] = -abs(fRot[1]);
		else
			fRot[1] = abs(fRot[1]);

		glPushMatrix();

		glTranslatef(1000.0f*g_fEKFSLAM_MeanCurr[nThisLMstate[0]], 1000.0f*g_fEKFSLAM_MeanCurr[nThisLMstate[1]], 1000.0f*g_fEKFSLAM_MeanCurr[nThisLMstate[2]]);
			
		glRotatef(fRot[2]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
		glRotatef(fRot[1]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch 
		//glRotated(dRot[0]*R2D, 1.0, 0.0, 0.0);	// roll

	
		fEllipsePos[0] = fEllipseAxis[0];
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = 0.0f;

		//x0=a+posX; y0=posY; z0=posZ;


		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*cos(i);
			fEllipsePos[1] = fEllipseAxis[1]*sin(i);
			//dEllipsePos[2] = 0.0;
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=posX; y0=b+posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = fEllipseAxis[1];
		fEllipsePos[2] = 0.0f;

		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			//dEllipsePos[0] = 0.0;
			fEllipsePos[1] = fEllipseAxis[1]*cos(i);
			fEllipsePos[2] = fEllipseAxis[2]*sin(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=a+posX; y0=posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = fEllipseAxis[2];
		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*sin(i);
			//dEllipsePos[1] = 0.0;
			fEllipsePos[2] = fEllipseAxis[2]*cos(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		glPopMatrix();
	}

}




GLvoid COpenglView::DrawLandmarkCovariance_InverseDepth()
{
// http://www.iph.ufrgs.br/corpodocente/marques/cd/multivar/eigen.htm
// Cov_xx, yy, zz는 ellipse의 major axis방향벡터
// Cov_xy, yz, xz는 각방향간의 관련성이므로 Cov가 0이면 서로 연관이 없으므로 구 형태가 된다.
// Cov_xy는 xy평면에 projection해볼때, 퍼져있는 정도를 나타내므로, xx,yy,zz벡터에 의해 회전이 들어간다면, 간단하게 xy를 간단하게 고려해 그릴수는 없고
// 회전에 맞게 euler transform을 반대로 넣어서 xy,yz,xz(ellipse굵기)를 표현해야 한다.

	// ellipsoid : x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
	// x = a*cos(lat)*cos(lon)
	// y = b*cos(lat)*sin(lon)
	// z = c*sin(lat)

	float fRot[3], fEllipsePos[3], fEllipseAxis[3];

	int iLM_Nums;
	float fItvRad = 15.0f*D2Rf;
	float fLMScale = 1000.0f;

	Mat matA( NUM_LANDMARKSTATES, NUM_LANDMARKSTATES, CV_32FC1 );
	cv::SVD svd1;

	glLineWidth(1.0f); 
	glColor3ub(255, 255, 0);  // 컬러 지정 

	float fDistToCov[3];	// two end points of cov. ellipse, near one[0] and farther one[1]. Center[2]

	int nThisLMstate[6] = {0, 0, 0, 0, 0, 0};
	for( int iLM=0; iLM<g_nNumEKFSLAMLMs; iLM++ )
	{
		iLM_Nums = iLM * NUM_LANDMARKSTATES;
		for( int ii=0; ii<NUM_LANDMARKSTATES; ii++ )
			nThisLMstate[ii] = NUM_ROBOTSTATES +iLM_Nums +ii;
		
		//for( int ii=0; ii<NUM_LANDMARKSTATES; ii++ )
		//	for( int jj=0; jj<NUM_LANDMARKSTATES; jj++ )
		//		matA.at<float>(ii,jj) = g_fEKFSLAM_CovCurr[nThisLMstate[0]+ii][nThisLMstate[0]+jj];
  
		//svd1.operator()( matA, SVD::MODIFY_A|SVD::NO_UV );
		if( g_fEKFSLAM_MeanCurr[nThisLMstate[5]] > 0.f )
		{
			fDistToCov[0] = 1.0f / (g_fEKFSLAM_MeanCurr[nThisLMstate[5]] +2.0f*g_fEKFSLAM_CovCurr[nThisLMstate[5]][nThisLMstate[5]]);	// near one
			fDistToCov[0] = MAX( MIN( fDistToCov[0], 1000.f ), 0.f );	// limit

			fDistToCov[1] = 1.0f / (g_fEKFSLAM_MeanCurr[nThisLMstate[5]] -2.0f*g_fEKFSLAM_CovCurr[nThisLMstate[5]][nThisLMstate[5]]);// farther one or negative one
			if( fDistToCov[1] < 0.f )
				fDistToCov[1] = 1000.f;	// limit
		}
		fDistToCov[2] = (fDistToCov[0]+fDistToCov[1])*0.5f;	// cov. center

		fEllipseAxis[0] = fLMScale*abs(fDistToCov[1]-fDistToCov[0])*0.5f;	// length of cov. 
		fEllipseAxis[1] = fLMScale*tan(2.0f*g_fEKFSLAM_CovCurr[nThisLMstate[3]][nThisLMstate[3]])*fDistToCov[2];	// yaw
		fEllipseAxis[2] = fLMScale*tan(2.0f*g_fEKFSLAM_CovCurr[nThisLMstate[4]][nThisLMstate[4]])*fDistToCov[2];	// pitch

		//fEllipseAxis[0] = fLMScale*( svd1.w.at<float>(0,0) );
		//fEllipseAxis[1] = fLMScale*( svd1.w.at<float>(1,0) );
		//fEllipseAxis[2] = fLMScale*( svd1.w.at<float>(2,0) );

		//fRot[2] = 0.5f*atan2( 2.0f*g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[1]], (g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[0]]-g_fEKFSLAM_CovCurr[nThisLMstate[1]][nThisLMstate[1]]) );	// yaw : between X- and Y-axis
		//fRot[1] = -0.5f*atan( 2.0f*(g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[2]]+g_fEKFSLAM_CovCurr[nThisLMstate[1]][nThisLMstate[2]])/(g_fEKFSLAM_CovCurr[nThisLMstate[0]][nThisLMstate[0]]+g_fEKFSLAM_CovCurr[nThisLMstate[1]][nThisLMstate[1]]-g_fEKFSLAM_CovCurr[nThisLMstate[2]][nThisLMstate[2]]) );	// pitch : between X- and Z-axis
		//dRot[0] = 0.5*atan2( 2.0*g_dEKFSLAM_CovCurr[10+iLM_3][11+iLM_3], (g_dEKFSLAM_CovCurr[10+iLM_3][10+iLM_3]-g_dEKFSLAM_CovCurr[11+iLM_3][11+iLM_3]) );	// roll : between Y- and Z-axis


		//if( g_fEKFSLAM_MeanCurr[nThisLMstate[2]] > g_fEKFSLAM_MeanCurr[2] )
		//	fRot[1] = -abs(fRot[1]);
		//else
		//	fRot[1] = abs(fRot[1]);

		glPushMatrix();

			glTranslatef(1000.0f*g_fEKFSLAM_MeanCurr[nThisLMstate[0]], 1000.0f*g_fEKFSLAM_MeanCurr[nThisLMstate[1]], 1000.0f*g_fEKFSLAM_MeanCurr[nThisLMstate[2]]);
			

			glRotatef(g_fEKFSLAM_MeanCurr[nThisLMstate[4]]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
			glRotatef(g_fEKFSLAM_MeanCurr[nThisLMstate[3]]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch 
			//glRotated(dRot[0]*R2D, 1.0, 0.0, 0.0);	// roll



			glTranslatef( 1000.f*fDistToCov[2], 0.f, 0.f);	// avg. of two points
			//glTranslatef(1000.0f/(0.001f+g_fEKFSLAM_MeanCurr[nThisLMstate[5]]), 0.f, 0.f);

			//x0=a+posX; y0=posY; z0=posZ;
			fEllipsePos[0] = fEllipseAxis[0];	fEllipsePos[1] = 0.0f;		fEllipsePos[2] = 0.0f;

			glBegin(GL_LINES);
			for( float i=0.0f; i<6.3f; i+=fItvRad )
			{
				glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
				fEllipsePos[0] = fEllipseAxis[0]*cos(i);
				fEllipsePos[1] = fEllipseAxis[1]*sin(i);
				glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
			}
			glEnd();

			//x0=posX; y0=b+posY; z0=posZ;
			//glColor3d(255,255,255);  // 컬러 지정
			fEllipsePos[0] = 0.0f;		fEllipsePos[1] = fEllipseAxis[1];		fEllipsePos[2] = 0.0f;

			glBegin(GL_LINES);
			for( float i=0.0f; i<6.3f; i+=fItvRad )
			{
				glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
				fEllipsePos[1] = fEllipseAxis[1]*cos(i);
				fEllipsePos[2] = fEllipseAxis[2]*sin(i);
				glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
			}
			glEnd();

			//x0=a+posX; y0=posY; z0=posZ;
			//glColor3d(255,255,255);  // 컬러 지정
			fEllipsePos[0] = 0.0f;		fEllipsePos[1] = 0.0f;		fEllipsePos[2] = fEllipseAxis[2];
			glBegin(GL_LINES);
			for( float i=0.0f; i<6.3f; i+=fItvRad )
			{
				glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
				fEllipsePos[0] = fEllipseAxis[0]*sin(i);
				fEllipsePos[2] = fEllipseAxis[2]*cos(i);
				glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
			}
			glEnd();

		glPopMatrix();
	}

}


GLvoid COpenglView::DrawRobotCovariance()
{
	float fRot[3], fEllipsePos[3], fEllipseAxis[3];
	float fItvRad = 15.0f*D2Rf;
	float fRobotScale = 1000.0f;


	glLineWidth(1.0f); 
	glColor3ub(255, 255, 0);  // 컬러 지정 

/*	Mat matA( 3, 3, CV_32FC1 );
	cv::SVD svd1;

	for( int ii=0; ii<3; ii++ )
		for( int jj=0; jj<3; jj++ )
			matA.at<float>(ii,jj) = g_fEKFSLAM_CovCurr[ii][jj];

	svd1.operator()( matA, SVD::MODIFY_A|SVD::NO_UV );

	// slightly bigger, but exact
	fEllipseAxis[0] = fRobotScale*sqrt( svd1.w.at<float>(0,0) );
	fEllipseAxis[1] = fRobotScale*sqrt( svd1.w.at<float>(1,0) );
	fEllipseAxis[2] = fRobotScale*sqrt( svd1.w.at<float>(2,0) );
*/
	// simple version
	fEllipseAxis[0] = fRobotScale*g_fEKFSLAM_CovCurr[0][0];
	fEllipseAxis[1] = fRobotScale*g_fEKFSLAM_CovCurr[1][1];
	fEllipseAxis[2] = fRobotScale*g_fEKFSLAM_CovCurr[2][2];

	fRot[2] = 0.5f*atan2( 2.0f*g_fEKFSLAM_CovCurr[0][1], (g_fEKFSLAM_CovCurr[0][0]-g_fEKFSLAM_CovCurr[1][1]) );	// yaw : between X- and Y-axis
	fRot[1] = -0.5f*atan2( 2.0f*g_fEKFSLAM_CovCurr[0][2], (g_fEKFSLAM_CovCurr[0][0]-g_fEKFSLAM_CovCurr[2][2]) );	// pitch : between X- and Z-axis
	fRot[0] = 0.5f*atan2( 2.0f*g_fEKFSLAM_CovCurr[1][2], (g_fEKFSLAM_CovCurr[1][1]-g_fEKFSLAM_CovCurr[2][2]) );	// roll : between Y- and Z-axis

	glPushMatrix();

		glTranslatef(1000.0f*g_fEKFSLAM_MeanCurr[0], 1000.0f*g_fEKFSLAM_MeanCurr[1], 1000.0f*g_fEKFSLAM_MeanCurr[2]);

		glRotatef(fRot[2]*R2Df, 0.0f, 0.0f, 1.0f);	// yaw
		glRotatef(fRot[1]*R2Df, 0.0f, 1.0f, 0.0f);	// pitch 
		glRotatef(fRot[0]*R2Df, 1.0f, 0.0f, 0.0f);	// roll

		fEllipsePos[0] = fEllipseAxis[0];
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = 0.0f;

		//x0=a+posX; y0=posY; z0=posZ;


		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*cos(i);
			fEllipsePos[1] = fEllipseAxis[1]*sin(i);
			//dEllipsePos[2] = 0.0;
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=posX; y0=b+posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = fEllipseAxis[1];
		fEllipsePos[2] = 0.0f;

		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			//dEllipsePos[0] = 0.0;
			fEllipsePos[1] = fEllipseAxis[1]*cos(i);
			fEllipsePos[2] = fEllipseAxis[2]*sin(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

		//x0=a+posX; y0=posY; z0=posZ;
		//glColor3d(255,255,255);  // 컬러 지정
		fEllipsePos[0] = 0.0f;
		fEllipsePos[1] = 0.0f;
		fEllipsePos[2] = fEllipseAxis[2];
		glBegin(GL_LINES);
		for( float i=0.0f; i<6.3f; i+=fItvRad )
		{
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2] );//x0, y0, z0);	
			fEllipsePos[0] = fEllipseAxis[0]*sin(i);
			//dEllipsePos[1] = 0.0;
			fEllipsePos[2] = fEllipseAxis[2]*cos(i);
			glVertex3f(fEllipsePos[0], fEllipsePos[1], fEllipsePos[2]);
		}
		glEnd();

	glPopMatrix();

}
#endif


#ifdef HS2_MCL3D_RELATED
GLvoid COpenglView::DrawMcl3dParticles()
{
	//assert( g_Mcl3dStates.nNumCurrParticle <= NUM_MAX_MCL3D_PARTICLES );

	for( int i=MIN(g_Mcl3dStates.nNumCurrParticle,NUM_MAX_MCL3D_PARTICLES)-1; i>=0; i-- )
	{
		glPushMatrix();
			glTranslatef(g_Mcl3dStates.fParticlePose[i][0]*1000.f, g_Mcl3dStates.fParticlePose[i][1]*1000.f, g_Mcl3dStates.fParticlePose[i][2]*1000.f);
			
			glRotatef(g_Mcl3dStates.fParticlePose[i][5]*R2DF, 0.0f, 0.0f, 1.0f);	// yaw
			glRotatef(g_Mcl3dStates.fParticlePose[i][4]*R2DF, 0.0f, 1.0f, 0.0f);	// pitch
			glRotatef(g_Mcl3dStates.fParticlePose[i][3]*R2DF, 1.0f, 0.0f, 0.0f);	// roll

			glColor3ub(255, 0, 0);  // 컬러 지정 
	
			glutSolidCube(50.0);
			glColor3ub(255, 255, 255);  // 컬러 지정 
			glutWireCube(50.0);
		glPopMatrix();
	}
}
#endif


#ifdef HS2_OCTREE_RELATED
	#ifdef USE_NVIDIA_GPU_FOR_OPENGL	// GPU related

GLvoid COpenglView::DrawOctree()
{
		glLineWidth(2.f);
		double dCubeSize;
		float fCubeSize;

		int nCount = 0;
		g_Octree1.m_bContinueLoop = 1;	// Octree의 끝까지 순회했는지(0)를 확인해준다

		// m_iLvNodeIdx[ii] : 각 treelevel에서 현재 몇번째 노드까지 왔는지 인덱스를 기록해준다. 처음에는 0에서 시작.
		for( int ii=0; ii<20; ii++ )
			g_Octree1.m_iLvNodeIdx[ii] = 0;

		// HeadNode부터 시작해서 Octree의 끝까지 순회한다.
		g_Octree1.m_CurrentNode = g_Octree1.m_HeadNode;

			// Draw HeadNode
			fCubeSize = (g_Octree1.m_CurrentNode->Bound[0][1]-g_Octree1.m_CurrentNode->Bound[0][0]);
			glPushMatrix();
				glTranslatef( g_Octree1.m_CurrentNode->Center[0], g_Octree1.m_CurrentNode->Center[1], g_Octree1.m_CurrentNode->Center[2] );
				if( g_Octree1.m_CurrentNode->IsOccupied )
				{
					glColor3ub(255,0,0);
				}
				else
				{
					glColor3ub(255,255,255);
				}
			glPopMatrix();

		g_Octree1.m_CurrentNode = g_Octree1.FindFirstLeafNode( g_Octree1.m_CurrentNode );
		
		while( g_Octree1.m_bContinueLoop )
		{
			// Draw CurrentNode
			fCubeSize = (g_Octree1.m_CurrentNode->Bound[0][1]-g_Octree1.m_CurrentNode->Bound[0][0]);
			glPushMatrix();
				glTranslatef( g_Octree1.m_CurrentNode->Center[0], g_Octree1.m_CurrentNode->Center[1], g_Octree1.m_CurrentNode->Center[2] );

				if( g_Octree1.m_CurrentNode->IsOccupied )
				{
					//if( g_Octree1.m_CurrentNode->Color[2]+g_Octree1.m_CurrentNode->Color[1]+g_Octree1.m_CurrentNode->Color[0] > 0 )
					{
						//glColor3ub(g_Octree1.m_CurrentNode->Color[2], g_Octree1.m_CurrentNode->Color[1], g_Octree1.m_CurrentNode->Color[0] );
					}
					//else
					{
						GPU_DrawOctreeCell( fCubeSize, g_Octree1.m_CurrentNode->Color );
					}
					nCount++;
				}
				else
				{
				}
			glPopMatrix();
			
			g_Octree1.m_CurrentNode = g_Octree1.FindNextLeafNode( g_Octree1.m_CurrentNode );
		}
}

#else

GLvoid COpenglView::DrawOctree()
{
		glLineWidth(2.f);
		double dCubeSize;
		float fCubeSize;

		int nCount = 0;
		g_Octree1.m_bContinueLoop = 1;	// Octree의 끝까지 순회했는지(0)를 확인해준다

		// m_iLvNodeIdx[ii] : 각 treelevel에서 현재 몇번째 노드까지 왔는지 인덱스를 기록해준다. 처음에는 0에서 시작.
		for( int ii=0; ii<20; ii++ )
			g_Octree1.m_iLvNodeIdx[ii] = 0;

		// HeadNode부터 시작해서 Octree의 끝까지 순회한다.
		g_Octree1.m_CurrentNode = g_Octree1.m_HeadNode;

			// Draw HeadNode
//			dCubeSize = (double)(g_Octree1.m_CurrentNode->Bound[0][1]-g_Octree1.m_CurrentNode->Bound[0][0]);
			fCubeSize = (g_Octree1.m_CurrentNode->Bound[0][1]-g_Octree1.m_CurrentNode->Bound[0][0]);
			glPushMatrix();
				glTranslatef( g_Octree1.m_CurrentNode->Center[0], g_Octree1.m_CurrentNode->Center[1], g_Octree1.m_CurrentNode->Center[2] );
				if( g_Octree1.m_CurrentNode->IsOccupied )
				{
					glColor3ub(255,0,0);
//					glutSolidCube(dCubeSize);
				}
				else
				{
					glColor3ub(255,255,255);
//					glutWireCube(dCubeSize);
				}
			glPopMatrix();
	//printf("%d(%d,%d,%d)", g_Octree1.CurrentNode->IsOccupied, (g_Octree1.CurrentNode->Center[0]/1000), (g_Octree1.CurrentNode->Center[1]/1000), (g_Octree1.CurrentNode->Center[2]/1000) );
	//printf("[%d%d%d] ", g_Octree1.m_iLvNodeIdx[0], g_Octree1.m_iLvNodeIdx[1], g_Octree1.m_iLvNodeIdx[2] );

		g_Octree1.m_CurrentNode = g_Octree1.FindFirstLeafNode( g_Octree1.m_CurrentNode );
		
		while( g_Octree1.m_bContinueLoop )
		{
			// Draw CurrentNode
			dCubeSize = (double)(g_Octree1.m_CurrentNode->Bound[0][1]-g_Octree1.m_CurrentNode->Bound[0][0]);
			glPushMatrix();
				glTranslatef( g_Octree1.m_CurrentNode->Center[0], g_Octree1.m_CurrentNode->Center[1], g_Octree1.m_CurrentNode->Center[2] );
	//printf("%d(%d,%d,%d)", g_Octree1.CurrentNode->IsOccupied, (g_Octree1.CurrentNode->Center[0]/1000), (g_Octree1.CurrentNode->Center[1]/1000), (g_Octree1.CurrentNode->Center[2]/1000) );
	//printf("[%d%d%d] ", g_Octree1.m_iLvNodeIdx[0], g_Octree1.m_iLvNodeIdx[1], g_Octree1.m_iLvNodeIdx[2] );

				if( g_Octree1.m_CurrentNode->IsOccupied )
				{
					if( g_Octree1.m_CurrentNode->Color[2]+g_Octree1.m_CurrentNode->Color[1]+g_Octree1.m_CurrentNode->Color[0] > 0 )
					{
						//glColor3ub(g_Octree1.m_CurrentNode->Color[2], g_Octree1.m_CurrentNode->Color[1], g_Octree1.m_CurrentNode->Color[0] );
					}
					else
					{
						glColor3ub(0,255,0);
						glutWireCube(dCubeSize);
						glColor3ub(0,0,0);
						glutSolidCube(dCubeSize);
					}
					nCount++;
				}
				else
				{
					//glColor3f(1.f, 1.f, 1.f);
					//glutWireCube(CubeSize);
				}
			glPopMatrix();
			
			//printf("%d Node Center:(%d, %d, %d) TreeLV:%d  #%d  ", nCount++, g_Octree1.CurrentNode->Center[0], g_Octree1.CurrentNode->Center[1], g_Octree1.CurrentNode->Center[2], g_Octree1.CurrentNode->TreeLevel, g_Octree1.m_iLvNodeIdx[g_Octree1.CurrentNode->TreeLevel] );
			//printf("(%d) [%d] %d \n", g_Octree1.CurrentNode->IsLeaf, g_Octree1.CurrentNode->IsOccupied, g_Octree1.CurrentNode->Color[2] );
			g_Octree1.m_CurrentNode = g_Octree1.FindNextLeafNode( g_Octree1.m_CurrentNode );
		}
	//printf("Drawn Points %d\n", nCount );
}
	#endif



GLvoid COpenglView::StoreAllOctreeNodeAsPoints()
{

	CTime today = CTime::GetCurrentTime();
	char output_file[80];
	sprintf_s( output_file, sizeof(output_file), "octreelog_%02d%02d%02d_%02dh%02dm%02ds.dat", 
				today.GetYear()-2000, today.GetMonth(), today.GetDay(), today.GetHour(), today.GetMinute(), today.GetSecond() );

	FILE* pDataFile;
	pDataFile = fopen(output_file, "w");

	int nCount = 0;
	g_Octree1.m_bContinueLoop = 1;	// Octree의 끝까지 순회했는지(0)를 확인해준다

	// m_iLvNodeIdx[ii] : 각 treelevel에서 현재 몇번째 노드까지 왔는지 인덱스를 기록해준다. 처음에는 0에서 시작.
	for( int ii=0; ii<20; ii++ )
		g_Octree1.m_iLvNodeIdx[ii] = 0;

	// HeadNode부터 시작해서 Octree의 끝까지 순회한다.
	g_Octree1.m_CurrentNode = g_Octree1.m_HeadNode;

	g_Octree1.m_CurrentNode = g_Octree1.FindFirstLeafNode( g_Octree1.m_CurrentNode );
		
	while( g_Octree1.m_bContinueLoop )
	{

		if( g_Octree1.m_CurrentNode->IsOccupied )
		{
			fprintf( pDataFile, "%.5f %.5f %.5f\n", g_Octree1.m_CurrentNode->Center[0], g_Octree1.m_CurrentNode->Center[1], g_Octree1.m_CurrentNode->Center[2] );

			//drawOctreeCell( fCubeSize );

			nCount++;

		}
		g_Octree1.m_CurrentNode = g_Octree1.FindNextLeafNode( g_Octree1.m_CurrentNode );
	}
	//printf("Drawn Points %d\n", nCount );

	fclose( pDataFile );
}



GLvoid COpenglView::ReadAllPointCloudsToMakeOctree( int iFile )
{
	if( (g_States.Octree_On==1) || (g_States.Octree_On==2) )
		return;

	char *cFileName;
	if( iFile == 0 )
		cFileName = GetDATfromFileDialog();
	else
		cFileName = "octreelog_130225_16h55m17s.dat";

	std::ifstream m_FileToRead;
	m_FileToRead.open( cFileName, std::ios_base::out );
	if( m_FileToRead.is_open() == false )
		return;

	printf( "ReadAllPointCloudsToMakeOctree:started.\n");

	g_Octree1.InitializeOctree( 50, 9, 0 );	//64, 7, 0
	//g_Octree1.InitializeOctree( 200, 9, 30000 );	//64, 7, 0

	float fPtlPos[3];
	unsigned char rgb_default[3] = {0,0,0};

	float fHeightOrigin = -260.f;	// [mm] compensation for defalut height of the origin
	
	while( m_FileToRead.eof() != true )
	{
		m_FileToRead >> fPtlPos[0] >> fPtlPos[1] >> fPtlPos[2];
		//std::cout << fPtlPos[0] << " " << fPtlPos[1] << " " << fPtlPos[2] << std::endl;
		fPtlPos[2] = fPtlPos[2] + fHeightOrigin;	
		g_Octree1.InputData2(fPtlPos, rgb_default);
	}

	g_States.Octree_On = 3;

	m_FileToRead.close();
}



GLvoid COpenglView::ShowOctreeRayCasting()
{
	float roll = 0.f;
	float pitch = 0.f*D2Rf+(float)(g_TcpSensorData.ServoState[1])*D2Rf;
	float yaw = 0.f;

	float sinRoll = sin(roll);	float sinPitch = sin(pitch);	float sinYaw = sin(yaw);
	float cosRoll = cos(roll);	float cosPitch = cos(pitch);	float cosYaw = cos(yaw);

	float DCM[3][3];	// inetial to body
	DCM[0][0] = cosYaw*cosPitch;	
	DCM[0][1] = sinYaw*cosPitch;	
	DCM[0][2] = -sinPitch;
	DCM[1][0] = -sinYaw*cosRoll+cosYaw*sinPitch*sinRoll;	
	DCM[1][1] = cosYaw*cosRoll+sinYaw*sinPitch*sinRoll;	
	DCM[1][2] = cosPitch*sinRoll;	
	DCM[2][0] = sinYaw*sinRoll+cosYaw*sinPitch*cosRoll;	
	DCM[2][1] = -cosYaw*sinRoll+sinYaw*sinPitch*cosRoll;	
	DCM[2][2] = cosPitch*cosRoll;


	float DCM2[3][3];
	float DCM12[3][3];

	// DCM(euler) to Unit vector 변환
	//float DCMtp_u[3] = {0.f, 0.f, 0.f};
	//for( int ii=0; ii<3; ii++ )
	//{
	//	for( int jj=0; jj<3; jj++ )
	//	{
	//		DCMtp_u[ii] += DCM[jj][ii]*u[jj];
	//	}
	//}



	float vec_l0[3] = { -2048.f, 0.f, 0.f };
	float vec_l[3] = { cos(0.277f)*sin(0.78f), cos(0.277f)*cos(0.78f), sin(0.277f) };
	float vec_endpt[3];
	float vec_u[3] = {1.f, 0.f, 0.f};


	//float vec_l[0] = 0.f;
	//float vec_l[1] = 0.f;
	//float vec_l[2] = g_dServoState[1]*D2Rf;

		//g_Octree1.RayCastingInOctree( vec_l0, vec_l, vec_endpt );

	int nDist, nOutofBound=0, nStuckedRay=0;
	for( float i=-135.f; i<135.1f; i+=0.25f )
	{
		DCM2[0][0] = cos(i*D2Rf);	
		DCM2[0][1] = sin(i*D2Rf);	
		DCM2[0][2] = 0.f;
		DCM2[1][0] = -sin(i*D2Rf);
		DCM2[1][1] = cos(i*D2Rf);
		DCM2[1][2] = 0.f;
		DCM2[2][0] = 0.f;
		DCM2[2][1] = 0.f;
		DCM2[2][2] = 1.f;

		for( int ii=0; ii<3; ii++ )
		{
			for( int jj=0; jj<3; jj++ )
			{
				DCM12[ii][jj] = 0.f;
				for( int kk=0; kk<3; kk++ )
				{
					DCM12[ii][jj] += DCM2[ii][kk]*DCM[kk][jj];
				}
			}
		}

		for( int ii=0; ii<3; ii++ )
		{
			vec_l[ii] = 0.f;
			for( int jj=0; jj<3; jj++ )
			{
				vec_l[ii] += DCM12[jj][ii]*vec_u[jj];
			}
		}

		//if( i>56.9 )
		//	printf("000");
		//vec_l[0] = cos(i*D2Rf)*cos(k*D2Rf);
		//vec_l[1] = sin(i*D2Rf)*cos(k*D2Rf);
		//vec_l[2] = sin(k*D2Rf);
		nDist = g_Octree1.RayCastingInOctree( vec_l0, vec_l, vec_endpt );
		if( nDist == 0 )
		{
			nOutofBound++; 
			//printf("OutOfBound ");
		}
		if( nDist == 1 )
		{
			nStuckedRay++;
			//printf("Stucked ray");
		}

		//printf("Dist%d  hor%.1f (%d,%d,%d)\n",nDist, i, vec_endpt[0], vec_endpt[1], vec_endpt[2] );

		//printf("ver%.1f, hor%.1f (%d,%d,%d)\n",i, k, vec_endpt[0], vec_endpt[1], vec_endpt[2] );


		glLineWidth(1.0f); 
		glColor3d(255, 255, 0);  // 컬러 지정 
		glBegin(GL_LINES);
		glVertex3f(vec_l0[0],vec_l0[1],vec_l0[2]);	 
		glVertex3f(vec_endpt[0], vec_endpt[1], vec_endpt[2] );
		glEnd();
	}

	//printf("nOutofBound %d   nStuckedRay %d\n",nOutofBound, nStuckedRay );


}
#endif





#ifdef USE_NVIDIA_GPU_FOR_OPENGL	// GPU related

void COpenglView::GPU_SetBuffer() 
{
	glewInit();

	fprintf(stdout, "OpenGL Version : %s\n", glGetString(GL_VERSION));
	fprintf(stdout, "OpenGL Vendor : %s\n", glGetString(GL_VENDOR));
/*	fprintf(stdout, "OpenGL Renderer : %s\n", glGetString(GL_RENDERER));
	fprintf(stdout, "OpenGL Extensions : %s\n", glGetString(GL_EXTENSIONS));
	
	if (glewIsSupported("GL_VERSION_3_0")) 
	{
		fprintf(stdout, "GL_VERSION_3_0\n");

        // Get Pointers To The GL Functions 
        glGenBuffersARB = (PFNGLGENBUFFERSARBPROC) wglGetProcAddress("glGenBuffersARB"); 
        glBindBufferARB = (PFNGLBINDBUFFERARBPROC) wglGetProcAddress("glBindBufferARB"); 
        glBufferDataARB = (PFNGLBUFFERDATAARBPROC) wglGetProcAddress("glBufferDataARB"); 
        glDeleteBuffersARB = (PFNGLDELETEBUFFERSARBPROC) wglGetProcAddress("glDeleteBuffersARB"); 

	}
*/
	// Load Vertex Data Into The Graphics Card Memory 

	float fScalefactor = 1.f;
	
	float fRadius = fScalefactor*0.5f;
//    v6--------- v5
//   /|           /|
//  v1------v0| |
//  | |          | |
//  | |v7------|-|v4
//  |/           |/
//  v2---------v3

// v0-v1-v2-v3
// v0-v3-v4-v5
// v0-v1-v6-v5
// v7-v6-v1-v2
// v7-v4-v3-v2
// v4-v7-v6-v5

	vertices[0] = fRadius;		vertices[1] = fRadius;		vertices[2] = fRadius;
	vertices[3] = -fRadius;		vertices[4] = fRadius;		vertices[5] = fRadius;
	vertices[6] = -fRadius;		vertices[7] = -fRadius;		vertices[8] = fRadius;
	vertices[9] = fRadius;		vertices[10] = -fRadius;	vertices[11] = fRadius; 

	vertices[12] = fRadius;		vertices[13] = fRadius;		vertices[14] = fRadius;
	vertices[15] = fRadius;		vertices[16] = -fRadius;	vertices[17] = fRadius;
	vertices[18] = fRadius;		vertices[19] = -fRadius;	vertices[20] = -fRadius;
	vertices[21] = fRadius;		vertices[22] = fRadius;		vertices[23] = -fRadius;

	vertices[24] = fRadius;		vertices[25] = fRadius;		vertices[26] = fRadius;
	vertices[27] = -fRadius;		vertices[28] = fRadius;		vertices[29] = fRadius;
	vertices[30] = -fRadius;	vertices[31] = fRadius;		vertices[32] = -fRadius;
	vertices[33] = fRadius;	vertices[34] = fRadius;		vertices[35] = -fRadius;

	vertices[36] = -fRadius;	vertices[37] = -fRadius;	vertices[38] = -fRadius;
	vertices[39] = -fRadius;	vertices[40] = fRadius;		vertices[41] = -fRadius;
	vertices[42] = -fRadius;	vertices[43] = fRadius;		vertices[44] = fRadius;
	vertices[45] = -fRadius;	vertices[46] = -fRadius;	vertices[47] = fRadius; 

	vertices[48] = -fRadius;	vertices[49] = -fRadius;	vertices[50] = -fRadius;
	vertices[51] = fRadius;		vertices[52] = -fRadius;	vertices[53] = -fRadius;
	vertices[54] = fRadius;		vertices[55] = -fRadius;	vertices[56] = fRadius;
	vertices[57] = -fRadius;	vertices[58] = -fRadius;	vertices[59] = fRadius;

	vertices[60] = fRadius;		vertices[61] = -fRadius;	vertices[62] = -fRadius;
	vertices[63] = -fRadius;	vertices[64] = -fRadius;	vertices[65] = -fRadius;
	vertices[66] = -fRadius;	vertices[67] = fRadius;		vertices[68] = -fRadius;
	vertices[69] = fRadius;		vertices[70] = fRadius;		vertices[71] = -fRadius;


	// 1.Generate a new buffer object
	glGenBuffers(1 , &m_vertexID );

	// 2.Bind the buffer object 
	glBindBuffer(GL_ARRAY_BUFFER , m_vertexID );
                                                                                                                 
	// 3.Copy vertex data to the buffer object 
	glBufferData(GL_ARRAY_BUFFER , sizeof(vertices) , vertices , GL_STATIC_DRAW);
} 

void COpenglView::GPU_DeleteBuffer()
{
	glDeleteBuffers(1,&m_vertexID);
}


void COpenglView::GPU_DrawOctreeCell( float fScale )
{
	glScalef( fScale, fScale, fScale );

	// bind VBOs for vertex array and index array
	glBindBuffer(GL_ARRAY_BUFFER , m_vertexID ); // for vertex coordinates

	// do same as vertex array except pointer
	glEnableClientState(GL_VERTEX_ARRAY);	// activate vertex coords array
	glVertexPointer(3 , GL_FLOAT , 0 , 0);  // last param is offset, not ptr

	// Draw black plane
	glColor3ub( 0,0,0 );
	glDrawArrays(GL_QUADS, 0, 24);

	glLineWidth( 2.f );
	glColor3ub( 0,255,0 );
    glDrawArrays(GL_LINE_STRIP, 4, 5);
    glDrawArrays(GL_LINE_STRIP, 12, 5);
	glDrawArrays(GL_LINES, 8, 4);
	glDrawArrays(GL_LINES, 16, 4);

	glDisableClientState(GL_VERTEX_ARRAY);	//  deactivate vertex array

	glBindBuffer(GL_ARRAY_BUFFER , 0 );
}


void COpenglView::GPU_DrawOctreeCell( float fScale, unsigned char *ubColor )
{
	glScalef( fScale, fScale, fScale );

	// bind VBOs for vertex array and index array
	glBindBuffer(GL_ARRAY_BUFFER , m_vertexID ); // for vertex coordinates

	// do same as vertex array except pointer
	glEnableClientState(GL_VERTEX_ARRAY);	// activate vertex coords array
	glVertexPointer(3 , GL_FLOAT , 0 , 0);  // last param is offset, not ptr

	// Draw black plane
	glColor3ub( 0,0,0 );
	glDrawArrays(GL_QUADS, 0, 24);

	glLineWidth( 2.f );
	glColor3ubv( ubColor );
    glDrawArrays(GL_LINE_STRIP, 4, 5);
    glDrawArrays(GL_LINE_STRIP, 12, 5);
	glDrawArrays(GL_LINES, 8, 4);
	glDrawArrays(GL_LINES, 16, 4);

	glDisableClientState(GL_VERTEX_ARRAY);	//  deactivate vertex array

	glBindBuffer(GL_ARRAY_BUFFER , 0 );
}

#endif	// GPU related











void MyMouseMove( int x, int y )
{
	//printf("At %d %d\n", x, y);

	if(glview1.m_LeftButtonDown)
	{
		CSize rotate = glview1.m_LeftDownPos - CPoint(x,y);
		glview1.m_LeftDownPos.SetPoint(x,y);
		glview1.m_dViewRotate[2] -= rotate.cx;
		glview1.m_dViewRotate[0] -= rotate.cy;
	}


	if(glview1.m_RightButtonDown)
	{
		CSize translate = glview1.m_RightDownPos - CPoint(x,y);
		glview1.m_RightDownPos.SetPoint(x,y);
		glview1.m_dLookAtCenter[1] += translate.cx*5.0;
		glview1.m_dLookAtCenter[0] -= translate.cy*5.0;
	}

	glutPostRedisplay();

}

void MyMouseButton( int button, int state, int x, int y )
{
/*
	// Wheel reports as button 3(scroll up) and button 4(scroll down)
	if ((button == 3) || (button == 4)) // It's a wheel event
	{
		// Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		if (state == GLUT_UP)
			return; // Disregard redundant GLUT_UP events

		printf("Scroll %s At %d %d\n", (button == 3) ? "Up" : "Down", x, y);
	}
	else
	{  // normal button event
		printf("Button %s At %d %d\n", (state == GLUT_DOWN) ? "Down" : "Up", x, y);
	}
*/
	if ( button == 3 ) // wheel-up event
	{
		glview1.m_dViewScale[2] += 500.f;
	}

	if ( button == 4 ) // wheel-down event
	{
		if( glview1.m_dViewScale[2] > 1000.0f )
			glview1.m_dViewScale[2] -= 500.f;
	}

	// change the viewpoint
	if( button == GLUT_MIDDLE_BUTTON )
	{
		if (state == GLUT_UP)	// only activate with GLUT_DOWN
			return;

		glview1.m_iViewpoint += 1;

		if( glview1.m_iViewpoint >= 4 )
			glview1.m_iViewpoint = 0;

	}


	if( button == GLUT_LEFT_BUTTON )
	{
		if (state == GLUT_DOWN)
		{
			glview1.m_iTimerOn = 0;
			glview1.m_LeftButtonDown = TRUE;
			glview1.m_LeftDownPos.SetPoint(x,y);			
		}
		else if (state == GLUT_UP)
		{
			glview1.m_LeftButtonDown = FALSE;
			glview1.m_iTimerOn = 1;
			glutTimerFunc(100, MyTimer, 1);
		}
	}
/*
	if( button == GLUT_RIGHT_BUTTON )
	{
		if (state == GLUT_DOWN)
		{
			glview1.m_iTimerOn = 0;
			glview1.m_RightButtonDown = TRUE;
			glview1.m_RightDownPos.SetPoint(x,y);			
		}
		else if (state == GLUT_UP)
		{
			glview1.m_RightButtonDown = FALSE;
			glview1.m_iTimerOn = 1;
			glutTimerFunc(100, MyTimer, 1);
		}
	}
*/
}



void MyKeyboard(int Key, int x, int y)
{
	switch(Key){
	
	case GLUT_KEY_PAGE_DOWN : 

		break;
	
	case GLUT_KEY_PAGE_UP : 
		
		break;
	
	case GLUT_KEY_UP :
		
		break;
	
	case GLUT_KEY_DOWN :
		
		break;
	
	case GLUT_KEY_LEFT :
		
		break;
	
	case GLUT_KEY_RIGHT :
		
		break;
	}
 glutPostRedisplay();
}



void MyMainMenu(int entryID)
{
	switch(entryID){
	case 1:

		break;
		
	case 2:

//		glutTimerFunc(10,MyTimer,1);
		break;

	case 3:
//		exit(0);
		break;

	case 4:
//		exit(0);
		break;

	case 5:
//		exit(0);
		break;

	case 6:
//		exit(0);
		break;


	default:
		break;
	}

	glutPostRedisplay();
}

void MySubMenu(int entryID)
{
	
	if(entryID == 1)
	{
		//glview1.StoreAllOctreeNodeAsPoints();
		//printf("StoreAllOctreeNodeAsPoints\n");
		//Sleep(100);
		//day = 0;
	}
	else if(entryID == 2)
	{
		//glview1.ReadAllPointCloudsToMakeOctree();
		//printf("Load Octree\n");
		//Sleep(100);
		//day = 1;
	}
	else if(entryID == 3)
	{
		//day = 2;
	}

	glutPostRedisplay();
}


/*





void COpenGLView::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	Invalidate(FALSE);	// WM_PAINT 메세지 발생 : 변화가 없더라도 강제로 뿌려줌

	fTimeCount += 0.05f;

	CView::OnTimer(nIDEvent);
}



void COpenGLView::OnLButtonDblClk(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_iViewpoint += 1;

	if( m_iViewpoint >= 3 )
		m_iViewpoint = 0;
	
	CView::OnLButtonDblClk(nFlags, point);
}


void COpenGLView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_LeftButtonDown = TRUE;
	m_LeftDownPos = point;
	KillTimer(0);

	CView::OnLButtonDown(nFlags, point);
}

void COpenGLView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_LeftButtonDown = FALSE;

	CView::OnLButtonUp(nFlags, point);
}

void COpenGLView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_RightButtonDown = TRUE;
	m_RightDownPos = point;
	KillTimer(0);

	CView::OnRButtonDown(nFlags, point);
}

void COpenGLView::OnRButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_RightButtonDown = FALSE;

	CView::OnRButtonUp(nFlags, point);
}

void COpenGLView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	{
		if(m_LeftButtonDown)
		{
			CSize rotate = m_LeftDownPos - point;
			m_LeftDownPos = point;
			zRotate -= rotate.cx;
			xRotate -= rotate.cy;
			InvalidateRect(NULL,FALSE);
		}
	}

	if(m_RightButtonDown)
	{
		CSize zDelta = m_RightDownPos - point;
		m_RightDownPos = point;

		if(zDelta.cy > 0)   // 휠이 위쪽으로 움직인 경우, 양수 발생
         {
               // wheel up   
				xScale += 0.1f;//0.00001f;
				yScale += 0.1f;//0.00001f;
				zScale += 1000.f;//0.00001f;
          }
          else      // 휠이 위쪽으로 움직인 경우, 음수 발생
          {
               // wheel down
				xScale -= 0.1f;//0.00001f;
				yScale -= 0.1f;//0.00001f;
				zScale -= 1000.f;//0.00001f;
          }
		InvalidateRect(NULL,FALSE);
	}
	CView::OnMouseMove(nFlags, point);
}

BOOL COpenGLView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: Add your message handler code here and/or call default
     if(zDelta) 
      {
          if(zDelta > 0)   // 휠이 위쪽으로 움직인 경우, 양수 발생
         {
               // wheel up   
				xScale += 0.1f;
				yScale += 0.1f;
				zScale += 0.1f;
				InvalidateRect(NULL,FALSE);                     
               //InvalidateRect(hWnd, NULL, TRUE);
          }
          else      // 휠이 위쪽으로 움직인 경우, 음수 발생
          {
               // wheel down
				xScale -= 0.1f;
				yScale -= 0.1f;
				zScale -= 0.1f;
				InvalidateRect(NULL,FALSE);               
               //InvalidateRect(hWnd, NULL, TRUE);
          }

      }

	return CView::OnMouseWheel(nFlags, zDelta, pt);
}

*/