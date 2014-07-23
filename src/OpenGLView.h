#pragma once

#include "hs2def.h"

#ifdef USE_NVIDIA_GPU_FOR_OPENGL
	#include "OpenGL\glew.h"
	#include "OpenGL\wglew.h"
#endif

// COpenGLView view
#include "OpenGL\freeglut.h" 
//#include "OpenGL\gl.h" 
#include "OpenGL\glu.h"
#include "OpenGL\glaux.h"


#include "OpenCV2.4.3/include/opencv2/opencv.hpp"

#include "HS2_Utils.h"


class COpenglView
{
public:

	// view point, scale, rotate, ..
	int m_iViewpoint;
	GLdouble m_dLookAtCenter[3];
	GLdouble m_dViewRotate[3];
	GLdouble m_dViewScale[3];

	double m_dRobotPos[3];
	double m_dRobotAtt[3];

	int m_iTimerOn;

	void RenderScene(void);
	void RenderText();

	int m_cx;	// opengl screen size
	int m_cy;	// opengl screen size


	int m_width;
	int m_height;
	int m_exp;

	float m_size;

	CPoint m_RightDownPos;
	CPoint m_LeftDownPos;
	BOOL m_RightButtonDown;
	BOOL m_LeftButtonDown;
	BOOL m_Rotate;


//	void OnRButtonDown(UINT nFlags, CPoint point);

//	void OnRButtonUp(UINT nFlags, CPoint point);


//	GLvoid BuildFont(HDC hdc);
//	GLvoid glPrint(const char *text);  
//	GLvoid KillFont(GLvoid);   
	void GLInitialize();
	void GLTerminate();

	GLvoid Draw3dGrid( int nItv=1000, int nScale=50000 );
	GLvoid Draw3dAxis();

	int DetectObstacles();

	// ----- EKF SLAM related -----
	//GLvoid drawEllipsoid(double a, double b, double c, double posX, double posY, double posZ );
	
	GLvoid DrawExLandmark();
	GLvoid DrawLandmarkCovariance();
	GLvoid DrawLandmarkCovariance_InverseDepth();

	GLvoid DrawLandmarkCovariance_LaserCamSlam();
	GLvoid DrawRobotCovariance_LaserCamSlam();
	GLvoid DrawRobotCovariance();
	// ----------------------------

	double m_dTurnProp;	
	GLvoid DrawVehicle_Quadrotor();
	GLvoid DrawVehicle_AutoCar();
	GLvoid DrawVehicle();


	GLvoid DrawCameraFOV();
	
	GLvoid GetTransformationMatrix();	// get transformation matrix using fslam/imu/servostate angles

	GLvoid DrawLRF_LatestData();

	float m_Rot_I2L[3][3];



	GLvoid DrawMcl3dParticles();


	// ----- Texture related -----
	GLvoid draw_Texture();
	AUX_RGBImageRec *m_pTextureImage[1];
	unsigned int m_TextureObject[1];
	AUX_RGBImageRec* LoadBMP( char* );
	int LoadGLTextures();
	// ---------------------------



	// ----- Vehicle Trace related -----
	GLvoid DrawVehicleTrace();
	float m_VehicleTrace[100][3];
	int m_nMaxVehicleTrace, m_nCurrentVehicleTrace;
	// ------------------------------------


	// ----- Point Cloud View related -----
	GLvoid DrawLRF_PointCloud();
	short*** m_LRF_PCD;
	unsigned char*** m_LRF_PCDcolor;
	int m_nMaxPCD, m_nCurrentPCD;
	// ------------------------------------


	// ----- Octree related -----
	GLvoid DrawOctree();
	GLvoid StoreAllOctreeNodeAsPoints();
	GLvoid ReadAllPointCloudsToMakeOctree( int iFile=0 );
	GLvoid ShowOctreeRayCasting();



#ifdef USE_NVIDIA_GPU_FOR_OPENGL	// GPU related
	GLuint m_vertexID;
	GLfloat vertices[72];
	void GPU_SetBuffer();
	void GPU_DeleteBuffer();
	void GPU_DrawOctreeCell( float fScale );
	void GPU_DrawOctreeCell( float fScale, unsigned char *ubColor );
#endif
	// --------------------------
};

