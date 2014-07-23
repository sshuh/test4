// Sungsik Huh
// 2013.09.23
// 
#ifndef HS2DEF_H_
#define HS2DEF_H_
#endif /* HS2DEF_H_ */

#pragma once



#ifndef PI
	#define PI		3.14159265358979
#endif
#ifndef PIf
	#define PIf		3.14159265358979f
#endif
#ifndef D2R
	#define D2R	0.01745329251994327	//(PI/180.)
#endif
#ifndef R2D
	#define R2D	57.2957795130823799	//(180./PI)
#endif
#ifndef D2Rf
	#define D2Rf	0.01745329251994327f	//(PIf/180.f)
#endif
#ifndef R2Df
	#define R2Df	57.2957795130823799f	//(180.f/PIf)
#endif
#ifndef D2RF
	#define D2RF	0.01745329251994327f	//(PIf/180.f)
#endif
#ifndef R2DF
	#define R2DF	57.2957795130823799f	//(180.f/PIf)
#endif

#ifndef MIN
	#define MIN(x,y) (((x) < (y)) ? (x):(y))
#endif

#ifndef MAX
	#define MAX(x,y) (((x) < (y)) ? (y):(x))
#endif

#ifndef L2_NORM_SQ_2D
	#define L2_NORM_SQ_2D(x1,x2,y1,y2)		((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
#endif

#ifndef L2_NORM_SQ_3D
	#define L2_NORM_SQ_3D(x1,x2,y1,y2,z1,z2)		((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2))
#endif

#ifndef L2_NORM_SQ_2D0
	#define L2_NORM_SQ_2D0(x,y)		((x)*(x) +(y)*(y))
#endif

#ifndef L2_NORM_SQ_3D0
	#define L2_NORM_SQ_3D0(x,y,z)		((x)*(x) +(y)*(y) +(z)*(z))
#endif

//-----------------------------------------------------------------------------------------------------------

#define VEHICLE_QUADROTOR	1
#define VEHICLE_AUTOCAR		2

#define VEHICLE_TYPE	VEHICLE_QUADROTOR

#define USE_OPENMP_FOR_MCL3D	1

#define USE_NVIDIA_GPU_FOR_OPENGL	1

#define FSLAM_VERSION		4

#define PROCESS_STORING_IMAGE_PATCH			0

// ---for groundstation---
#define REMOTE_REC_ON		1
#define	REMOTE_REC_OFF		2
//#define 			4
// -----------------------

#pragma pack(push, 1)
typedef struct {
	int Header;	// 20095180
	int Status;	// 
	double RobotState[24];	// processed result: pos3, vel3, att3, angvel3, poscov3, reserved9
	double IMUdata[12];		// 
	double GPSdata[12];		// LLH3, LCC3, vel3, poscov3
	double ServoState[3];
	short MotorControl[10];
	unsigned short LRFdata[1081];
	int OtherStates[10];	// reserved10
} StoredRobotStates_New;


typedef struct {
	//HEADER header;
	double RobotPos[3];
	double RobotVel[3];
	double RobotAtt[3];
	double IMUdata[12];
	double ServoState[3];
	short MotorControl[40];//	int MotorControl[20];

	short ImageProperty[3];
	unsigned short LRFdata[1081];

	//WORD DataChecksum;
} StoredRobotStates_Legacy;



typedef struct {
	char ID[4];
	unsigned int Length;
} StructCommHeader;


// Packet : Client -> Server
typedef struct 
{
	StructCommHeader Header;

	short Dynamixel_Mode;
	double Dynamixel_GoalAngle;
	short Dynamixel_GoalVel;

	unsigned int CheckSum;
} StructCommandUplink;


// Packet : Server -> Client
typedef struct {
	StructCommHeader Header;

	unsigned int Timestamp;
	int SensorStatus[4];
	double IMUdata[12];		// 96
	double ServoState[3];		// 24
	unsigned short LRFdata[1081];	// 2*1081 = 2162

	unsigned int CheckSum;
} StructSensorsDownlink;

// Packet : Server -> Client
typedef struct {
	StructCommHeader Header;

	unsigned int Timestamp;	// [ms]
	unsigned short Laserdata[1081];	// 2*1081 = 2162

	unsigned int CheckSum;
} StructLaserdataDownlink;


// Packet : Server -> Client
typedef struct {
	char Header[24];

	float EKFSLAM_MeanCurr[12];
	int NumEKFSLAMLMs;

	int CheckSum;
} StructEkfSlamDownlink;


// ---- Dynamixel code ----
typedef struct 
{
	unsigned char ucServoID;
	unsigned char ucServoModel;

	short sEncoderTrim;
	short sEncoderMin;
	short sEncoderMax;

	double dAngleLimitMin;
	double dAngleLimitMax;

	double dRad2Encoder;
	double dDeg2Encoder;
	double dEncoder2Deg;

	double dAutoTiltAngleMin;
	double dAutoTiltAngleMax;

} StructDynamixelParameters;

typedef struct {
	unsigned char ucServoID;
	short sControlMode;
	double dCurrentAngle; // [deg]
	double dGoalAngle;
	
	short sCurrentVel;
	short sGoalVel;

} StructDynamixelStates;
// ---------------------------


// Packet : Mobile Client -> Server
typedef struct 
{
	int Command[2];

} StructMobileUplink;


// Packet : Server -> Mobile Client
typedef struct {
	int Status[2];

} StructMobileDownlink;


// Packet :  Client -> Bearnav Server
typedef struct 
{
	unsigned short SyncWord;	// =0x81ff

	double TimeTag;
    double llhSlam[3];  // Estimated Position in lat lon height, rad,rad, m
    float velSlam[3];   // Estimated Velocity:  NED, m/s
    float stddevSlam[4];   // Estimated stddev=sqrt(Covariance): [0] hor pos-x cov, [1] hor pos-y cov, [2] vertical pos cov, [2] velocity cov
    char  Flag;        // 0: not ready, 1: ok to use, ...
	short sFlagHS2;	// for obstacle avoidance
	short Checksum;
} PCK_COMMAND_BEARNAV;


// Packet : Bearnav Server ->  Client	// M6002
typedef struct 
{
	unsigned short SyncWord;	// =0x81ff
	
	//HEADERWOM header;
	unsigned short MessageID;	// =6002
	unsigned short WordCount;	// =(sizeof(M6002)-sizeof(HEADERWOM)-2-2)/2
	unsigned short Flag;		// =VEHICLE_ID
	unsigned short HeaderChecksum;	// =-0x81ff-m6002.header.MessageID-m6002.header.WordCount-m6002.header.Flag

	double GPSTime;
	float accel[3];
	float pqr[3];
	double llh[3];	//lat, longe, height
	float	vel_ned[3];
	float	attitude[3];  // roll, pitch, yaw, in radian
	float	vel_bxyz[3];
	float	lcc[3];
	unsigned short   FailureCode;
	unsigned short   DataChecksum;

} PCK_SENSORDATA_BEARNAV;	// M6002



#ifndef NUM_MAX_MCL3D_PARTICLES
	#define NUM_MAX_MCL3D_PARTICLES		120
#endif
typedef struct 
{
	double dMeanPos[3];
	double dMeanVel[3];
	double dMeanAtt[3];
	double dCovPos[3];

	//static const int nNumMaxPaticle = 120;
	int nNumCurrParticle;// = NUM_MAX_MCL3D_PARTICLES;
	float fParticlePose[NUM_MAX_MCL3D_PARTICLES][6];
	float fLccPosPrev[3];
	float fLccPosCurr[3];
	int flag;
} MCL3dStates;


typedef struct 
{
	double dMeanPos[3];
	double dMeanVel[3];
	double dMeanAtt[3];
	double dCovPos[3];

	float fLccPosPrev[3];
	float fLccPosCurr[3];
	int flag;
} EstimatedVehiclePoseStates;


typedef struct
{
	int CAM_On;
	int LRF_On;
	int IMU_On;
	int Motor_On;

	int TcpComm_On;
	int TcpCommBearnav_On;
	int TcpCommMobile_On;

	int RemoteComm_States_On;
	int RemoteComm_Video_On;

	int GPS_On;

	int LaserPointsViewMode;	// opengl


	int Octree_On;
	int MCL3D_On;
	int FusionSLAM_On;

	int CamPixelToLaserData;

	int DataPlot_On;
	int OpenGL_On;

	int ReplayDataVideoLegacy_On;
	int ReplayDataVideo_On;
	int ReplayVideo_On;
	int ReplayData_On;
	
	int LogDataVideo_On;
	int LogData_On;
	int LogVideo_On;

	int LogMCL_On;

	int SendCommandToServer;
	int ServerStatus;

	int Test;
	
	int LaserCamSLAM_On;

	int Test_FeatureTrackingStandalone;
	
	enum
	{
		Off=0, Standby=1, PartiallyActivated=1, FullyActivated=2, On=2, On_Replay=5, On_Initialize=10, On_Loop=20, On_Terminate=30
	};	

} SharedStates; 


// calibrated initial camera and lrf w/ servo pose
// Body center : upper plate, center of the box
#if( VEHICLE_TYPE == VEHICLE_AUTOCAR )
// For BIG frame : car
	const double POSE_CAMfromBODY[6]	= { 0.0, 100.0, 0.0,	0.0, 0.0, 0.0 };	// [mm], [deg]	x,y,z,r,p,y
	const double POSE_SERVOfromBODY[6]	= {0.0, 0.0, 0.0,		0.0, 0.0, 0.0 };	// [mm], [deg]	x,y,z,r,p,y
	const double POSE_LASERfromSERVO[6] = {0.0, 0.0, -35.0,		0.0, 0.0, 0.0 };	// [mm], [deg]	x,y,z,r,p,y
#endif

#if( VEHICLE_TYPE == VEHICLE_QUADROTOR )
// For quadrotor frame
	const double POSE_CAMfromBODY[6]	= { 0.100, 0.045, 0.130,	0.0, -12.0, 0.0 };	// [m], [deg]	x,y,z,r,p,y -7.0
	const double POSE_SERVOfromBODY[6]	= { 0.060, -0.060, 0.120,	0.0, 0.0, 0.0 };	// [m], [deg]	x,y,z,r,p,y
	const double POSE_LASERfromSERVO[6] = { 0.060, 0.0, 0.0,		0.0, 0.0, 0.0 };	// [m], [deg]	x,y,z,r,p,y
#endif


// ------------ Laser Scanner ------------
typedef struct
{
	// laser scanner inner parameter: hokuyo
	char Laser_Model[100];
	int Laser_MinDist;
	int Laser_MaxDist;
	int Laser_TotalSteps;
	int Laser_FirstStep;
	int Laser_LastStep;
	int Laser_FrontStep;
	int Laser_ScanRPM;

	// user-defined parameter: hokuyo, sick
	int LeftEndStep;
	int CenterFrontStep;
	int RightEndStep;
	double LeftEndAngle;
	double RightEndAngle;
	double AngleResolution;

	int Left90DegStep;
	int Right90DegStep;

	int NumTotalMeas;

	float LUT_CosScanAngle[1081];
	float LUT_SinScanAngle[1081];

	void SetLaserVendorParameter( char* Model, int MinDist, int MaxDist, int TotalSteps, int FirstStep,	int LastStep, int FrontStep, int ScanRPM )
	{
		for( int i=0; i<sizeof(Model); i++ )
			Laser_Model[i] = Model[i];

		Laser_MinDist = MinDist;
		Laser_MaxDist = MaxDist;
		Laser_TotalSteps = TotalSteps;
		Laser_FirstStep = FirstStep;
		Laser_LastStep = LastStep;
		Laser_FrontStep = FrontStep;
		Laser_ScanRPM = ScanRPM;
	}

	void SetLaserUserDefinedParameter( double _AngleResolution, double _LeftEndAngle, double _RightEndAngle, int _Left90DegStep, int _Right90DegStep )
	{
		NumTotalMeas = Laser_LastStep -Laser_FirstStep +1;
		LeftEndStep = 0;//LaserParam1.Laser_FirstStep;
		RightEndStep = Laser_LastStep -Laser_FirstStep;
		CenterFrontStep = Laser_FrontStep -Laser_FirstStep;

		AngleResolution = _AngleResolution;	
		LeftEndAngle = _LeftEndAngle;
		RightEndAngle = _RightEndAngle;
		Left90DegStep = _Left90DegStep;
		Right90DegStep = _Right90DegStep;
	}

	void SetLaserSinCosScanAngleLUT()
	{
		//for(int i=LeftEndStep; i<=RightEndStep; i++)
		//{
		//	LUT_CosScanAngle[i] = (float)( cos(D2R*( LeftEndAngle+((double)(i-LeftEndStep)*AngleResolution) )) );
		//	LUT_SinScanAngle[i] = (float)( sin(D2R*( LeftEndAngle+((double)(i-LeftEndStep)*AngleResolution) )) );
		//}
	}

} StructLaserParameters;
#pragma pack(pop)

#ifdef __linux__

#ifndef Sleep
#define Sleep(x)	usleep(x*1000)
#endif

#endif

/*
#define URG04LX		1
#define UTM30LX		2
#define LMS291		3
#define LMS151_541	4
#define LMS151_1081	5

#if( VEHICLE_TYPE == VEHICLE_QUADROTOR )
	#define LASER_SCANNER		LMS151_1081//URG04LX	//LMS151_1081//URG04LX//UTM30LX
#endif

#if( VEHICLE_TYPE == VEHICLE_AUTOCAR )
	#define LASER_SCANNER		LMS151_1081
#endif

#if( LASER_SCANNER == URG04LX )	// first point 0 ~ first detection 44 ~ front 384 ~ end detection 725 ~ last point 768,  detection range 239.77 deg, angle resolution 360/1024
	#define LRF_START_ANG		-120.0		// MUST BE double type
	#define LRF_END_ANG			120.0	// MUST BE double type
	#define INIT_MEAS_STEP		44
	#define CENTER_MEAS_STEP	384	// shoud be int
	#define END_MEAS_STEP		725
	#define LRF_RES_ANG		0.3515625	//( LRF_START_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) - LRF_END_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) )//deg

	#define L_MEAS_STEP		129	// shoud be int
	#define R_MEAS_STEP		640	// should be int

	#define LRF_MAX_DIST		5600	//mm
	#define LRF_MEAS_PT			726
#endif

#if( LASER_SCANNER == UTM30LX )	// first point 0 ~ first detection 0 ~ front 520 ~ end detection 1080 ~ last point 1080,  detection range 270.25 deg, angle resolution 360/1440
	// CCW is inverted to CW	
	#define LRF_START_ANG		-135.0	//deg		// MUST BE double type
	#define LRF_END_ANG			135.0		//deg		// MUST BE double type
	#define INIT_MEAS_STEP		0	// shoud be int
	#define CENTER_MEAS_STEP		540	// shoud be int
	#define END_MEAS_STEP		1080	// should be int
	#define LRF_RES_ANG		0.25	//360.0/1440.0	//( LRF_START_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) - LRF_END_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) )//deg

	#define L_MEAS_STEP		180	// shoud be int
	#define R_MEAS_STEP		900	// should be int

	#define LRF_MAX_DIST	30000	//mm
	#define LRF_MEAS_PT		1081
#endif

#if( LASER_SCANNER == LMS151_541 )	// first point 0 ~ first detection 0 ~ front 520 ~ end detection 1080 ~ last point 1080,  detection range 270.25 deg, angle resolution 360/1440
	// CCW is inverted to CW	
	#define LRF_START_ANG		-135.0	//deg		// MUST BE double type
	#define LRF_END_ANG			135.0		//deg		// MUST BE double type
	#define INIT_MEAS_STEP		0	// shoud be int
	#define CENTER_MEAS_STEP		270	// shoud be int
	#define END_MEAS_STEP		540	// should be int
	#define LRF_RES_ANG		0.5	//360.0/1440.0	//( LRF_START_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) - LRF_END_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) )//deg

	#define L_MEAS_STEP		90	// shoud be int
	#define R_MEAS_STEP		450	// should be int

	#define LRF_MAX_DIST	50000	//mm
	#define LRF_MEAS_PT		541
#endif

#if( LASER_SCANNER == LMS151_1081 )	// first point 0 ~ first detection 0 ~ front 520 ~ end detection 1080 ~ last point 1080,  detection range 270.25 deg, angle resolution 360/1440
	// CCW is inverted to CW	
	#define LRF_START_ANG		-135.0	//deg		// MUST BE double type
	#define LRF_END_ANG			135.0		//deg		// MUST BE double type
	#define INIT_MEAS_STEP		0	// shoud be int
	#define CENTER_MEAS_STEP		540	// shoud be int
	#define END_MEAS_STEP		1080	// should be int
	#define LRF_RES_ANG		0.25	//360.0/1440.0	//( LRF_START_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) - LRF_END_ANG/(END_MEAS_STEP-INIT_MEAS_STEP) )//deg

	#define L_MEAS_STEP		0	// shoud be int
	#define R_MEAS_STEP		1080	// should be int

	#define LRF_MAX_DIST	50000	//mm
	#define LRF_MEAS_PT		1081
#endif
*/

#define LASER_UPDOWN_MIRROR_ON		1


// ------------ Camera ------------
#define MONO_TWAIN			1
#define MONO_FFMV			2
#define STEREO_BB2			5

#define ONBOARD_CAMERA		MONO_FFMV 


// MONO_FIREFLY_MV : ptgrey FireflyMV wide angle
#define CAM_FOCALL			340.0		// original estimated:340, undistorted:355~365
#define CAM_PP_HOR			320.0
#define CAM_PP_VER			240.0
#define CAM_RDIST_1		-0.23603
#define CAM_RDIST_2		0.04642
#define CAM_RDIST_3		-0.00261
#define CAM_RDIST_4		-0.00042




// ------------ Servo Motor ------------
/*
#define SERVO_RAD2TILTSERVO	195.56959407132119	// R2D * 512 signal / 150 deg
#define SERVO_DEG2TILTSERVO	3.41333333333			// 512 signal / 150 deg
const double SERVO_TILTSERVO2DEG	 = 0.29296875;				// 150 deg / 512 signal


#if( VEHICLE_TYPE == VEHICLE_QUADROTOR )
	const int SERVO_TILTSERVO_RANGEMIN = 480;
	const int SERVO_TILTSERVO_RANGEMAX = 700;
	const int SERVO_TILTSERVO_TRIM = 550;
#endif

#if( VEHICLE_TYPE == VEHICLE_AUTOCAR )
	const int SERVO_TILTSERVO_RANGEMIN = 480;
	const int SERVO_TILTSERVO_RANGEMAX = 700;
	const int SERVO_TILTSERVO_TRIM = 550;
#endif
*/

// ------------ IMU ------------
#define	IMU_NONE		1
#define	IMU_3DMGX3		2
#define	IMU_NTARS		3
#define	IMU_MYARS		4
#define IMU_3DMGX3_15	5

#define ONBOARD_IMU		IMU_3DMGX3//IMU_3DMGX3_15 



// ------------ etc ------------

//#define FLAG_TCPIPSTANDALONE	0
//#define FLAG_TCPIPSERVER			1
//#define FLAG_TCPIPCLIENT			2


#define PORT_BEARNAV		27902
//#define IPADDRESS_BEARNAV		"20.0.1.85"
#define IPADDRESS_BEARNAV		"10.0.1.70"

#define TCPPORT_COMMAND_UPLINK			9189
#define TCPPORT_SENSORDATA_DOWNLINK		9190

#define TCPPORT_MOBILE					9191
#define TCPPORT_VIDEO_DOWNLINK			9193

#define IPADDRESS_HS2SERVER		"10.0.1.153"
//#define IPADDRESS_HS2SERVER		"127.0.0.1"
//#define IPADDRESS_BEARNAV		"10.0.1.70"

/*
ptgrey FireflyMV f3.6mm 253x196mm 9x7cell cell28mm

Calibration results (with uncertainties):

Focal Length:          fc = [ 607.49412   605.56509 ] ?[ 2.34825   2.08009 ]
Principal point:       cc = [ 311.74297   237.74341 ] ?[ 2.01010   2.50804 ]
Skew:             alpha_c = [ 0.00000 ] ?[ 0.00000  ]   => angle of pixel axes = 90.00000 ?0.00000 degrees
Distortion:            kc = [ -0.43055   0.22127   0.00150   -0.00046  0.00000 ] ?[ 0.00785   0.02137   0.00117   0.00045  0.00000 ]
Pixel error:          err = [ 0.39906   0.42129 ]

Note: The numerical errors are approximately three times the standard deviations (for reference).




ptgrey FireflyMV wide angle
Focal Length:   340, 340  
Principal point: 320, 240
Distortion:            kc = [ -0.23603   0.04642   -0.00261   -0.00042  0.00000 ]

*/

