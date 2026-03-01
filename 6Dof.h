#ifndef SIXDOFALGORITHM_H
#define SIXDOFALGORITHM_H
//六自由度位姿解算
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


typedef enum  { 
	No_Single,          //无
	Motion_Stop,        //停止
	Motion_Zero,        //回零
	Slow_UpTopMost,     //渐变（由快到慢）上升到最高
	Slow_ReachMiddle,   //到达中点位置
	Slow_DownLowest,    //缓慢到最低点
	Normal_Motion       //正常开启自动控制
}ControlSingles;        //控制信号

typedef enum  { 
	No_Mode,   //无模式
	Dx_Mode,   //x方向正弦
	Dy_Mode,   //y方向正弦
	Dz_Mode,   //z方向正弦
	Rx_Mode,   //x方向旋转
	Ry_Mode,   //y方向旋转
	Rz_Mode,   //z方向旋转
	Wave_Mode, //叠加海浪模式
	Net_Mode,  //网络控制
	File_Mode,  //文件控制
	Compensate_Mode,//补偿模式
}ControlModes;


typedef enum  {
	S_MIDDLE,
	S_TOPMOST,
	S_LOWEST,
	S_NOTSP
}SpicialPoint;//位置状态标志

typedef struct {
	int currentPos[6];
	double ctlOutput[6];
	float input_0[6];
	SpicialPoint currentPoint;
} DOF_PlatformData;


typedef struct{
	unsigned short m_Index;//下六自由度平台为0，上六自由平台为1
	double m_Amp[6];//
	double m_SampT;//
	double m_Frq[6];//
	double m_Ominga[6];//
	ControlModes m_CtlMode;//控制模式
	double m_E;//E=0
	double m_H;//中位时上下平台间的高度距离
	double m_RF;//上平台直径
	double m_RG;//下平台直径
	double m_GA;//上平台最临近铰点的角度
	double m_BE;//下平台最临近铰点的角度
	double m_DLT_Max;//缸中位的最大伸出量
	double m_LL_Mid;//缸中位长度  需要对其进行赋值，并去掉calInitParam中相应的赋值
	double offset[3];
} PlatformInitParameter;//结构参数和运动参数


class SixDofAlgorithm
{
public:
	SixDofAlgorithm();
	PlatformInitParameter m_CurrentPara;
	void initPlatformParameter(PlatformInitParameter InitPara);//输入平台参数
public:
	unsigned short INDEX;
	unsigned int counter;//控制开始后的循环次数
	//原始控制数据
	double	DX0;
	double	DY0;
	double	DZ0;
	double	WX0;//弧度
	double	WY0;//弧度
	double	WZ0;//弧度

	//滤波器输出后的控制数据
	double	DX;
	double	DY;
	double	DZ;
	double	WX;//弧度
	double	WY;//弧度
	double	WZ;//弧度

	double	PTX[12], PTY[12], PTZ[12];
	double  PTX0[12], PTY0[12], PTZ0[12];

	double sampT;
	/*限制范围*/
	double ampDX, ampDY, ampDZ, ampWX, ampWY, ampWZ;
	double frqDX, frqDY, frqDZ, frqWX, frqWY, frqWZ;
	double offsetX, offsetY, offsetZ;

	double DX_OUT, DY_OUT, DZ_OUT, WX_OUT, WY_OUT, WZ_OUT;
	//0--没有控制
	//1--stop
	//2--slow up
	//3--reach middle
	//4--slow down
	//5--mormal move
	ControlSingles preStep;
	ControlSingles signal;
	ControlSingles nextStep;
	//0--没有控制
	//1~6对应dx,dy,dz,rx,ry,rz
	//10--网络数据跟随控制
	//15--文件数据播放
	ControlModes ctlMode;

	bool isTopMost;
	bool isLowest;
	bool reachMidle;

	double	E, H/*中位时上下平台间的高度距离*/, RF, RG, GA, BE, DLT_max, LL_mid, LL_max, LL_min;
	double ominga[6];

	int cynLength[6];
	double ctlOutput[6];//输出杆伸长量
	int currentPos[6];

	int realCurrentPos[6];
	int realctlOutput[6];

	void  calInitParam(void);
	int   relationalTransformationIn(float realPos);
	float relationalTransformationOut(int controlOut);
	void  setMotionParam(double dx, double dy, double dz, double wx, double wy, double wz);
	void  tickMotionControl(void);
	bool  tickSlowUpTopMost(void);
	bool  tickSlowDown(void);
	bool  tickMiddlePosition();
	bool  leadZeroPosition();//导引回零

	bool isPaltformMiddle(void);
	bool isPaltformTopMost(void);
	bool isPlatformLowest(void) ; 
	void isReachSpecialPointPosition();
	//更新数据
	void updateDofData();
	DOF_PlatformData m_DOF_PlatformData;
	void initGetDof_PlatformData();
private:
	double	s1_mm;//fi1，	lxy1_mm,
	double	s2_mm;//lxy2_mm,,fi2
	double	s3_mm;//lxy3_mm,,fi3
	double	s4_mm;//fi1，	lxy1_mm,
	double	s5_mm;//lxy2_mm,,fi2
	double	s6_mm;//lxy3_mm,,fi3
	double  A[4][4];

private:
	double input_d_wx, input_wx, input_d_wy, input_wy, input_d_wz, input_wz;//输入信号的位移和速度
	double input_d_dx, input_dx, input_d_dy, input_dy, input_d_dz, input_dz;
	double pre_input_wx, pre_input_wy, pre_input_wz, pre_input_dx, pre_input_dy, pre_input_dz;
	double output_wx[2];//六路输出信号
	double output_wy[2];
	double output_wz[2];
	double output_dx[2];
	double output_dy[2];
	double output_dz[2];

	double work1_wx[3], work2_wx[3], work3_wx[3];
	double work1_wy[3], work2_wy[3], work3_wy[3];
	double work1_wz[3], work2_wz[3], work3_wz[3];
	double work1_dx[3], work2_dx[3], work3_dx[3];
	double work1_dy[3], work2_dy[3], work3_dy[3];
	double work1_dz[3], work2_dz[3], work3_dz[3];

	void fkt(double y[], double f[], double input, double input_d, double epsin, double omiga);
	void rgkt(int n/*方程个数*/, double step, double(*y)[2]/*存放应变量初值，返回结果*/, double work1[], double work2[], double work3[], double input, double input_d, double epsin, double omiga);

};

#endif	// __FIRE_H