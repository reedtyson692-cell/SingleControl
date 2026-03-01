#include "6Dof.h"
#include <windows.h>
#include <math.h>
#include <stdio.h>

#define  PI 3.14159262728
#define  DU 57.295780//180/PI

#define FAST_UPDOWN 100            //快速运动
#define SLOW_UPDOWN 50             //慢速运动
#define SLOW_FAST_SHIFT 1000       //快慢运动切换1
#define REACH_ERROR 35             //到达目标点位的误差值

#define TICK_TOPMOST_POINT 10000   //最高点缸伸长量
#define TICK_MIDDLE_POINT  5000    //中间点缸伸长量
#define TICK_LOWSET_POINT  0       //最低点缸伸长量

SixDofAlgorithm::SixDofAlgorithm()
{
	DX0 = 0;
	DY0 = 0;
	DZ0 = 0;
	WX0 = 0;
	WY0 = 0;
	WZ0 = 0;
	DX = 0;
	DY = 0;
	DZ = 0;
	WX = 0;
	WY = 0;
	WZ = 0;
	counter = 0;
	signal = No_Single;
	nextStep = No_Single;
	isLowest = false;
	reachMidle = false;
	isTopMost = false;
	initGetDof_PlatformData();
}         

void SixDofAlgorithm::initGetDof_PlatformData()
{
	for (int i = 0; i < 6; i++)
	{
		m_DOF_PlatformData.ctlOutput[i] = 0;
		m_DOF_PlatformData.currentPos[i] = 0;
		m_DOF_PlatformData.input_0[i] = 0;
	}
	m_DOF_PlatformData.currentPoint = S_LOWEST;
}
//输入平台参数
void SixDofAlgorithm:: initPlatformParameter(PlatformInitParameter InitPara)
{
	m_CurrentPara = InitPara;

	INDEX = m_CurrentPara.m_Index;//1号平台

	//控制量初始值为0
	for (int i = 0; i<6; i++){
		ctlOutput[i] = 0;
	}

	ampDX = m_CurrentPara.m_Amp[0];//x平移280
	ampDY = m_CurrentPara.m_Amp[1];//y平移280
	ampDZ = m_CurrentPara.m_Amp[2];//z平移280
	ampWX = m_CurrentPara.m_Amp[3];//x转角(度)
	ampWY = m_CurrentPara.m_Amp[4];//y转角(度)
	ampWZ = m_CurrentPara.m_Amp[5];//z转角(度)

	sampT = m_CurrentPara.m_SampT;//0.01
	frqDX = m_CurrentPara.m_Frq[0]; frqDY = m_CurrentPara.m_Frq[1]; frqDZ = m_CurrentPara.m_Frq[2];
	frqWX = m_CurrentPara.m_Frq[3]; frqWY = m_CurrentPara.m_Frq[4]; frqWZ = m_CurrentPara.m_Frq[5];

	ominga[0] = m_CurrentPara.m_Ominga[0]; ominga[1] = m_CurrentPara.m_Ominga[1]; ominga[2] = m_CurrentPara.m_Ominga[2];
	ominga[3] = m_CurrentPara.m_Ominga[3]; ominga[4] = m_CurrentPara.m_Ominga[4]; ominga[5] = m_CurrentPara.m_Ominga[5];

	/*ominga[0] = 0.99; ominga[1] = 0.99; ominga[2] = 0.99;
	ominga[3] = 0.99; ominga[4] = 0.99; ominga[5] = 0.99;*/

	ctlMode = m_CurrentPara.m_CtlMode;

	E = m_CurrentPara.m_E;
	H = m_CurrentPara.m_H;//中位时上下平台间的高度距离
	RF = m_CurrentPara.m_RF;
	RG = m_CurrentPara.m_RG;
	GA = m_CurrentPara.m_GA;
	BE = m_CurrentPara.m_BE;
	DLT_max = m_CurrentPara.m_DLT_Max;//缸中位的最大伸出、缩回量,为行程的一半
	LL_mid = m_CurrentPara.m_LL_Mid;

	offsetX = m_CurrentPara.offset[0];
	offsetY = m_CurrentPara.offset[1];
	offsetZ = m_CurrentPara.offset[2];
}

void SixDofAlgorithm::calInitParam()
{
	//platformData.signal =0;
	/*上平台*/
	PTX[0] = RF / 2 * cos(GA / 2 / DU) - offsetX;
	PTY[0] = -RF / 2 * sin(GA / 2 / DU) - offsetY;
	PTZ[0] = -E - offsetZ;
	PTX[1] = -RF / 2 * cos(PI / 3 + GA / 2 / DU) - offsetX;
	PTY[1] = -RF / 2 * sin(PI / 3 + GA / 2 / DU) - offsetY;
	PTZ[1] = -E - offsetZ;
	PTX[2] = -RF / 2 * cos(PI / 3 - GA / 2 / DU) - offsetX;
	PTY[2] = -RF / 2 * sin(PI / 3 - GA / 2 / DU) - offsetY;
	PTZ[2] = -E - offsetZ;
	PTX[3] = -RF / 2 * cos(PI / 3 - GA / 2 / DU) - offsetX;
	PTY[3] = RF / 2 * sin(PI / 3 - GA / 2 / DU) - offsetY;
	PTZ[3] = -E - offsetZ;
	PTX[4] = -RF / 2 * cos(PI / 3 + GA / 2 / DU) - offsetX;
	PTY[4] = RF / 2 * sin(PI / 3 + GA / 2 / DU) - offsetY;
	PTZ[4] = -E - offsetZ;
	PTX[5] = RF / 2 * cos(GA / 2 / DU) - offsetX;
	PTY[5] = RF / 2 * sin(GA / 2 / DU) - offsetY;
	PTZ[5] = -E - offsetZ;

	/*下平台*/
	PTX[6] = RG / 2 * cos(PI / 3 - BE / 2 / DU) - offsetX;
	PTY[6] = -RG / 2 * sin(PI / 3 - BE / 2 / DU) - offsetY;
	PTZ[6] = -E - H - offsetZ;
	PTX[7] = RG / 2 * cos(PI / 3 + BE / 2 / DU) - offsetX;
	PTY[7] = -RG / 2 * sin(PI / 3 + BE / 2 / DU) - offsetY;
	PTZ[7] = -E - H - offsetZ;
	PTX[8] = -RG / 2 * cos(BE / 2 / DU) - offsetX;
	PTY[8] = -RG / 2 * sin(BE / 2 / DU) - offsetY;
	PTZ[8] = -E - H - offsetZ;
	PTX[9] = -RG / 2 * cos(BE / 2 / DU) - offsetX;
	PTY[9] = RG / 2 * sin(BE / 2 / DU) - offsetY;
	PTZ[9] = -E - H - offsetZ;
	PTX[10] = RG / 2 * cos(PI / 3 + BE / 2 / DU) - offsetX;
	PTY[10] = RG / 2 * sin(PI / 3 + BE / 2 / DU) - offsetY;
	PTZ[10] = -E - H - offsetZ;
	PTX[11] = RG / 2 * cos(PI / 3 - BE / 2 / DU) - offsetX;
	PTY[11] = RG / 2 * sin(PI / 3 - BE / 2 / DU) - offsetY;
	PTZ[11] = -E - H - offsetZ;

	for (int i = 0; i < 12; i++) {
		PTX0[i] = PTX[i];
		PTY0[i] = PTY[i];
		PTZ0[i] = PTZ[i];
	}
	//缸中位长度计算有问题

	//LL_mid=909.0;
	double L0[6];
	for (int i = 0; i<6; i++){
		L0[i] = sqrt((PTX[i + 6] - PTX[i])*(PTX[i + 6] - PTX[i]) + (PTY[i + 6] - PTY[i])*(PTY[i + 6] - PTY[i]) + (PTZ[i + 6] - PTZ[i])*(PTZ[i + 6] - PTZ[i]));
		//printf("初始化L0[%d]=%f\n",i,L0[i]);
	}
    //2017.10.13修改
	//LL_mid = (L0[0] + L0[1] + L0[2] + L0[3] + L0[4] + L0[5]) / 6.0;
	//printf("LL_mid=%f\n",LL_mid);
	LL_min = LL_mid - DLT_max;
	LL_max = LL_mid + DLT_max;

	input_d_wx = input_wx = input_d_wy = input_wy = input_d_wz = input_wz = 0.0;
	input_d_dx = input_dx = input_d_dy = input_dy = input_d_dz = input_dz = 0.0;
	pre_input_wx = pre_input_wy = pre_input_wz = pre_input_dx = pre_input_dy = pre_input_dz = 0.0;
	output_wx[0] = output_wx[1] = 0.0;
	output_wy[0] = output_wy[1] = 0.0;
	output_wz[0] = output_wz[1] = 0.0;
	output_dx[0] = output_dx[1] = 0.0;
	output_dy[0] = output_dy[1] = 0.0;
	output_dz[0] = output_dz[1] = 0.0;

	for (int i = 0; i < 6; i++){
		cynLength[i] = 0;
	}
}

void SixDofAlgorithm::setMotionParam(double dx, double dy, double dz, double wx, double wy, double wz)
{
	DX = dx; DY = dy, DZ = dz;
	WX = wx; WY = wy; WZ = wz;
	//没有滤波
	DX0 = DX; DY0 = DY; DZ0 = DZ;
	WX0 = WX; WY0 = WY; WZ0 = WZ;
}

//运动解析（输出ctlOutput）
void SixDofAlgorithm::tickMotionControl(void)
{
	double L1[6];
	double sin_WX, sin_WY, sin_WZ, cos_WX, cos_WY, cos_WZ;
	/*double DX_OUT, DY_OUT, DZ_OUT, WX_OUT, WY_OUT, WZ_OUT;*/
	//double dt = sampT;
	double dt = 0.02;
	//为何引入_OUT?
	//如果DX~WZ一直连续给出,则没有问题.
	//否则,比如播放文件中的暂停,虽然WX等不再给新数据,
	//但input_wx=WX;WX=output_wx[0];会导致WX自身不断改变
	input_wx = WX;
	input_d_wx = (input_wx - pre_input_wx) / dt;
	rgkt(2, dt, &output_wx, work1_wx, work2_wx, work3_wx, input_wx, input_d_wx, 1.0, 2 * ominga[0] * 3.1415926);//output[0]是位移，output[1]是速度
	WX_OUT = output_wx[0];
	//WX_OUT = WX;

	input_wy = WY;;//输入信号wy振动抑制
	input_d_wy = (input_wy - pre_input_wy) / dt;
	rgkt(2, dt, &output_wy, work1_wy, work2_wy, work3_wy, input_wy, input_d_wy, 1.0, 2 * ominga[1] * 3.1415926);//output[0]是位移，output[1]是速度
	WY_OUT = output_wy[0];
	//WY_OUT = WY;
	
	input_wz = WZ;//输入信号wz振动抑制
	input_d_wz = (input_wz - pre_input_wz) / dt;
	rgkt(2, dt, &output_wz, work1_wz, work2_wz, work3_wz, input_wz, input_d_wz, 1.0, 2 * ominga[2] * 3.1415926);//output[0]是位移，output[1]是速度
	WZ_OUT = output_wz[0];
	//WZ_OUT = WZ;

	//printf("inputWx %.1f,preInWx %.1f, Wx %.1f,inputWy %.1f,preInWy %.1f, Wy %.1f,inputWz %.1f,preInWz %.1f, Wz %.1f\n",input_wx*60,pre_input_wx*60,WX*60,input_wy*60,pre_input_wy*60,WY*60,input_wz*60,pre_input_wz*60,WZ*60);

	//输入平移信号控制算法
	input_dx = DX;//输入信号dx振动抑制
	input_d_dx = (input_dx - pre_input_dx) / dt;
	rgkt(2, dt, &output_dx, work1_dx, work2_dx, work3_dx, input_dx, input_d_dx, 1.0, 2 * ominga[3] * 3.1415926);//output[0]是位移，output[1]是速度
	DX_OUT = output_dx[0];
	//DX_OUT = DX;

	input_dy = DY;//输入信号dy振动抑制
	input_d_dy = (input_dy - pre_input_dy) / dt;
	rgkt(2, dt, &output_dy, work1_dy, work2_dy, work3_dy, input_dy, input_d_dy, 1.0, 2 * ominga[4] * 3.1415926);//output[0]是位移，output[1]是速度
	DY_OUT = output_dy[0];
	//DY_OUT = DY;

	input_dz = DZ;//输入信号dz振动抑制
	input_d_dz = (input_dz - pre_input_dz) / dt;
	rgkt(2, dt, &output_dz, work1_dz, work2_dz, work3_dz, input_dz, input_d_dz, 1.0, 2 * ominga[5] * 3.1415926);//output[0]是位移，output[1]是速度
	DZ_OUT = output_dz[0];
	//DZ_OUT = DZ;

	//printf("%f,%f,%f,%f,%f,%f\n", WX_OUT * 180 / 3.1415926, WY_OUT * 180 / 3.1415926,WZ_OUT * 180 / 3.1415926,DX_OUT * 180 / 3.1415926,DY_OUT * 180 / 3.1415926,DZ_OUT * 180 / 3.1415926);

	pre_input_wx = input_wx; pre_input_wy = input_wy; pre_input_wz = input_wz;
	pre_input_dx = input_dx; pre_input_dy = input_dy; pre_input_dz = input_dz;

	/*限制输出范围*/
	if (DX_OUT > ampDX)	DX_OUT = ampDX;
	if (DX_OUT<-1.0*ampDX)	DX_OUT = -1.0*ampDX;
	if (DY_OUT>ampDY)	DY_OUT = ampDY;
	if (DY_OUT<-1.0*ampDY)	DY_OUT = -1.0*ampDY;
	if (DZ_OUT>ampDZ)	DZ_OUT = ampDZ;
	if (DZ_OUT<-1.0*ampDZ)	DZ_OUT = -1.0*ampDZ;

	if (WX_OUT>ampWX / DU)	WX_OUT = ampWX / DU;
	if (WX_OUT<-1.0*ampWX / DU)	WX_OUT = -1.0*ampWX / DU;
	if (WY_OUT>ampWY / DU)	WY_OUT = ampWY / DU;
	if (WY_OUT<-1.0*ampWY / DU)	WY_OUT = -1.0*ampWY / DU;
	if (WZ_OUT>ampWZ / DU)	WZ_OUT = ampWZ / DU;
	if (WZ_OUT < -1.0*ampWZ / DU)	WZ_OUT = -1.0*ampWZ / DU;

	sin_WX = sin(WX_OUT);
	cos_WX = cos(WX_OUT);
	sin_WY = sin(WY_OUT);
	cos_WY = cos(WY_OUT);
	sin_WZ = sin(WZ_OUT);
	cos_WZ = cos(WZ_OUT);

	/*X-Y-Z固定角*/
	A[1][1] = cos_WZ * cos_WY;
	//A[1][2] = -cos_WZ * sin_WY * sin_WX + sin_WZ * cos_WX;
	A[1][2] = cos_WZ * sin_WY * sin_WX - sin_WZ * cos_WX;
	A[1][3] = cos_WZ * sin_WY * cos_WX + sin_WZ * sin_WX;
	//A[2][1] = -sin_WZ * cos_WY;
	A[2][1] = sin_WZ * cos_WY;
	A[2][2] = sin_WZ * sin_WY * sin_WX + cos_WZ * cos_WX;
	//A[2][3] = -sin_WZ * sin_WY * cos_WX + cos_WZ * sin_WX;
	A[2][3] = sin_WZ * sin_WY * cos_WX - cos_WZ * sin_WX;
	A[3][1] = -sin_WY;
	//A[3][2] = -cos_WY * sin_WX;
	A[3][2] = cos_WY * sin_WX;
	A[3][3] = cos_WY * cos_WX;

	for (int i = 0; i < 12; i++) {
		PTX[i] = PTX0[i];
		PTY[i] = PTY0[i];
		PTZ[i] = PTZ0[i];
	}

	for (int i = 0; i < 6; i++) {
		PTX[i] = A[1][1] * PTX0[i] + A[1][2] * PTY0[i] + A[1][3] * PTZ0[i] + DX_OUT;
		PTY[i] = A[2][1] * PTX0[i] + A[2][2] * PTY0[i] + A[2][3] * PTZ0[i] + DY_OUT;
		PTZ[i] = A[3][1] * PTX0[i] + A[3][2] * PTY0[i] + A[3][3] * PTZ0[i] + DZ_OUT;

	}
	for (int i = 0; i < 6; i++) {
		L1[i] = sqrt((PTX[i + 6] - PTX[i])*(PTX[i + 6] - PTX[i]) + (PTY[i + 6] - PTY[i])*(PTY[i + 6] - PTY[i]) + (PTZ[i + 6] - PTZ[i])*(PTZ[i + 6] - PTZ[i]));
		if (L1[i] > LL_max) L1[i] = LL_max;
		if (L1[i] < LL_min) L1[i] = LL_min;
	}

	//s1_mm = L1[0] - LL_mid;
	//s2_mm = L1[1] - LL_mid;
	//s3_mm = L1[2] - LL_mid;
	//s4_mm = L1[3] - LL_mid;
	//s5_mm = L1[4] - LL_mid;
	//s6_mm = L1[5] - LL_mid;

	ctlOutput[0] = L1[0] - LL_mid + DLT_max;
	ctlOutput[1] = L1[1] - LL_mid + DLT_max;
	ctlOutput[2] = L1[2] - LL_mid + DLT_max;
	ctlOutput[3] = L1[3] - LL_mid + DLT_max;
	ctlOutput[4] = L1[4] - LL_mid + DLT_max;
	ctlOutput[5] = L1[5] - LL_mid + DLT_max;

	//printf("[MotionCtl] output: %d,%d,%d,%d,%d,%d\n", ctlOutput[0], ctlOutput[1], ctlOutput[2], ctlOutput[3], ctlOutput[4], ctlOutput[5]);

	//for (int i = 0; i < 6; i++)
	//{
	//	if (ctlOutput[i] >= TICK_TOPMOST_POINT*0.95)
	//		ctlOutput[i] = TICK_TOPMOST_POINT*0.95;
	//	if (ctlOutput[i] <= TICK_LOWSET_POINT)//最大行程为10000,为保证不冲击缸,设定满行程的95%为最大值
	//		ctlOutput[i] = TICK_LOWSET_POINT;
	//}

}

//找回到中位
bool SixDofAlgorithm::tickMiddlePosition()
{
	if (reachMidle)
		return true;
	//根据各自位置或加或减 为了回到中位
	//printf("[CurrentPos] output: %d,%d,%d,%d,%d,%d\n", currentPos[0], currentPos[1], currentPos[2], currentPos[3], currentPos[4], currentPos[5]);
	for (int j = 0; j<6; j++){
		if ((currentPos[j] - TICK_MIDDLE_POINT<0)){			
			//正常点加速
			if (currentPos[j]>SLOW_FAST_SHIFT&&(currentPos[j] <(TICK_MIDDLE_POINT - SLOW_FAST_SHIFT)))
			{
				ctlOutput[j] = currentPos[j] + FAST_UPDOWN;
			}else//起终点减速			
				ctlOutput[j] = currentPos[j] + SLOW_UPDOWN;	
		}
		else if ((currentPos[j] - TICK_MIDDLE_POINT>0)){
			//正常点加速
			if ((currentPos[j]>(TICK_MIDDLE_POINT + SLOW_FAST_SHIFT))&&(currentPos[j] <(TICK_TOPMOST_POINT - SLOW_FAST_SHIFT)))
			{
				ctlOutput[j] = currentPos[j] - FAST_UPDOWN;
			}
			else//起终点减速
				ctlOutput[j] = currentPos[j] - SLOW_UPDOWN;				
		}
		else{
			ctlOutput[j] = TICK_MIDDLE_POINT;
		}
	}
	for (int i = 0; i<6; i++){
		if (ctlOutput[i] >= TICK_TOPMOST_POINT*0.95)
			ctlOutput[i] = TICK_TOPMOST_POINT*0.95;
		if (ctlOutput[i]<=TICK_LOWSET_POINT)
			ctlOutput[i] = TICK_LOWSET_POINT;
	}

	//printf("[MotionMiddle] output: %d,%d,%d,%d,%d,%d\n", ctlOutput[0], ctlOutput[1], ctlOutput[2], ctlOutput[3], ctlOutput[4], ctlOutput[5]);
	return false;
}
//缓慢上升一直到最高点
bool SixDofAlgorithm::tickSlowUpTopMost(void)
{
	if (isTopMost)
		return true;
	//找到最高位置点
	short int maxCyn = ctlOutput[0];
	for (int j = 1; j<6; j++){
		if (ctlOutput[j]>maxCyn)
			maxCyn = ctlOutput[j];
	}
	if (maxCyn>TICK_TOPMOST_POINT) 
		maxCyn = TICK_TOPMOST_POINT;
	//以最高位置为基准，先回平，再上升
	for (int j = 0; j<6; j++){
		if (ctlOutput[j] - maxCyn<REACH_ERROR){
			if (abs(currentPos[j] - maxCyn)>SLOW_FAST_SHIFT)
				ctlOutput[j] = currentPos[j] + FAST_UPDOWN;
			else
				ctlOutput[j] = currentPos[j] + SLOW_UPDOWN;
		}else if (ctlOutput[j] - maxCyn >= REACH_ERROR){
			ctlOutput[j] = maxCyn;
		}
	}
	//判断是否都已经回平
	if (abs(currentPos[0] - maxCyn)>REACH_ERROR && abs(currentPos[1] - maxCyn)>REACH_ERROR&&
		abs(currentPos[2] - maxCyn)>REACH_ERROR &&abs(currentPos[3] - maxCyn)>REACH_ERROR&&
		abs(currentPos[4] - maxCyn)>REACH_ERROR &&abs(currentPos[5] - maxCyn)>REACH_ERROR)
	{
		//TODO ：根据目标大小，分段快慢推进，到达零位
		for (int j = 0; j<6; j++){
			if ((currentPos[j] - TICK_LOWSET_POINT<REACH_ERROR)){
				if (abs(currentPos[j] - TICK_LOWSET_POINT)>SLOW_FAST_SHIFT)
					ctlOutput[j] = currentPos[j] + FAST_UPDOWN;
				else
					ctlOutput[j] = currentPos[j] + SLOW_UPDOWN;
			}
			else if ((currentPos[j] - TICK_LOWSET_POINT>REACH_ERROR)){
				if (abs(currentPos[j] - TICK_LOWSET_POINT)>SLOW_FAST_SHIFT)
					ctlOutput[j] = currentPos[j] - FAST_UPDOWN;
				else
					ctlOutput[j] = currentPos[j] - SLOW_UPDOWN;
			}
			else{
				ctlOutput[j] = TICK_LOWSET_POINT;
			}
		}
	}
	for (int i = 0; i < 6; i++){
		if (ctlOutput[i]<TICK_LOWSET_POINT)
			ctlOutput[i] = TICK_LOWSET_POINT;
		if (ctlOutput[i]>TICK_TOPMOST_POINT)
			ctlOutput[i] = TICK_TOPMOST_POINT;
	}
	printf("[SlowUpToTopMost] output: %d,%d,%d,%d,%d,%d\n",ctlOutput[0],ctlOutput[1],ctlOutput[2],ctlOutput[3],ctlOutput[4],ctlOutput[5]);

	return false;
}
//缓慢下降一直到最低点（平台先水平位再缓慢下降）
bool SixDofAlgorithm::tickSlowDown(void)
{
	if (isLowest)
		return true;
	//找到最低位置点
	static short int minCyn = currentPos[0];
	for (int j = 1; j<6; j++){
		if (currentPos[j]<minCyn)
			minCyn = currentPos[j];
	}
	if (minCyn<0) minCyn = 0;
	//以最低位置为基准，先回平，再下降
	for (int j = 0; j<6; j++){
		if ((currentPos[j] - minCyn>0)){
			if (abs(currentPos[j] - minCyn)>SLOW_FAST_SHIFT)
				ctlOutput[j] = currentPos[j] - FAST_UPDOWN;
			else
				ctlOutput[j] = currentPos[j] - SLOW_UPDOWN;

			if (ctlOutput[j] - minCyn <= 0){
				ctlOutput[j] = minCyn;
			}
		 }
	}
	//判断是否已经回平
	if (abs(currentPos[0] - minCyn)<REACH_ERROR && abs(currentPos[1] - minCyn)<REACH_ERROR &&
		abs(currentPos[2] - minCyn)<REACH_ERROR && abs(currentPos[3] - minCyn)<REACH_ERROR &&
		abs(currentPos[4] - minCyn)<REACH_ERROR && abs(currentPos[5] - minCyn)<REACH_ERROR)
	{
		//TODO ：根据目标大小，分段快慢推进，到达零位
		for (int j = 0; j<6; j++){
			if ((currentPos[j] - TICK_LOWSET_POINT<0)){
				if (TICK_LOWSET_POINT - currentPos[j]>SLOW_FAST_SHIFT){
					ctlOutput[j] = currentPos[j] + FAST_UPDOWN;
				}else{
					ctlOutput[j] = currentPos[j] + SLOW_UPDOWN;
				}
			}
			else if ((currentPos[j] - TICK_LOWSET_POINT>0)){
				//正常点加速
				if (currentPos[j]>SLOW_FAST_SHIFT && (currentPos[j] <(TICK_TOPMOST_POINT - SLOW_FAST_SHIFT)))
				{
					ctlOutput[j] = currentPos[j] - FAST_UPDOWN;
				}
				else//起终点减速			
					ctlOutput[j] = currentPos[j] - SLOW_UPDOWN;
			}
			else{
				ctlOutput[j] = TICK_LOWSET_POINT;
			}
		}
	}
	for (int i = 0; i<6; i++){
		if (ctlOutput[i]>TICK_TOPMOST_POINT*0.95)
			ctlOutput[i] = TICK_TOPMOST_POINT*0.95;
		if (ctlOutput[i]<TICK_LOWSET_POINT)
			ctlOutput[i] = TICK_LOWSET_POINT;
	}

	printf("[tickSlowDown] output: %d,%d,%d,%d,%d,%d\n", ctlOutput[0], ctlOutput[1], ctlOutput[2], ctlOutput[3], ctlOutput[4], ctlOutput[5]);
	return false;
}

//判断是否在最下位
bool SixDofAlgorithm::isPlatformLowest(void)
{
	int dest[6];
	for (int i = 0; i<6; i++) { dest[i] = TICK_LOWSET_POINT; }

	if (abs(currentPos[0] - dest[0]) <=  REACH_ERROR
		&& abs(currentPos[1] - dest[1]) <=  REACH_ERROR
		&& abs(currentPos[2] - dest[2]) <=  REACH_ERROR
		&& abs(currentPos[3] - dest[3]) <=  REACH_ERROR
		&& abs(currentPos[4] - dest[4]) <=  REACH_ERROR
		&& abs(currentPos[5] - dest[5]) <=  REACH_ERROR
		)
	{
		return true;	
	}
	else
	{
		return false;
	}
		
}
//判断是否在中位
bool SixDofAlgorithm::isPaltformMiddle(void)
{
	if (abs(currentPos[0] - TICK_MIDDLE_POINT) <=  REACH_ERROR
		&& abs(currentPos[1] - TICK_MIDDLE_POINT) <=  REACH_ERROR
		&& abs(currentPos[2] - TICK_MIDDLE_POINT) <=  REACH_ERROR
		&& abs(currentPos[3] - TICK_MIDDLE_POINT) <=  REACH_ERROR
		&& abs(currentPos[4] - TICK_MIDDLE_POINT) <=  REACH_ERROR
		&& abs(currentPos[5] - TICK_MIDDLE_POINT) <=  REACH_ERROR
		)
	{
	
		return true;
	}
	else
	{
		return false;
	}
}
//判断是否在最上位
bool SixDofAlgorithm::isPaltformTopMost(void)
{
	int dest[6];
	for (int i = 0; i<6; i++) { dest[i] = TICK_TOPMOST_POINT; }

	if (abs(currentPos[0] - dest[0]) <= 2 * REACH_ERROR
		&& abs(currentPos[1] - dest[1]) <= 2 * REACH_ERROR
		&& abs(currentPos[2] - dest[2]) <= 2 * REACH_ERROR
		&& abs(currentPos[3] - dest[3]) <= 2 * REACH_ERROR
		&& abs(currentPos[4] - dest[4]) <= 2 * REACH_ERROR
		&& abs(currentPos[5] - dest[5]) <= 2 * REACH_ERROR
		)
	{
		return true;
	}
	else
	{	
		return false;
	}
	
}
//判断位置
void SixDofAlgorithm::isReachSpecialPointPosition()
{
	isLowest = isPlatformLowest();
	reachMidle = isPaltformMiddle();
	isTopMost = isPaltformTopMost();
}

//更新6自由度数据
void SixDofAlgorithm::updateDofData()
{
	for (int i = 0; i < 6; i++)
	{
		//m_DOF_PlatformData.currentPos[i] = currentPos[i];
		m_DOF_PlatformData.ctlOutput[i] = ctlOutput[i];
	}

	//m_DOF_PlatformData.input_0[0] = output_dx[0];
	//m_DOF_PlatformData.input_0[1] = output_dy[0];
	//m_DOF_PlatformData.input_0[2] = output_dz[0];
	//m_DOF_PlatformData.input_0[3] = output_wx[0];
	//m_DOF_PlatformData.input_0[4] = output_wy[0];
	//m_DOF_PlatformData.input_0[5] = output_wz[0];
	

	//printf("/**********/%f\n", m_DOF_PlatformData.input_0[2]);
	//if (reachMidle){
	//	m_DOF_PlatformData.currentPoint = S_MIDDLE;
	//}
	//else if (isTopMost){
	//	m_DOF_PlatformData.currentPoint = S_TOPMOST;
	//}
	//else if (isLowest){
	//	m_DOF_PlatformData.currentPoint = S_LOWEST;
	//}else{
	//	m_DOF_PlatformData.currentPoint = S_NOTSP;
	//}
}

//将实际位置转换为算法中的位置
int SixDofAlgorithm::relationalTransformationIn(float realPos)
{
	int algorthmPos = (short int)(realPos * TICK_TOPMOST_POINT / (2 * DLT_max) + TICK_MIDDLE_POINT);
	return algorthmPos;
}
//将输出位置转换为实际位置
float SixDofAlgorithm:: relationalTransformationOut(int controlOut)
{
	float realPos = float(controlOut - TICK_MIDDLE_POINT) * 2 * DLT_max / TICK_TOPMOST_POINT;
	return realPos;
}

void SixDofAlgorithm::fkt(double y[], double f[], double input, double input_d, double epsin, double omiga)
{
	f[0] = y[1];
	f[1] = (-2 * epsin*omiga*y[1] - omiga*omiga*y[0]) + (2 * epsin*omiga*input_d + omiga*omiga*input);
	//printf("fkt:%f,%f\n", f[0] * 180 / 3.1415926, f[1] * 180 / 3.1415926);
}

                     //rgkt(2,                        dt,     &output_dx,                                  work1_dx,        work2_dx,      work3_dx,      input_dx,    input_d_dx,       1.0,     2 * ominga[3] * 3.1415926);//output[0]是位移，output[1]是速度
void SixDofAlgorithm::rgkt(int n/*方程个数*/, double step, double(*y)[2]/*存放应变量初值，返回结果*/, double work1[], double work2[], double work3[], double input, double input_d, double epsin, double omiga)
{
	double temp[5];
	int i, j;
	temp[0] = 0.5*step;
	temp[1] = temp[0];
	temp[2] = temp[0];
	temp[3] = step;
	temp[4] = step;
	for (i = 0; i<n; i++)
	{
		work1[i] = (*y)[i];
		work3[i] = (*y)[i];
	}

	for (i = 0; i<4; i++)
	{
		fkt(work1, work2, input, input_d, epsin, omiga);
		for (j = 0; j<n; j++)
		{
			work1[j] = work3[j] + temp[i] * work2[j];
			(*y)[j] = (*y)[j] + temp[i + 1] * work2[j] / 3;
		}
	}
	//printf("%f,%f\n", (*y)[0] * 180 / 3.1415926, (*y)[1] * 180 / 3.1415926);
}


//导引回零位
bool SixDofAlgorithm::leadZeroPosition()
{
	isLowest = false;
	if (abs(realCurrentPos[0] - TICK_LOWSET_POINT) <= REACH_ERROR
		&& abs(realCurrentPos[1] - TICK_LOWSET_POINT) <= REACH_ERROR
		&& abs(realCurrentPos[2] - TICK_LOWSET_POINT) <= REACH_ERROR
		&& abs(realCurrentPos[3] - TICK_LOWSET_POINT) <= REACH_ERROR
		&& abs(realCurrentPos[4] - TICK_LOWSET_POINT) <= REACH_ERROR
		&& abs(realCurrentPos[5] - TICK_LOWSET_POINT) <= REACH_ERROR
		)
	{
		isLowest = true;
		reachMidle = false;
		isTopMost = false;
		return true;
	}
	else{
		for (int j = 0; j<6; j++){
			if ((realCurrentPos[j] - TICK_LOWSET_POINT<REACH_ERROR)){

				realctlOutput[j] = realCurrentPos[j] + SLOW_UPDOWN;
			}
			else if ((realCurrentPos[j] - TICK_LOWSET_POINT>REACH_ERROR)){
				realctlOutput[j] = realCurrentPos[j] - SLOW_UPDOWN;
			}
			else{
				realctlOutput[j] = TICK_LOWSET_POINT;
			}
		}
		isLowest = false;
		return false;
	}
}