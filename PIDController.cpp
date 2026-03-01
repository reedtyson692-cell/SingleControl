#include "PIDController.h"

#define VL_MAX 12500  //15000~250mm/s 此值由电动缸限定速度计算
//19800~330mm/s 此值由电动缸限定速度计算

PIDController::PIDController()
{


}

PIDController::~PIDController()
{

}

//单轴调试pid
float PIDController::single_Velocity_Control_PID(float current_Velocity, float target_Velocity, PIDRatio m_PidRatio)
{
	float p_p = m_PidRatio.Kp;// 0.98;//30;5
	float p_i = m_PidRatio.Ki;//30;
	float p_d = m_PidRatio.Kd;// 0.1;//15

	static float error[3] = { 0, 0, 0 };
	static float output = 0;
	static float integral = 0;

	error[0] = error[1];
	error[1] = error[2];
	error[2] = target_Velocity - current_Velocity;

	integral += p_i*error[2];

	if (integral>5000)
		integral = 5000;
	if (integral<-5000)
		integral = -5000;

	output = error[2] * p_p + p_d* (error[2] - error[1]) + integral;

	if (output >= VL_MAX){
		output = VL_MAX;
	}
	else if (output <= (-VL_MAX)){
		output = -VL_MAX;
	}

	return output;
}

//多轴运动pid
float PIDController::down_MultiAxis_Velocity_Control_PID(float current_Velocity, float target_Velocity, PIDRatio m_PidRatio, int currentIndex)
{
	float p_p = m_PidRatio.Kp;// 0.98;//30;5
	float p_i = m_PidRatio.Ki;//30;
	float p_d = m_PidRatio.Kd;// 0.1;//15

	static float error[6][3] = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0 };

	static float output[6] = { 0, 0, 0, 0, 0, 0 };
	static float integral[6] = { 0, 0, 0, 0, 0, 0 };

	error[currentIndex][0] = error[currentIndex][1];
	error[currentIndex][1] = error[currentIndex][2];
	error[currentIndex][2] = target_Velocity - current_Velocity;

	integral[currentIndex] += p_i*error[currentIndex][2];

	if (integral[currentIndex]>5000)
		integral[currentIndex] = 5000;
	if (integral[currentIndex]<-5000)
		integral[currentIndex] = -5000;

	output[currentIndex] = error[currentIndex][2] * p_p + p_d* (error[currentIndex][2] - error[currentIndex][1]) + integral[currentIndex];

	if (output[currentIndex] >= VL_MAX){
		output[currentIndex] = VL_MAX;
	}
	else if (output[currentIndex] <= (-VL_MAX)){
		output[currentIndex] = -VL_MAX;
	}
	return output[currentIndex];
}

float PIDController::up_MultiAxis_Velocity_Control_PID(float current_Velocity, float target_Velocity, PIDRatio m_PidRatio, int currentIndex)
{
	float p_p = m_PidRatio.Kp;// 0.98;//30;5
	float p_i = m_PidRatio.Ki;//30;
	float p_d = m_PidRatio.Kd;// 0.1;//15

	static float error[6][3] = {
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0,
		0, 0, 0 };

	static float output[6] = { 0, 0, 0, 0, 0, 0 };
	static float integral[6] = { 0, 0, 0, 0, 0, 0 };

	error[currentIndex][0] = error[currentIndex][1];
	error[currentIndex][1] = error[currentIndex][2];
	error[currentIndex][2] = target_Velocity - current_Velocity;

	integral[currentIndex] += p_i*error[currentIndex][2];

	if (integral[currentIndex]>5000)
		integral[currentIndex] = 5000;
	if (integral[currentIndex]<-5000)
		integral[currentIndex] = -5000;

	output[currentIndex] = error[currentIndex][2] * p_p + p_d* (error[currentIndex][2] - error[currentIndex][1]) + integral[currentIndex];

	if (output[currentIndex] >= VL_MAX){
		output[currentIndex] = VL_MAX;
	}
	else if (output[currentIndex] <= (-VL_MAX)){
		output[currentIndex] = -VL_MAX;
	}
	return output[currentIndex];
}