#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#pragma once
struct PIDRatio
{
	float  Kp;
	float  Ki;
	float  Kd;
};

class PIDController
{
public:
	PIDController();
	~PIDController();
	float IncrementalPIDControlMethod(float current_Velocity, float target_Velocity);
	float single_Velocity_Control_PID(float current_Velocity, float target_Velocity, PIDRatio m_PidRatio);
	float down_MultiAxis_Velocity_Control_PID(float current_Velocity, float target_Velocity, PIDRatio m_PidRatio, int currentIndex);
	float up_MultiAxis_Velocity_Control_PID(float current_Velocity, float target_Velocity, PIDRatio m_PidRatio, int currentIndex);
};

#endif