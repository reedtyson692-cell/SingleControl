#include "stdafx.h"
#include "DriverTransInterface.h"
#pragma execution_character_set("utf-8")

LPCWSTR ETHERCATMOTIONDRIVER_API stringToLPCWSTR(std::string orig)
{
	size_t origsize = orig.length() + 1;
	const size_t newsize = 100;
	size_t convertedChars = 0;
	wchar_t *wcstring = (wchar_t *)malloc(sizeof(wchar_t)*(orig.length() - 1));
	mbstowcs_s(&convertedChars, wcstring, origsize, orig.c_str(), _TRUNCATE);
	return wcstring;
}
DriverTransInterface::DriverTransInterface(U16 masterId, U16 slaveId, CEtherCatMotionDriver *parent)
: CEtherCatMotionDriver(parent), gSlotID(0), SvonEnable(false)
{
	MasterCardID = masterId;
	SlaveCardID = slaveId;
	char s[12];  
	_itoa(slaveId, s, 10);
	string string_slaveId = s;
	SlaveCardName = "A2-E-" + string_slaveId;
}

DriverTransInterface::~DriverTransInterface()
{

}

//重置卡
void DriverTransInterface::ResetCard()
{
	I16 rt;
	rt = _ECAT_Slave_Motion_Set_Position(MasterCardID, SlaveCardID, gSlotID, 0);
	rt = _ECAT_Slave_Motion_Set_Command(MasterCardID, SlaveCardID, gSlotID, 0);
}
//停止
void DriverTransInterface::StopMotion()
{
	I16 rt;
	char infoStr[100];
	switch (mCurrentModeType)
	{
	case PP:
		rt = _ECAT_Slave_Motion_Sd_Stop(MasterCardID, SlaveCardID, gSlotID, mModeParmConfig.ppConfig.Tdec);
		if (rt){
			sprintf(infoStr, "_ECAT_Slave_Motion_Sd_Stop, rt=%d", rt);
			MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);		
		}
		break;
	case PV:
		rt = _ECAT_Slave_Motion_Sd_Stop(MasterCardID, SlaveCardID, gSlotID, mModeParmConfig.pvConfig.Tdec);
		if (rt){
			sprintf(infoStr, "_ECAT_Slave_Motion_Sd_Stop, rt = %d",  rt);
			MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
		}
		break;
	case PT:
		rt = _ECAT_Slave_Motion_Emg_Stop(MasterCardID, SlaveCardID, gSlotID);
		if (rt){
			sprintf(infoStr, "_ECAT_Slave_Motion_Emg_Stop=%d",  rt);
			MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
		}
	case PID:
		rt = _ECAT_Slave_Motion_Sd_Stop(MasterCardID, SlaveCardID, gSlotID, mModeParmConfig.pidConfig.Tdec);
		if (rt){
			sprintf(infoStr, "_ECAT_Slave_Motion_Sd_Stop =%d",  rt);
			MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
		}
		break;
	default:
		break;
	}
}
//伺服启动
void DriverTransInterface::Svon(bool ok)
{
	I16 rt;
	char* infoStr = NULL;
	rt = _ECAT_Slave_Motion_Set_Svon(MasterCardID, SlaveCardID, gSlotID, ok);
	if (ok){
		SvonEnable = true;
		//QLOG_INFO() << "Motor ID:" << SlaveCardID << "Has Servo ON";
	}else{
		SvonEnable = false;
		//QLOG_INFO() << "Motor ID:" << SlaveCardID << "Has Servo OFF";
	}
	if (rt){
		sprintf(infoStr, "_ECAT_Slave_Motion_Set_Svon, rt=%d", rt);
		MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
		//QLOG_ERROR() << "Motor ID:" << SlaveCardID << xx; 
	}
}
//清除Alarm
void DriverTransInterface::Ralm()
{
	I16 rt;
	char* infoStr = NULL;
	rt = _ECAT_Slave_Motion_Ralm(MasterCardID, SlaveCardID, gSlotID);
	if (rt){
		sprintf(infoStr, "_ECAT_Slave_Motion_Ralm, rt =%d", rt);
		MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
	}
}
//伺服运动  该函数会被用到定时器中断线程不能创建widget
void DriverTransInterface::Move(ModeType type)
{
	I16 rt;
	I32 Dist;
	U32 MaxVel, Tacc, Tdec;
	char* infoStr = NULL;
	mCurrentModeType = type;
	switch (mCurrentModeType)
	{
		case PP:
		{
				   rt = _ECAT_Slave_PP_Start_Move(MasterCardID, SlaveCardID, gSlotID,
					   mModeParmConfig.ppConfig.targetPositon, mModeParmConfig.ppConfig.maxVel,
					   mModeParmConfig.ppConfig.Tacc, mModeParmConfig.ppConfig.Tdec, 1);
				   if (rt){
					 /*  xx = QString("_ECAT_Slave_PP_Start_Move, rt = %1").arg(rt);
					   QLOG_ERROR() << "Motor ID:" << SlaveCardID << xx;*/
				   }
				   break;
		}
		case PV:
		{
				   rt = _ECAT_Slave_PV_Start_Move(MasterCardID, SlaveCardID, gSlotID,
					   mModeParmConfig.pvConfig.targetVelocity, mModeParmConfig.pvConfig.Tacc,
					   mModeParmConfig.pvConfig.Tdec);
				   if (rt){
					/*   xx = QString("_ECAT_Slave_PV_Start_Move, rt = %1").arg(rt);
					   QLOG_ERROR() << "Motor ID:" << SlaveCardID << xx;*/
				   }
				   break;
		}
		case PT:
		{
				   I16 Torque = mModeParmConfig.ptConfig.Torque;
				   U32 Slope = mModeParmConfig.ptConfig.Slope;
				   I16 Torque_Profile = mModeParmConfig.ptConfig.Torque_Profile;
				   U16 SetBit = 0x02, Max_Current = 0;
				   Slope = (Slope > 1000) ? 1000 : Slope;
				   Torque = (Torque > 1000) ? 1000 : Torque;
				   rt = _ECAT_Slave_PT_Advance_Config(MasterCardID, SlaveCardID, gSlotID, SetBit, Max_Current, Torque_Profile);
				   if (rt){
					 /*  xx = QString("_ECAT_Slave_PT_Advance_Config, rt = %1").arg(rt);
					   QLOG_ERROR() << "Motor ID:" << SlaveCardID << xx;*/
				   }
				   rt = _ECAT_Slave_PT_Start_Move(MasterCardID, SlaveCardID, gSlotID, Torque, Slope);
				   if (rt){
					   /*xx = QString("_ECAT_Slave_Torque_Start_Move, rt = %1").arg(rt);
					   QLOG_ERROR() << "Motor ID:" << SlaveCardID << xx;*/
				   }
				   break;
		}
		case PID: //
		{
					rt = _ECAT_Slave_PV_Start_Move(MasterCardID, SlaveCardID, gSlotID,
						pidControlValue, mModeParmConfig.pidConfig.Tacc,
						mModeParmConfig.pidConfig.Tdec);
					if (rt){
						/*xx = QString("_ECAT_Slave_PV_Start_Move, rt = %1").arg(rt);
						QLOG_ERROR() << "Motor ID:" << SlaveCardID << xx;*/
					}
					break;
		}
		default:
			break;
	}
}

MotionStatus DriverTransInterface::getCurrentMotionStatus()
{
	return currentStatus;
}
//1ms更新一次的
void DriverTransInterface::updateStatus()
{
	I16 rt;
	I32 Pos, Spd;
	I16 Torque = 0;
	I32 Cmd;
	U16 MCDone, Ststus;
	//获取运动信息位置和速度
	rt = _ECAT_Slave_Motion_Get_Position(MasterCardID, SlaveCardID, gSlotID, &Pos);
	rt = _ECAT_Slave_Motion_Get_Current_Speed(MasterCardID, SlaveCardID, gSlotID, &Spd);
	rt = _ECAT_Slave_Motion_Get_Command(MasterCardID, SlaveCardID, gSlotID, &Cmd);
	//获取状态
	rt = _ECAT_Slave_Motion_Get_StatusWord(MasterCardID, SlaveCardID, gSlotID, &Ststus);
	rt = _ECAT_Slave_Motion_Get_Mdone(MasterCardID, SlaveCardID, gSlotID, &MCDone);
	currentStatus.currentCmd = Cmd;
	//电机编码位置
	currentStatus.currentPos = Pos;
	currentStatus.currentTeque = Torque;
	currentStatus.currentVelicity = Spd;
	currentStatus.MCDone = MCDone;
	currentStatus.StstusWord = Ststus;
	currentStatus.currentMotionMode = mCurrentModeType;
	//电动缸绝对位置
	ElectricCylinderPos = (float)Pos / ENCODE_RESOLUTION;//圈数
	ElectricCylinderPos = (ElectricCylinderPos / BELT_REDUCTION_RATIO) * BALL_SCREW;//单位mm 减速比为2：1 导程为4
	ElectricCylinderSpeed = Spd*0.1*BALL_SCREW/60 ;//mm/s
	MessageBox(NULL, stringToLPCWSTR("Reference to successful!"), stringToLPCWSTR("INFO"), MB_OK);
	//cout << "调用成功！";
}

void DriverTransInterface::setCurrentInfoValue(ModeParmConfig config)
{
	mModeParmConfig = config;
}

//运动变换
float DriverTransInterface::curveMotionTargetValue()
{
	static float tt = 0;
	CurveMode curve = mModeParmConfig.pidConfig.mCurveMode;
	float pidTargerCurveValue;
	switch (curve)
	{
	case NO_Curve:
		pidTargerCurveValue = mModeParmConfig.pidConfig.targetStroke;
		break;
	case SinusoidalCurve:{
			double  dz = 0.0;
			double amp = 55;
			double frq =0.1;
			dz = amp*sin(3.1415926*2.0*frq*tt);
			pidTargerCurveValue = dz + amp;
		}
		break;
	case CosineCurve:
		pidTargerCurveValue = mModeParmConfig.pidConfig.targetStroke;
		break;
	case SeaWaveCurve:
		pidTargerCurveValue = mModeParmConfig.pidConfig.targetStroke;
		break;
	case RandomCurve:
		pidTargerCurveValue = mModeParmConfig.pidConfig.targetStroke;
		break;
	default:
		break;
	}
	tt = tt + 0.01;
	if (tt >=10)
		tt = 0.0;
	return pidTargerCurveValue;
}
//紧急停止
void DriverTransInterface::emergencyStopMotion()
{
	U16 Status;
	Status=_ECAT_Slave_Motion_Emg_Stop(MasterCardID, SlaveCardID, gSlotID);
}