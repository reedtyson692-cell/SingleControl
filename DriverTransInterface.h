/******************************************************/
/**************functionAPI by Lirui 2017 /9.6********* /
/******************************************************/
#ifndef DRIVERTRANSINTERFACE_H
#define DRIVERTRANSINTERFACE_H
#include  <map>
#include  <string>
#include  <iostream>
#include "TYPE_DEF.H"
#include "EtherCAT_DLL.h"
#include "EtherCAT_DLL_Err.h"
#include "EtherCatMotionDriver.h"

using namespace std;
typedef enum  { PP = 0, PV = 1, PT = 2, PID = 3 ,UN_KNOW=4}ModeType;
#define MaxAxis        13
#define BALL_SCREW            10   //导程 mm
#define ENCODE_RESOLUTION 1280000 //编码器的分辨率17~Bit
#define BELT_REDUCTION_RATIO  1   //带传动减速比2：1
#define CYLINDERSTROKE 260        //电动缸的行程 单位mm
#define MAX_CYLINDERSTROKE  260    //最大行程位置
#define MIN_CYLINDERSTROKE  0    //最小行程位置
extern LPCWSTR ETHERCATMOTIONDRIVER_API stringToLPCWSTR(std::string orig);
typedef struct
{
	float  Kp;
	float  Ki;
	float  Kd;
}Pidratio;

typedef enum
{
	NO_Curve,
	SinusoidalCurve,
	CosineCurve,
	SeaWaveCurve,
	RandomCurve
}CurveMode;

typedef struct
{
	I32 targetPositon;
	U32 maxVel;
	U32 Tacc;
	U32 Tdec;
}PPConfig;

typedef struct
{
	I32 targetVelocity;
	U32 Tacc;
	U32 Tdec;
}PVConfig;

typedef struct
{
	U32 Slope;
	I16 Torque;
	I16 Torque_Profile;
}PTConfig;

typedef struct
{
	float targetStroke;
	U32 Tacc;
	U32 Tdec;
	Pidratio mRatio;
	CurveMode mCurveMode;
}PIDConfig;

typedef struct
{
	PPConfig ppConfig;
	PVConfig pvConfig;
	PTConfig ptConfig;
	PIDConfig pidConfig;
}ModeParmConfig;

typedef struct
{
	bool slaveInitFlag;
	I32 currentCmd;
	I32 currentPos;
	I32 currentVelicity;
	I16 currentTeque;
	U16 MCDone;
	U16 StstusWord;
	I32 currentMotionMode;
}MotionStatus;

typedef struct
{
	string slaveName;
	U16     slaveNum;
	ModeType currentMode;
	ModeParmConfig config;
}MotorInfo;


class ETHERCATMOTIONDRIVER_API DriverTransInterface : public CEtherCatMotionDriver
{
public:
	DriverTransInterface() {};
	DriverTransInterface(U16 masterId, U16 slaveId, CEtherCatMotionDriver *parent);
	~DriverTransInterface();
	void ResetCard();                                                             //当前轴卡重置
	void StopMotion();                                                            //停止运动
	void Svon(bool ok);                                                           //伺服启动
	void Ralm();                                                                  //消除警告
	void Move(ModeType type);                                                     //开始运动
	void emergencyStopMotion();                                                   //紧急停止
	void updateStatus();                                                          //更新状态
	float curveMotionTargetValue();                                               //目标曲线运动
	MotionStatus getCurrentMotionStatus();                                        //获取运动状态
	U16 getCurrentMasterID() { return MasterCardID; };                            //设置当前主设备id
	inline void setCurrentMasterID(U16 id){ SlaveCardID = id; };                  //获取当前主设备id
	inline U16 getCurrentSlaveCardID() { return SlaveCardID; };                   //获取当前设备id
	inline U16 getCurrentGSlotID(){ return gSlotID; };                            //获取当前节点id
	inline string getCurrentSlaveCardName(){ return SlaveCardName; };             //获取当前驱动器名称
	void setCurrentpidControlValue(float pidValue){ pidControlValue = pidValue; };//设置PID目标值
	float getCurrentpidControlValue() { return  pidControlValue; };               //获取当前PID值   
	ModeType getCurrentMoveType(){ return mCurrentModeType; };                    //获取当前运动模式
	float getCurrentElectricCylinderPos() {return ElectricCylinderPos; };         //返回电动缸的位置
	float getCurrenElectricCylinderSpeed() { return ElectricCylinderSpeed; };     //获取电动缸的速度
	bool getCurrentSvonStatus(){ return SvonEnable; };                            //获取当前伺服状态
	void setCurrentInfoValue(ModeParmConfig config);                              //设置当前配置信息
	ModeParmConfig getCurrentConfigParameter(){	return mModeParmConfig;};         //返回当前配置信息
private:
	MotionStatus currentStatus;                                                    //当前状态
	U16  MasterCardID;                                                             //主站id
	U16  SlaveCardID;                                                              //子站id
	string  SlaveCardName;                                                         //子站设备型号
	U16  gSlotID;                                                                  //节点 默认0
	float pidControlValue;                                                         //PID设定值
	ModeType mCurrentModeType;                                                     //运动模式
	ModeParmConfig mModeParmConfig;                                                //参数配置
	float ElectricCylinderPos;                                                     //电动缸位置
	float ElectricCylinderSpeed;                                                   //电动缸速度
	bool SvonEnable;                                                               //伺服使能标志位
};
#endif