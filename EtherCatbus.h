/*
//FunctionAPI by Lirui 2017/12.19 MotorDriver
*/
#ifndef ETHERCATBUS_H
#define ETHERCATBUS_H
#include  "DriverTransInterface.h"
typedef enum {
	Init=1,
	Pre_OP=2,
	Safe_OP=4,
	OP =8
}MasterStatus;

typedef enum{
	Unknow,
	On_line,
	Off_line,
}ConnectSlaveStatus;

typedef struct{
	bool isConnectOk;
	string runStatus;
	float  electricCylinderPos;
	int    electricCylinderSpeed;
	bool   svonEnable;
}MotionMonitor;

typedef struct
{
	int slaveNum;
	int platformPostion;
	string m_currentDCTime;
	string m_DeviceErrors;
	string motionTimer;
	MasterStatus m_currentMasterStatus;
	ConnectSlaveStatus m_currentSlaveStatus;
}WinCurrentStatus;

typedef map<int, MotionMonitor> MotionMonitorMap;

class ETHERCATMOTIONDRIVER_API EtherCatBus : public CEtherCatMotionDriver
{
public:
	EtherCatBus(CEtherCatMotionDriver *parent = nullptr);
	~EtherCatBus();
	static EtherCatBus *mEtherCatBus;
    static EtherCatBus *getEtherCatBusInstance(){
		if (mEtherCatBus!=NULL){
			return mEtherCatBus;
		}	
	};
	void InitialCard();                                                                               //初始化主站设备
	void FindSlave();                                                                                 //寻找子站
	void ExitCard();                                                                                  //退出轴卡
	void InitEtherCatErrorList();                                                                     //初始化错误列表
	void checkMasterConnectStatus();                                                                  //检查连接状态
	void recordNormalMotionTime();                                                                    //记录开始运行时间
	I32 readDeltaServoSlaveParameter(U8 SlotNo, U16 Page, U16 Index);                                 //读取寄存器参数
	void writeDeltaServoSlaveParameter(U8 NodeID, U16 Page, U16 Index, I32 WriteData);                //写入寄存器参数
	void loadDriverErrorTable();                                                                      //加载错误列表
	void loadInitSingleAxisConfigSetting(ModeParmConfig config);                                      //加载单轴调试默认配置         
	map <int, string> mFindAllProductInfo;                                                            //设备列表
	DriverTransInterface *mDriverTransInterface[MaxAxis];                                             //设备接口
	map <int, string> driverErrorTable;                                                               //驱动器列表
	MotionMonitorMap m_MotionMonitorMap;                                                             
	WinCurrentStatus mWinCurrentStatus;                                                                //运动状态
	U8 getMasterInitFlag(){ return InitialFlag; };                                                     //获取初始化标志位
	U8 getCheckStatus(){ return CheckStatusOk; };                                                      //获取连接状态
	U16 getConnectSlaveNum(){ return slaveNum; }                                                       //获取子站设备个数
	int getSpecialPointPosition(){	return specialPointPosition;}                                      //获取平台特殊点位置
	ModeParmConfig initDriverSettingConfig();                                                          //初始化默认配置
//signals:
//	void showFindProtectName(QMap< int, QString >);
//	void isShowWaitWidget(bool);
//	void taskProgressBarShowInfo(QString);
//	void updateCurrentWinStatus(WinCurrentStatus status, MotionMonitorMap);
private:
	U16 gESCExistCards;                             //主设备
	U16 gESCCardNo;                                 //主卡标号
	U16 gpESCCardNoList[ESCtMaxCardNum];
	U16 gNodeID, gSlotID;                           // 节点，轴号
	ModeParmConfig initConfig;                      // 配置结构体
	U8 InitialFlag;                                 // 初始化标志位
	U8 CheckStatusOk;                               // 检查连接状态   
	U16 slaveNum;                                   // 子站设备
	int specialPointPosition;                       // 特殊点位
};

#endif // ETHERCATBUS_H