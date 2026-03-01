#include "stdafx.h"
#include "EtherCatbus.h"
EtherCatBus *EtherCatBus::mEtherCatBus = NULL;
EtherCatBus::EtherCatBus(CEtherCatMotionDriver *parent):
CEtherCatMotionDriver(parent),
	gESCExistCards(0),
	gESCCardNo(0),
	gNodeID(0),
	gSlotID(0),
	CheckStatusOk(false),
	specialPointPosition(0),
	InitialFlag(0)
{
		gpESCCardNoList[ESCtMaxCardNum] = { 0 };
		loadDriverErrorTable();
		mWinCurrentStatus.motionTimer = "运行时长：00:00:00";
		loadInitSingleAxisConfigSetting(initDriverSettingConfig());
		mEtherCatBus = this;
}

EtherCatBus::~EtherCatBus()
{
	ExitCard();
}
//初始化错误列表
void EtherCatBus::InitEtherCatErrorList()
{
	//TODO
}

ModeParmConfig EtherCatBus::initDriverSettingConfig()
{
	ModeParmConfig mParmConfig;
	//PP
	mParmConfig.ppConfig.targetPositon = 10000;
	mParmConfig.ppConfig.maxVel = 10000;
	mParmConfig.ppConfig.Tacc = 200;
	mParmConfig.ppConfig.Tdec = 200;
	//PV
	mParmConfig.pvConfig.targetVelocity = 50;
	mParmConfig.pvConfig.Tacc = 1000;
	mParmConfig.pvConfig.Tdec = 1000;
	//PT
	mParmConfig.ptConfig.Slope = 20;
	mParmConfig.ptConfig.Torque = 20;
	mParmConfig.ptConfig.Torque_Profile = 0;
	//PID
	mParmConfig.pidConfig.targetStroke = (double)(MAX_CYLINDERSTROKE + MIN_CYLINDERSTROKE) / 2;
	mParmConfig.pidConfig.Tacc = 1000;
	mParmConfig.pidConfig.Tdec = 1000;
	mParmConfig.pidConfig.mRatio.Kp =0;
	mParmConfig.pidConfig.mRatio.Ki = 0;
	mParmConfig.pidConfig.mRatio.Kd = 0;
	return mParmConfig;
}
//装载单轴调试的配置参数
void EtherCatBus::loadInitSingleAxisConfigSetting(ModeParmConfig config)
{
		initConfig = config;
}
//初始化运动控制卡
void EtherCatBus::InitialCard()
{
	I16 rt;
	U16 i, CardNo;
	/*gESCExistCards 变量会被填入EtherCAT轴卡数量*/
	rt = _ECAT_Master_Open(&gESCExistCards);
	if (rt){
		char infoStr[100];
		sprintf(infoStr, "_ECAT_Master_Open, rt = %d",rt);
		MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
		//MessageBox(NULL, stringToLPCWSTR("I'm here!"), stringToLPCWSTR("INFO"), MB_OK);
	}
	else{
		if (gESCExistCards == 0){
			MessageBox(NULL, stringToLPCWSTR("No EtherCat can be found!"), stringToLPCWSTR("INFO"), MB_OK);
			InitialFlag = 0;
		}
	}
	for (i = 0; i<ESCtMaxCardNum; i++){
		gpESCCardNoList[i] = 0;
	}

	for (i = 0; i<gESCExistCards; i++)
	{
		rt = _ECAT_Master_Get_CardSeq(i, &CardNo);
		rt = _ECAT_Master_Initial(CardNo);
		if (rt != 0){
			char infoStr[100];	
			sprintf(infoStr, "_ECAT_Master_Initial, rt=%d", rt);
			MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
			InitialFlag = 0;
		}
		else{
			gpESCCardNoList[i] = CardNo;
			InitialFlag = 1;
			//TODO:
		}
	}
	if (InitialFlag){
		gESCCardNo = gpESCCardNoList[0];
	}	
}
//寻找从站
void EtherCatBus::FindSlave()
{
	I16 rt;
	U16 i, j, SlaveNum = 0, Cnt = 0, ReMapID;
	U32 VendorID, ProductCode, RevisionNo, DCTime;
	string productName;
	mFindAllProductInfo.clear();
	rt = _ECAT_Master_Get_SlaveNum(gESCCardNo, &SlaveNum);
	slaveNum = SlaveNum;
	if (rt){
		char infoStr[100];
		sprintf(infoStr, "_ECAT_Master_Get_SlaveNum, rt=%d", rt);
		MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
	}
	else
	{
		if (SlaveNum == 0){
			char infoStr[100];
			sprintf(infoStr, "Card NO: %d% No Slave Found!", gESCCardNo);
			MessageBox(NULL, stringToLPCWSTR(infoStr), stringToLPCWSTR("INFO"), MB_OK);
		}
		else{	
			for (i = 0; i < SlaveNum; i++){
				rt = _ECAT_Master_Get_Slave_Info(gESCCardNo, i, &ReMapID, &VendorID, &ProductCode, &RevisionNo, &DCTime);

				if (VendorID == 0x1DD && ProductCode == 0x10305070) //A2E
				{
					j = 0;
					productName = "A2-E";
					mFindAllProductInfo.insert(map<int, string>::value_type (i, productName));
					Cnt++;
				}
				else if (VendorID == 0x539 && ProductCode == 0x2200001) //Yaskawa
				{
					j = 0;
					productName = "Yaskawa";
					mFindAllProductInfo.insert(map<int, string>::value_type(i, productName));
					Cnt++;
				}
				else if ((VendorID == 0x1A05 || VendorID == 0x01DD) && ProductCode == 0x0624) //Ec4Axis
				{
					productName = "Ec4Axis";
					for (j = 0; j < 4; j++){
						//Todo:
						Cnt++;
					}
					mFindAllProductInfo.insert(map<int, string>::value_type(i, productName));
				}
				else if ((VendorID == 0x1A05 || VendorID == 0x01DD) && ProductCode == 0x5621) //EcAxis
				{
					j = 0;
					productName = "EcAxis";
					mFindAllProductInfo.insert(map<int, string>::value_type(i, productName));
					Cnt++;
				}
				//根据子站的个数来设置控制接口数量
				mDriverTransInterface[i] = new DriverTransInterface(gESCCardNo, i, this);
				mDriverTransInterface[i]->setCurrentInfoValue(initConfig);
				mWinCurrentStatus.slaveNum = slaveNum;
			}
		}
	}
}
//退出轴卡
void EtherCatBus::ExitCard()
{
	for (int i = 0; i<gESCExistCards; i++){
		_ECAT_Master_Reset(gpESCCardNoList[i]);//将界面卡重置
	}
	_ECAT_Master_Close();//终止轴卡运行
}
//计算运行时长
void EtherCatBus::recordNormalMotionTime()
{
	string tickTimerStr;
	int hour, min, sec;
	string HourStr, MinStr, SecStr;
	static U32 timerCount=0;
	timerCount++;
	sec = timerCount * 20 / 1000;
	hour = sec / 3600;
	min = (sec % 3600) / 60;
	sec = (sec % 3600) % 60;
	char s[12];  //设定12位对于存储32位int值足够  
	_itoa(hour, s, 10);
	string string_hour = s;
	if (hour < 10)
		HourStr = "0" + string_hour;
	else
		HourStr = string_hour;

	_itoa(min, s, 10);
	string string_min = s;
	if (min < 10)
		MinStr = "0" + string_min;
	else
		MinStr = string_min;

	_itoa(sec, s, 10);
	string string_sec = s;
	if (sec < 10)
		SecStr = "0" + string_sec;
	else
		SecStr = string_sec;
	tickTimerStr = "运行时长：" + HourStr + ":" + MinStr + ":" + SecStr;
	mWinCurrentStatus.motionTimer = tickTimerStr;
}
//检查主站连接状态
void EtherCatBus::checkMasterConnectStatus()
{
	I16 rt;
	U32 State;
	I32 DCTime = 0;
	I32 OffsetTime = 0;
	U16 InitialDone;

	//TODO:显示信息
	rt = _ECAT_Master_Check_Initial_Done(gESCCardNo, &InitialDone);
	if (rt == 0){
		//初始化完毕
		if (InitialDone == 0){
			//emit isShowWaitWidget(false);
			mWinCurrentStatus.slaveNum = slaveNum;
			CheckStatusOk = true;
		}
		else if (InitialDone == 1){
			CheckStatusOk = false;
		}
		else if (InitialDone == 99){
			CheckStatusOk = false;
		}
	}
	rt = _ECAT_Master_Get_DC_Status(gESCCardNo, &State, &DCTime, &OffsetTime);
	if (rt == 0){
		char s[12];  //设定12位对于存储32位int值足够  
		_itoa(DCTime, s, 10);
		string string_DcTime = s;
		string mDCTime = "DCTime: " + string_DcTime;
		if (State == 1){
			mWinCurrentStatus.m_currentDCTime = mDCTime;
		}else{
			//表示错误
			mDCTime = "0";
			mWinCurrentStatus.m_currentDCTime = mDCTime;
		}
	}
	U16 Status;
	U16 m_MasterStatus = 0;
	U16  Abnormal_Flag = false;
	Status = _ECAT_Master_Get_Connect_Status(gESCCardNo, &m_MasterStatus);
	mWinCurrentStatus.m_currentMasterStatus = (MasterStatus)m_MasterStatus;
	U16 c, Working_Slave_Cnt;
	//检查主从站连接状态
	Status = _ECAT_Master_Check_Working_Counter(gESCCardNo, &Abnormal_Flag, &Working_Slave_Cnt);
	if (Abnormal_Flag == 0 && CheckStatusOk){
		mWinCurrentStatus.m_currentSlaveStatus = On_line;
	}
	else{
		mWinCurrentStatus.m_currentSlaveStatus = Off_line;
	}
	mWinCurrentStatus.platformPostion = specialPointPosition;
	//emit updateCurrentWinStatus(mWinCurrentStatus);
	m_MotionMonitorMap.clear();
	for (int i = 0; i < slaveNum; i++){
		U16 Page = 0,  Index = 1;//P0-1
		MotionMonitor m_MotionMonitor;
		string currentError = driverErrorTable[readDeltaServoSlaveParameter(i, Page, Index)];
		m_MotionMonitor.runStatus = currentError;//P0-01
		m_MotionMonitor.svonEnable = mDriverTransInterface[i]->getCurrentSvonStatus();
		m_MotionMonitor.isConnectOk = Abnormal_Flag;
		m_MotionMonitor.electricCylinderPos = mDriverTransInterface[i]->getCurrentElectricCylinderPos();
		m_MotionMonitor.electricCylinderSpeed = mDriverTransInterface[i]->getCurrenElectricCylinderSpeed();
		m_MotionMonitorMap.insert(MotionMonitorMap::value_type(i, m_MotionMonitor));
	}
	//emit updateCurrentWinStatus(mWinCurrentStatus, m_MotionMonitorMap);
}
//读取驱动器寄存器参数 AxisNo=NodeID編號
//P0-01 驱动器目前警报代码显示（七段显示器）
//P0-08 伺服启动时间
//P1-42 电磁刹车开启延迟时间
//P1-54
I32 EtherCatBus::readDeltaServoSlaveParameter(U8 NodeID, U16 Page, U16 Index)
{
	U16 Status = 0;
	I32 ReadData = 0;
	Status = _ECAT_Slave_DeltaServo_Read_Parameter(gESCCardNo, NodeID, 0, Page, Index, &ReadData);
	if (Status){
		//AppHelper::ShowMessageBoxInfo(QString("读取参数失败！"));
	}else{

	}
	return ReadData;
}
//写入驱动器参数
void EtherCatBus::writeDeltaServoSlaveParameter(U8 NodeID, U16 Page, U16 Index, I32 WriteData)
{
	U16 Status = 0;
	Status = _ECAT_Slave_DeltaServo_Write_Parameter(gESCCardNo, NodeID, 0, Page, Index, WriteData);
	if (Status){
		MessageBox(NULL, stringToLPCWSTR("Write FAil"), stringToLPCWSTR("INFO"), MB_OK);	
	}
	else{
		MessageBox(NULL, stringToLPCWSTR("Write Success"), stringToLPCWSTR("INFO"), MB_OK);
	}
}
//加载驱动器警告对照表
void EtherCatBus::loadDriverErrorTable()
{
	//QFile errorFile("./ConfigFile/DeriveError.txt");
	//if (!errorFile.open(QIODevice::ReadWrite | QIODevice::Text)) {
	//	QLOG_ERROR() << "Can't open the file!" << endl;
	//}
	//QTextStream in(&errorFile);
	//QString allString = in.readAll();
	//QStringList lineList = allString.split("\n");
	//QString currentLine;
	//
	//if (!allString.isEmpty())
	//{
	//	foreach(currentLine, lineList)
	//	{
	//		QString currentRow;
	//		QStringList rowList = currentLine.split("：");//是中文的:
	//		driverErrorTable.insert(rowList.at(0).toInt(), rowList.at(1));
	//	}
	//	QLOG_INFO() << "Load Driver Message Success";
	//}else{
	//	QLOG_ERROR() << "This File Has Been Destoryed";
	//}
	//驱动器异警一览表
}


