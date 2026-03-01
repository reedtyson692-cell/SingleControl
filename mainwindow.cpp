#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "6Dof.h"

#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>
//#include <QRandomGenerator>
#include <QtCharts/QSplineSeries>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace std;
float chartTime = 0;

ofstream out("C://Users//Administrator//Desktop//out.txt");

#pragma execution_character_set("utf-8")
#define deviceDescription L"PCI-1751,BID#0"
EtherCatBus* MainWindow::mEtherCatBus = NULL;
TimerPulseCtrl* MainWindow::timerPulseCtrl = NULL;
MotionCal* MainWindow::motioncal = NULL;
DataLogger* MainWindow::dataLogger = NULL;
DeviceManager* MainWindow::manager = NULL;
//CSerialPort* MainWindow::mySerialPort = NULL;
static int i = 0;
static float nowpos = 0;//推杆伸长量
static float nowPos[12];
static float nowvel = 0;//推杆速度
static float targetpos = 0;
static float PIDOutput = 0;
static float SampttCounter = 0;
//static int SendControlCmdCounter = 0;//轮询控制
static float WX,WY,WZ,DZ = 0;
static float k = 0;
static int up_mid_check = 0;
static int down_mid_check = 0;
static int up_zero_check = 0;
static int down_zero_check = 0;
static float roll = 0;
static float pitch = 0;
static float yaw = 0;
static double Nowpos[6];
static bool up_mid = false;
static bool down_mid = false;
static bool up_zero = false;
static bool down_zero = false;
static bool up_trans = false;
static bool down_trans = false;
static bool timer = false;//是否开始计时器标志符
static int p = 0;

//WS601Struct INS550C;
ForceSensorData forceData;
IMUData imuData[2];
SixDofAlgorithm mSixDofAlgorithm[2];


// ============ 【修改1：新增以下全局数组，方便 UI 线程读取】 ============
static double current_pos[6] = { 0 };       // 下平台电缸长度数据
static double current_pos1[6] = { 0 };      // 上平台电缸长度数据
static double current_thrust_N[6] = { 0 };  // 下平台电缸推力数据
static double current_thrust_N1[6] = { 0 }; // 上平台电缸推力数据
// =========================================================================

U16 SlaveNum;
I32 Positions;
short raw_torque;
I32 Speed;
ControlIntermediateQuantity m_ControlIntermediateQuantity;
//U16 No;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);// 初始化UI组件
	ui->start->setCheckable(true);// 设置“伺服启动”开关按钮为可选中状态
	mEtherCatBus = new EtherCatBus;
	Velocity = 0;
	motioncal = new MotionCal;

	m_pTimer = new QTimer(this);
	m_pChart = new MyChart;
	m_pChart->setTitle("这是一条曲线");
	//隐藏图例
	m_pChart->legend()->hide();
	m_pChart->setTheme(QChart::ChartThemeLight);//设置系统主题
	QChartView* pView = new QChartView(m_pChart);
	ui->showChart->addWidget(pView);
	pView->setRenderHint(QPainter::Antialiasing);
	
	// 连接信号槽
	connect(m_pTimer, &QTimer::timeout, this, &MainWindow::onUpdateData);
	connect(ui->btnShow, &QPushButton::clicked, this, &MainWindow::onStart);
	connect(ui->btnStop, &QPushButton::clicked, this, &MainWindow::onStop);

	//SixDofAlgorithm mSixDofAlgorithm[2];
	double dx, dy, dz, wx, wy, wz = 0.0;
	double ampDz = 0.1;
	double ampWx = 0.1;
	double ampWy = 0.1;

	double frqDz = 0.1;
	double frqWx = 0.1;
	double frqWy = 0.1;

	double i = 0;

	PlatformInitParameter DonwPlatformInit;
	DonwPlatformInit.m_Index = 0;
	DonwPlatformInit.m_Amp[0] = 150;//ampDX
	DonwPlatformInit.m_Amp[1] = 150;//ampDY 
	DonwPlatformInit.m_Amp[2] = 100;//ampDZ
	DonwPlatformInit.m_Amp[3] = 8; //ampWX 
	DonwPlatformInit.m_Amp[4] = 4; //ampWY 
	DonwPlatformInit.m_Amp[5] = 8; //ampWZ 
	DonwPlatformInit.m_SampT = 0.02;//
	for (int i = 0; i < 6; i++) {
		DonwPlatformInit.m_Frq[i] = 0.1;
		DonwPlatformInit.m_Ominga[i] = 0.99;
	}
	DonwPlatformInit.m_CtlMode = Wave_Mode;//z方向正弦
	DonwPlatformInit.m_E = 0;
	DonwPlatformInit.m_H = 999.7;
	DonwPlatformInit.m_RF = 2200;
	DonwPlatformInit.m_RG = 2400;
	DonwPlatformInit.m_GA = 15;
	DonwPlatformInit.m_BE = 20;
	DonwPlatformInit.m_DLT_Max = 230;
	DonwPlatformInit.m_LL_Mid = 1304.977;
	for (int i = 0; i < 3; i++) {
		DonwPlatformInit.offset[i] = 0;
	}
	//DonwPlatformInit.offset[2] = 999.7;
	DonwPlatformInit.offset[2] = 0;

	PlatformInitParameter UpPlatformInit;
	UpPlatformInit.m_Index = 1;
	UpPlatformInit.m_Amp[0] = 150;//ampDX x平移
	UpPlatformInit.m_Amp[1] = 150;//ampDY x平移
	UpPlatformInit.m_Amp[2] = 100;//ampDZ x平移
	UpPlatformInit.m_Amp[3] = 8; //ampWX x平移
	UpPlatformInit.m_Amp[4] = 4; //ampWY x平移
	UpPlatformInit.m_Amp[5] = 8; //ampWZ x平移
	UpPlatformInit.m_SampT = 0.02;//
	for (int i = 0; i < 6; i++) {
		UpPlatformInit.m_Frq[i] = 0.1;
		UpPlatformInit.m_Ominga[i] = 0.99;
	}
	UpPlatformInit.m_CtlMode = Compensate_Mode;//z方向正弦
	UpPlatformInit.m_E = 0;
	UpPlatformInit.m_H = 915.19;//因为增加了88mm的高度导致杆长增加，顶部平台到中间平台的高度由815.95变成915.19
	UpPlatformInit.m_RF = 1200;
	UpPlatformInit.m_RG = 1300;
	UpPlatformInit.m_GA = 18;
	UpPlatformInit.m_BE = 18;
	UpPlatformInit.m_DLT_Max = 130;
	UpPlatformInit.m_LL_Mid = 932.0+88;//黄东龙改加力传感器长度88mm
	for (int i = 0; i < 3; i++) {
		UpPlatformInit.offset[i] = 0;
	}

	mSixDofAlgorithm[0].initPlatformParameter(UpPlatformInit);
	mSixDofAlgorithm[1].initPlatformParameter(DonwPlatformInit);

	mSixDofAlgorithm[0].calInitParam();
	mSixDofAlgorithm[1].calInitParam();
/**********************************************************************/
	//motioncal->InitParam();
	//mySerialPort = new CSerialPort;
	dataLogger = new DataLogger;
	manager = new DeviceManager;
	if (!dataLogger->Initialize(2)) {
		std::cerr << "数据记录器初始化失败，程序将继续运行但不存储数据" << std::endl;
	}
/************************************************************************/
	/*更新UI状态的代码，更新当前位置框图内的数据*/
	myTimer = new QTimer(this);
	connect(myTimer, &QTimer::timeout, [=](){
		switch (m_ControlIntermediateQuantity.mMotorMode){
		case Run_No_Mode:{
							 /**/
							 ui->state->setText("当前位置： " + QString::number(nowpos, 'f', 5) + "\n"
								 + "当前速度： " + QString::number(nowvel, 'f', 5) + "\n"
								 + "PID输出： " + QString::number(PIDOutput, 'f', 5) + "\n"
								 + "橫摇： " + QString::number(roll, 'f', 2) + "\n"
								 + "纵摇： " + QString::number(pitch, 'f', 2) + "\n"
								 + "艏摇： " + QString::number(yaw, 'f', 2) + " ");
							 
							 break;
		}
		//case All_Zero:{
		//				  ui->state->setText(QString::number(Nowpos[0], 'f', 5) + " "
		//					  + QString::number(Nowpos[1], 'f', 5) + " "
		//					  + QString::number(Nowpos[2], 'f', 5) + "\n" + " "
		//					  + QString::number(Nowpos[3], 'f', 5) + " "
		//					  + QString::number(Nowpos[4], 'f', 5) + " "
		//					  + QString::number(Nowpos[5], 'f', 5));
		//				  break;
		//}
		case All_Trans:{
						   //ui->state->setText(QString::number(roll, 'f', 5) + " "
							  // + QString::number(pitch, 'f', 5) + " "
							  // + QString::number(yaw, 'f', 5) + " ");
						   //ui->state->setText(QString::number(motioncal->UpPlatform->Lenth[0], 'f', 5) + " "
							  // + QString::number(motioncal->UpPlatform->Lenth[1], 'f', 5) + " "
							  // + QString::number(motioncal->UpPlatform->Lenth[2], 'f', 5) + " "
							  // + QString::number(motioncal->UpPlatform->Lenth[3], 'f', 5) + " "
							  // + QString::number(motioncal->UpPlatform->Lenth[4], 'f', 5) + " "
							  // + QString::number(motioncal->UpPlatform->Lenth[5], 'f', 5) + " "
							  // + "橫摇： " + QString::number(roll, 'f', 2) + "\n"
							  // + "纵摇： " + QString::number(pitch, 'f', 2) + "\n"
							  // + "艏摇： " + QString::number(yaw, 'f', 2) + " ");
						  //  ui->state->setText(QString::number(motioncal->DownPlatform->Lenth[0], 'f', 5) + " "
						  //   + QString::number(motioncal->DownPlatform->Lenth[1], 'f', 5) + " "
						  //   + QString::number(motioncal->DownPlatform->Lenth[2], 'f', 5) + " "
						  //   + QString::number(motioncal->DownPlatform->Lenth[3], 'f', 5) + " "
						  //   + QString::number(motioncal->DownPlatform->Lenth[4], 'f', 5) + " "
						  //   + QString::number(motioncal->DownPlatform->Lenth[5], 'f', 5) + "\n"
							 //+ QString::number(motioncal->UpPlatform->Lenth[0], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[1], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[2], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[3], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[4], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[5], 'f', 5) + " "
							 //+ "橫摇： " + QString::number(roll, 'f', 2) + "\n"
							 //+ "纵摇： " + QString::number(pitch, 'f', 2) + "\n"
							 //+ "艏摇： " + QString::number(yaw, 'f', 2) + " ");
							ui->state->setText(QString::number(mSixDofAlgorithm[0].ctlOutput[0], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[0].ctlOutput[1], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[0].ctlOutput[2], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[0].ctlOutput[3], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[0].ctlOutput[4], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[0].ctlOutput[5], 'f', 5) + "\n"
								+ QString::number(mSixDofAlgorithm[1].ctlOutput[0], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[1].ctlOutput[1], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[1].ctlOutput[2], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[1].ctlOutput[3], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[1].ctlOutput[4], 'f', 5) + " "
								+ QString::number(mSixDofAlgorithm[1].ctlOutput[5], 'f', 5) + "\n"
								+ "橫摇： " + QString::number(roll, 'f', 2) + "\n"
								+ "纵摇： " + QString::number(pitch, 'f', 2) + "\n"
								+ "艏摇： " + QString::number(yaw, 'f', 2) + " ");
						   //if (up_trans){
							   //ui->state->setText(QString::number(motioncal->UpPlatform->Lenth[0], 'f', 5) + " "
								  // + QString::number(motioncal->UpPlatform->Lenth[1], 'f', 5) + " "
								  // + QString::number(motioncal->UpPlatform->Lenth[2], 'f', 5) + " "
								  // + QString::number(motioncal->UpPlatform->Lenth[3], 'f', 5) + " "
								  // + QString::number(motioncal->UpPlatform->Lenth[4], 'f', 5) + " "
								  // + QString::number(motioncal->UpPlatform->Lenth[5], 'f', 5) + " ");
						   //}
						   //if (down_trans){
							  // ui->state->setText(QString::number(motioncal->DownPlatform->Lenth[0], 'f', 5) + " "
								 //  + QString::number(motioncal->DownPlatform->Lenth[1], 'f', 5) + " "
								 //  + QString::number(motioncal->DownPlatform->Lenth[2], 'f', 5) + " "
								 //  + QString::number(motioncal->DownPlatform->Lenth[3], 'f', 5) + " "
								 //  + QString::number(motioncal->DownPlatform->Lenth[4], 'f', 5) + " "
								 //  + QString::number(motioncal->DownPlatform->Lenth[5], 'f', 5) + " ");
						   //}
						   break;
		}
		case Motion_Test:{
							 //  ui->state->setText(QString::number(motioncal->DownPlatform->Lenth[0], 'f', 5) + " "
							 //   + QString::number(motioncal->DownPlatform->Lenth[1], 'f', 5) + " "
							 //   + QString::number(motioncal->DownPlatform->Lenth[2], 'f', 5) + " "
							 //   + QString::number(motioncal->DownPlatform->Lenth[3], 'f', 5) + " "
							 //   + QString::number(motioncal->DownPlatform->Lenth[4], 'f', 5) + " "
							 //   + QString::number(motioncal->DownPlatform->Lenth[5], 'f', 5) + "\n"
							 //+ QString::number(motioncal->UpPlatform->Lenth[0], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[1], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[2], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[3], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[4], 'f', 5) + " "
							 //+ QString::number(motioncal->UpPlatform->Lenth[5], 'f', 5) + " "
							 //+ "橫摇： " + QString::number(roll, 'f', 2) + "\n"
							 //+ "纵摇： " + QString::number(pitch, 'f', 2) + "\n"
							 //+ "艏摇： " + QString::number(yaw, 'f', 2) + " ");
							 ui->state->setText(QString::number(mSixDofAlgorithm[0].ctlOutput[0], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[0].ctlOutput[1], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[0].ctlOutput[2], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[0].ctlOutput[3], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[0].ctlOutput[4], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[0].ctlOutput[5], 'f', 5) + "\n"
								 + QString::number(mSixDofAlgorithm[1].ctlOutput[0], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[1].ctlOutput[1], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[1].ctlOutput[2], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[1].ctlOutput[3], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[1].ctlOutput[4], 'f', 5) + " "
								 + QString::number(mSixDofAlgorithm[1].ctlOutput[5], 'f', 5) + "\n"
								 + "橫摇： " + QString::number(roll, 'f', 2) + "\n"
								 + "纵摇： " + QString::number(pitch, 'f', 2) + "\n"
								 + "艏摇： " + QString::number(yaw, 'f', 2) + " ");
							  //ui->state->setText(QString::number(mSixDofAlgorithm[1].WX_OUT));
							 break;
		}
		default:{
					ui->state->setText(QString::number(nowpos, 'f', 5) + " "
						+ "DOWN_MID_CHECK = "
						+ QString::number(down_mid_check) + " "
						+ "UP_MID_CHECK = "
						+ QString::number(up_mid_check) + " ");
						//+ QString::number(yaw, 'f', 5) + " ");
					break;

		}
			break;
		}
		//ui->state->setText(QString::number(nowpos, 'f', 5) + " " + QString::number(targetpos, 'f', 5) + " "+QString::number(PIDOutput, 'f', 5) + " ");
		//ui->state->setText(QString::number(nowpos, 'f', 5) + " " + QString::number(nowvel, 'f', 5) + " " + QString::number(PIDOutput, 'f', 5) + " ");
		//ui->state->setText(QString::number(roll, 'f', 5) + " " + QString::number(pitch, 'f', 5) + " " + QString::number(yaw, 'f', 5) + "\n");
		//ui->state->setText(QString::number(nowpos, 'f', 5) + " " + QString::number(nowvel, 'f', 5) + " " + QString::number(PIDOutput, 'f', 5) + " "
		//	+ QString::number(motioncal->Lenth[0], 'f', 5) + " "
		//	+ QString::number(motioncal->Lenth[1], 'f', 5) + " " 
		//	+ QString::number(motioncal->Lenth[2], 'f', 5) + " "
		//	+ QString::number(motioncal->Lenth[3], 'f', 5) + " "
		//	+ QString::number(motioncal->Lenth[4], 'f', 5) + " "
		//	+ QString::number(motioncal->Lenth[5], 'f', 5) + "\n");
		//ui->X_output->setText(QString::number(m, 'f', 5));
		//ui->Y_output->setText(QString::number(m, 'f', 5));
	});

	timerPulseCtrl = TimerPulseCtrl::Create(); //创建硬件定时器控制器实例
	timerPulseCtrl->addTimerTickHandler(OnCounterEvent, this);//注册硬件定时中断回调函数OnCounterEvent：中断处理函数的函数指针，this：传递给回调函数的上下文指针（当前MainWindow实例）
	initSystemStruct(); //初始化运动控制系统的状态和参数

	// ============ 【修改3：在构造函数末尾新增独立的存盘定时器】 ============
	QTimer* logTimer = new QTimer(this);
	connect(logTimer, &QTimer::timeout, [=]() {
		// 只有当定时器中断开启（伺服启动）时，才记录数据
		if (timerPulseCtrl && timerPulseCtrl->getEnabled()) {
			// 保存力传感器数据
			forceData = manager->GetForceSensorData();
			if (forceData.IsValid()) {
				dataLogger->LogForceData(forceData);
			}
			// 保存上平台数据
			imuData[0] = manager->GetIMUData(0);
			dataLogger->LogIMUData(0, imuData[0], current_pos1, current_thrust_N1);
			// 保存下平台数据
			imuData[1] = manager->GetIMUData(1);
			dataLogger->LogIMUData(1, imuData[1], current_pos, current_thrust_N);
		}
		});
	logTimer->start(200); // 严格设定为 200ms 执行一次
	// =========================================================================
}
//初始化系统指令
void MainWindow::initSystemStruct()
{
	//up_mid = false;
	//down_mid = false;
	//timer = false;
	m_ControlIntermediateQuantity.currentSingleAxisId = 0;
	m_ControlIntermediateQuantity.mDownNormalControlFlag = false;
	m_ControlIntermediateQuantity.mMotorMode = Run_No_Mode;
	m_ControlIntermediateQuantity.samplingTime = 0;
	m_ControlIntermediateQuantity.singleAxisControlFlag = false;
	m_ControlIntermediateQuantity.startAppendRecord = false;
	m_ControlIntermediateQuantity.mUpNormalControlFlag = false;
	motionparam.X = 0;
	motionparam.Y = 0;
	motionparam.Z = 0;
	motionparam.ROLL = 0;
	motionparam.PITCH = 0;
	motionparam.YAW = 0;
}

MainWindow::~MainWindow()
{
	delete motioncal;
	delete mEtherCatBus;
	delete dataLogger;
	delete manager;
	//delete mySerialPort;
	delete myTimer;
    delete ui;
}

void MainWindow::onStart()
{
	m_pTimer->start(100);
}
void MainWindow::onStop()
{
	m_pTimer->stop();
}
/*负责更新图表数据并实现图表的动态滚动效果*/
void MainWindow::onUpdateData()
{
	qreal x = m_pChart->plotArea().width() / m_pChart->m_pAxisx->tickCount();
	//cout << x << "u8我是平移量" << endl;

	qreal xValue = (m_pChart->m_pAxisx->max() - m_pChart->m_pAxisx->min()) / m_pChart->m_pAxisx->tickCount();
	m_pChart->m_x = chartTime*xValue/5;
	//cout << xValue << endl;
	//qreal yValue = QRandomGenerator::global()->bounded(5);
	//qreal yValue = 3 * sin(chartTime * 90 / 180 * 3.1415926) + 2 * cos(chartTime * 10 / 180 * 3.1415926) + sin((chartTime + 20) * 45 / 180 * 3.1415926);
	qreal yValue = nowPos[11];
	qreal yValue1 = mSixDofAlgorithm[0].ctlOutput[5];
	m_pChart->m_y1 = yValue;
	//if (chartTime < 3) {
	//	m_pChart->m_y2 = 0;
	//}
	//else {
	//	m_pChart->m_y2 = yValue1;
	//}
	m_pChart->m_y2 = yValue1;
	chartTime++; 
	m_pChart->m_pSeries->append(m_pChart->m_x, m_pChart->m_y1);
	m_pChart->m_pSeries1->append(m_pChart->m_x, m_pChart->m_y2);
	//cout << "x=" << m_pChart->m_x << "y=" << m_pChart->m_y1 <<endl;
	//滚动坐标轴
	cout << m_pChart->m_y1 << endl;
	if (m_pChart->m_x > m_pChart->m_pAxisx->max() - 5) {
		m_pChart->scroll(x, 0);
		cout << "m_x" << m_pChart->m_x << "max" << m_pChart->m_pAxisx->max() << endl;
	}
	//m_pChart->scroll(x, 0);
}

//系统初始化，搜索设备
void MainWindow::on_init_clicked()
{
	U16 IntialDone;
	mEtherCatBus->InitialCard();
	mEtherCatBus->FindSlave();
	initialize();                             //初始化PCI-1751定时器功能
}
//设置控制模式为回零模式
void MainWindow::on_findslave_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = Single_Zero;
}
//设置电机轴号
void MainWindow::on_setMotor_valueChanged()
{
	SlaveNO = (ui->setMotor->text()).toInt();
	m_ControlIntermediateQuantity.currentSingleAxisId = SlaveNO;
	//j = SlaveNO;
}
//设置单缸控制速度
void MainWindow::on_setVelocity_valueChanged()
{
	Velocity = (ui->setVelocity->text()).toInt();

}
//设置目标位置，该模式暂未启用
void MainWindow::on_setPosition_valueChanged()
{
	m_ControlIntermediateQuantity.TargetPOS = (ui->setPosition->text()).toInt();
	//argetpos = m_ControlIntermediateQuantity.TargetPOS;
	//ui->state->setText(QString::number(m_ControlIntermediateQuantity.TargetPOS));
}
//单轴pid控制位置
void MainWindow::on_PID_clicked()
{
	//m_ControlIntermediateQuantity.mMotorMode = Single_PID;
	m_ControlIntermediateQuantity.mMotorMode = IMU_control;
}
//伺服开关
void MainWindow::on_start_clicked(bool checked)
{
	if (checked)
	{
		for (SlaveNO = 0; SlaveNO < 6; SlaveNO++){
			_ECAT_Slave_Motion_Set_Svon(0, SlaveNO, 0, Enable);
		}
		startcount();
		ui->start->setText("关闭伺服");
	}
	else
	{
		for (SlaveNO = 0; SlaveNO < 6; SlaveNO++){
			_ECAT_Slave_Motion_Set_Svon(0, SlaveNO, 0, Unable);
		}
	//	myTimer->stop();
	//}
		stopcount();
		if (myTimer->isActive() == true)
		{
			myTimer->stop();
		}
		ui->start->setText("打开伺服");
	}
}
//运动停止
void MainWindow::on_stop_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = Run_No_Mode;
	_ECAT_Slave_Motion_Sd_Stop(0, SlaveNO, 0, 100);
	//if (myTimer->isActive() == true)
	//{
}
//运动开始
void MainWindow::on_rotate_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = Run_No_Mode;
	//startcount();
	//_ECAT_Slave_CSV_Start_Move(0, SlaveNO, 0, Velocity, 0.2, 1, 0);
	_ECAT_Slave_PV_Start_Move(0, SlaveNO, 0, Velocity, 100, 100);
	if (myTimer->isActive() == false)
	{
		myTimer->start(100);//100ms为周期更新UI
	}
}
//位置重置
void MainWindow::on_reset_clicked()
{
	_ECAT_Slave_Motion_Set_Position(0, SlaveNO, 0, 0);
	_ECAT_Slave_Motion_Set_Command(0, SlaveNO, 0, 0);
	_ECAT_Slave_Motion_Ralm(0, SlaveNO, 0);
}
void MainWindow::on_ALL_UP_clicked()
{
	for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
		//_ECAT_Slave_CSV_Start_Move(0,id, 0, 400, 0.2, 0.5, 0);
		_ECAT_Slave_PV_Start_Move(0, id, 0, -200, 100, 100);
	}
	k = 0;
	p = 0;
}

void MainWindow::on_ALL_STOP_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = Run_No_Mode;
	for (int id = DOWN_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
		_ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
	}
	k = 0;
	p = 0;
}

void MainWindow::on_ALL_ZERO_clicked()
{
	for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
		//_ECAT_Slave_CSV_Start_Move(0,id, 0, 400, 0.2, 0.5, 0);
		_ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
		float data = (float)Positions / 1280000 * 10;
		Nowpos[id] = data - 230;
	}
	m_ControlIntermediateQuantity.mMotorMode = All_Zero;
}
//PCI-1751定时器初始化函数
void MainWindow::initialize(){
	DeviceInformation devInfo(deviceDescription);
	const wchar_t* profilePath = L"PCI-1751.xml";
	ret = timerPulseCtrl->setSelectedDevice(devInfo);
	ret = timerPulseCtrl->LoadProfile(profilePath);
	ret = timerPulseCtrl->setChannelCount(1);
	ret = timerPulseCtrl->setChannelStart(1);
}
//PCI-1751定时器启动
void MainWindow::startcount(){
	//int length = 8;//定义传输的长度
	//unsigned char *temp = new unsigned char[8];//动态创建一个数组
	//mySerialPort->InitPort(2, CBR_115200, 'N', 8, 1, EV_RXCHAR);
	//mySerialPort->OpenListenThread();

	/*********************************************************************/
	// 添加力传感器设备 (COM19, 115200bps)
	std::cout << "初始化力传感器 (COM19, 115200bps)... ";
	manager->AddDevice(new ForceSensorDevice(19, CBR_115200));
	std::cout << "完成" << std::endl;
	// 添加IMU设备 (COM15, 115200bps)(COM2, 115200bps)	std::cout << "初始化IMU (COM15, 115200bps)(COM2, 115200bps)... ";
	std::cout << "初始化IMU (COM15, 115200bps)... ";
	manager->AddDevice(new IMUDevice(15, CBR_115200));
	std::cout << "初始化IMU (COM2, 115200bps)... ";
	manager->AddDevice(new IMUDevice(2, CBR_115200));
	std::cout << "完成" << std::endl;
	// 启动所有设备
	if (!manager->StartAllDevices()) {
		std::cerr << "设备启动失败，程序退出" << std::endl;
	}
	/*********************************************************************/
	ErrorCode ret = Success;
	for (int i = timerPulseCtrl->getChannelStart(); i < timerPulseCtrl->getChannelStart() + timerPulseCtrl->getChannelCount(); i++){
		ret = timerPulseCtrl->getChannels()->getItem(i).setFrequency(1000);
	}
	ret = timerPulseCtrl->setEnabled(true);
}
//PCI-1751定时器停止
void MainWindow::stopcount(){
	ErrorCode ret = Success;
	ret = timerPulseCtrl->setEnabled(false);
}
//1ms中断函数
void MainWindow::OnCounterEvent(void *sender, CntrEventArgs *args, void * userParam){
	static unsigned int updataInfoCounter = 0;//位置更新计数
	static float SendControlCmdCounter = 0;//轮询控制
	static int CheckConnectCounter = 0;  //检查状态
	static PIDController m_pidControl;
	SendControlCmdCounter++;

	// ============ 【修改2：删除此处原有的 current_pos 等4个局部数组的声明】 ============
	// ============ 【修改2：删除此处原有的 if (currentTime - lastDisplayTime > 200) {...} 整段存盘代码】 ============

	/*static DWORD lastDisplayTime = GetTickCount();*/
	//// 每200ms保存一次数据
	//DWORD currentTime = GetTickCount();
	//if (currentTime - lastDisplayTime > 200) {
	//	lastDisplayTime = currentTime;
	//	// 保存力传感器数据
	//	forceData = manager->GetForceSensorData();
	//	if (forceData.IsValid()) {
	//		dataLogger->LogForceData(forceData);
	//	}
	//	//// 保存IMU数据
	//	//for (size_t i = 1; i < 2; i++)
	//	//{
	//	//	imuData[i] = manager->GetIMUData(i);
	//	//	if (imuData[i].IsValid()) {
	//	//		dataLogger->LogIMUData(i, imuData[i], current_pos);
	//	//	}
	//	//}
	//	//上平台数据采集
	//	imuData[0] = manager->GetIMUData(0);
	//	dataLogger->LogIMUData(0, imuData[0], current_pos1, current_thrust_N1);
	//	//下平台数据采集
	//	imuData[1] = manager->GetIMUData(1);
	//	dataLogger->LogIMUData(1, imuData[1], current_pos, current_thrust_N);
	//	
	//};

	//每10ms更新一次不同电机数据
	if (updataInfoCounter >= 10){
		/*WS601Resolve(&INS550C, mySerialPort->Data);*/
		roll = round(imuData[1].angles[0] * 100) / 100 - 1.88;
		pitch = round(imuData[1].angles[1] * 100) / 100 - 0.34;
		yaw = round(imuData[1].angles[2] * 100) / 100;
		switch (m_ControlIntermediateQuantity.mMotorMode){
		case Run_No_Mode:
		{
							_ECAT_Slave_Motion_Get_Position(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, &Positions);
							float data = (float)Positions / 1280000 * 10;
							//motioncal->UpPlatform->nowpos[6] = data;
							nowpos = data;
							_ECAT_Slave_Motion_Get_Current_Speed(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, &Speed);
							nowvel = Speed;
							break;
							
		}
		case Single_Zero:
		{
							_ECAT_Slave_Motion_Get_Position(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, &Positions);
							float data = (float)Positions / 1280000 * 10;
							nowpos = data;
							_ECAT_Slave_Motion_Get_Current_Speed(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, &Speed);
							nowvel = Speed;
							if (nowpos - TICK_LOWSET_POINT <= 0.1)
							{
								//_ECAT_Slave_Motion_Sd_Stop(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, 0.1);
								_ECAT_Slave_Motion_Sd_Stop(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, 100);
							}
							else{
								if (nowpos > 0){
									if ((nowpos - TICK_LOWSET_POINT>5)){
										//_ECAT_Slave_CSV_Start_Move(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, -800, 0.2, 1, 0);
										_ECAT_Slave_PV_Start_Move(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, -800, 100, 100);
									}
									else if(nowpos - TICK_LOWSET_POINT>2){
										_ECAT_Slave_PV_Start_Move(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, -200, 100, 100);
									}
									else{
										_ECAT_Slave_PV_Start_Move(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, -50, 100, 100);
									}
								}
							}
							break;
		}
		case Single_PID:{
							//同时正弦
							//for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
							//	_ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
							//	float data = (float)Positions / 1280000 * 10;
							//	nowpos = data;
							//	double  dz = 0.0;
							//	double amp = ((double)(150 - 0)) / 2 - 20;
							//	double frq = 0.1;
							//	dz = amp*sin(3.1415926*2.0*frq*(SampttCounter));
							//	float sincurve = ((double)(260 + 0)) / 2 + dz;
							//	PIDRatio m_PidRatio = { 150, 0.0014, 0 };
							//	//float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, sincurve, m_PidRatio);
							//	float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, a[id-6], m_PidRatio);
							//	PIDOutput = a[id-6];
							//	_ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
							//}

							//单缸PID
							_ECAT_Slave_Motion_Get_Position(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, &Positions);
							float data = (float)Positions / 1280000 * 10;
							nowpos = data;
							_ECAT_Slave_Motion_Get_Current_Speed(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, &Speed);
							nowvel = Speed;
							PIDRatio m_PidRatio = { 150 , 0.0014 , 0 };
							float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, m_ControlIntermediateQuantity.TargetPOS, m_PidRatio);
							PIDOutput = outputData;
							_ECAT_Slave_PV_Start_Move(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, outputData, 100, 100);
							nowvel++;

							//单缸正弦
							//double  dz = 0.0;
							//double amp = ((double)(200 - 0)) / 2 - 20;
							//double frq = 0.1;
							//_ECAT_Slave_Motion_Get_Position(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, &Positions);
							//float data = (float)Positions / 1280000 * 10;
							//nowpos = data;
							//dz = amp*sin(3.1415926*2.0*frq*SampttCounter);
							//float sincurve = ((double)(260 + 0)) / 2 + dz;
							//PIDRatio m_PidRatio = { 150, 0.0014, 0 };
							//float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, sincurve, m_PidRatio);
							//PIDOutput = sincurve;
							//_ECAT_Slave_PV_Start_Move(0, m_ControlIntermediateQuantity.currentSingleAxisId, 0, outputData, 100, 100);
							SampttCounter = SampttCounter + 0.005;
							if (SampttCounter >= 40)
								SampttCounter = 0.0;
							break;

		}
		case All_Zero:{
						  if (up_zero){
							  for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
								  _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
								  float data = (float)Positions / 1280000 * 10;
								  //motioncal->UpPlatform->nowpos[id - 6] = data;
								  nowpos = data;
								  if (nowpos - TICK_LOWSET_POINT <= 0.1)
								  {
									  _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
									  up_zero_check++;
								  }
								  else{
									  if (nowpos > 0){
										  if ((nowpos - TICK_LOWSET_POINT>5)){
											  _ECAT_Slave_PV_Start_Move(0, id, 0, -800, 100, 100);
										  }
										  else if (nowpos - TICK_LOWSET_POINT>2){
											  _ECAT_Slave_PV_Start_Move(0, id, 0, -200, 100, 100);
										  }
										  else{
											  _ECAT_Slave_PV_Start_Move(0, id, 0, -50, 100, 100);
										  }
									  }
								  }
							  }
							  if (up_zero_check >= 240){
								  up_zero = false;
								  //up_mid_check = 1;
							  }
						  }
						  if (down_zero){
							  for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
								  _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
								  float data = (float)Positions / 1280000 * 10;
								  nowpos = data;
								  if (nowpos - TICK_LOWSET_POINT <= 0.1)
								  {
									  _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
									  down_zero_check++;
								  }
								  else{
									  if (nowpos > 0){
										  if ((nowpos - TICK_LOWSET_POINT>5)){
											  _ECAT_Slave_PV_Start_Move(0, id, 0, -800, 100, 100);
										  }
										  else if (nowpos - TICK_LOWSET_POINT>2){
											  _ECAT_Slave_PV_Start_Move(0, id, 0, -200, 100, 100);
										  }
										  else{
											  _ECAT_Slave_PV_Start_Move(0, id, 0, -50, 100, 100);
										  }
									  }
								  }
							  }
							  if (down_zero_check >= 240){
								  down_zero = false;
								  //up_mid_check = 1;
							  }
						  }
						  //全回零
						  //for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
							 // _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
							 // float data = (float)Positions / 1280000 * 10;
							 // nowpos = data;
							 // if (nowpos - TICK_LOWSET_POINT <= 0.1)
							 // {
								//  _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
							 // }
							 // else{
								//  if (nowpos > 0){
								//	  if ((nowpos - TICK_LOWSET_POINT>5)){
								//		  _ECAT_Slave_PV_Start_Move(0, id, 0, -800, 100, 100);
								//	  }
								//	  else if (nowpos - TICK_LOWSET_POINT>2){
								//		  _ECAT_Slave_PV_Start_Move(0, id, 0, -200, 100, 100);
								//	  }
								//	  else{
								//		  _ECAT_Slave_PV_Start_Move(0, id, 0, -50, 100, 100);
								//	  }
								//  }
							 // }
						  //}

						  //for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
						  // _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
						  // float data = (float)Positions / 1280000 * 10;
						  // nowpos = data;
						  // if (Nowpos[id] >= nowpos){
						  //  if (Nowpos[id] - nowpos <= 0.1)
						  //  {
						  //	  _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
						  //  }
						  //  else{
						  //	  //if (nowpos > 0){
						  //	  if (Nowpos[id] - nowpos>5){
						  //		  _ECAT_Slave_PV_Start_Move(0, id, 0, 800, 100, 100);
						  //	  }
						  //	  else if (Nowpos[id] - nowpos>2){
						  //		  _ECAT_Slave_PV_Start_Move(0, id, 0, 200, 100, 100);
						  //	  }
						  //	  else{
						  //		  _ECAT_Slave_PV_Start_Move(0, id, 0, 50, 100, 100);
						  //	  }
						  //	  //}
						  //  }
						  // }
						  // else{
						  //  if (nowpos - Nowpos[id] <= 0.1)
						  //  {
						  //	  _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
						  //  }
						  //  else{
						  //	  //if (nowpos > 0){
						  //	  if (nowpos - Nowpos[id] > 5){
						  //		  _ECAT_Slave_PV_Start_Move(0, id, 0, -800, 100, 100);
						  //	  }
						  //	  else if (nowpos - Nowpos[id] > 2){
						  //		  _ECAT_Slave_PV_Start_Move(0, id, 0, -200, 100, 100);
						  //	  }
						  //	  else{
						  //		  _ECAT_Slave_PV_Start_Move(0, id, 0, -50, 100, 100);
						  //	  }
						  //	  //}
						  //  }
						  // }
						  //}
						  break;
		}
		case All_Mid:{
						 if (up_mid){
							 for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
								 _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
								 float data = (float)Positions / 1280000 * 10;
								 nowpos = data;
								 if (UP_TICK_MIDEST_POINT >= nowpos){
									 if (UP_TICK_MIDEST_POINT - nowpos <= 0.1)
									 {
										 _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
										 up_mid_check++;
									 }
									 else{
										 //if (nowpos > 0){
										 if (UP_TICK_MIDEST_POINT - nowpos > 5){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, 800, 100, 100);
										 }
										 else if (UP_TICK_MIDEST_POINT - nowpos > 2){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, 200, 100, 100);
										 }
										 else{
											 _ECAT_Slave_PV_Start_Move(0, id, 0, 50, 100, 100);
										 }
										 //}
									 }
								 }
								 else{
									 if (nowpos - UP_TICK_MIDEST_POINT <= 0.1)
									 {
										 _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
										 up_mid_check++;
									 }
									 else{
										 //if (nowpos > 0){
										 if (nowpos - UP_TICK_MIDEST_POINT > 5){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, -800, 100, 100);
										 }
										 else if (nowpos - UP_TICK_MIDEST_POINT > 2){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, -200, 100, 100);
										 }
										 else{
											 _ECAT_Slave_PV_Start_Move(0, id, 0, -50, 100, 100);
										 }
										 //}
									 }
								 }
								 if (up_mid_check >= 240){
									 up_mid = false;
									 //up_mid_check = 1;
								 }
							 }
						 }
						 if (down_mid){
							 for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
								 _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
								 float data = (float)Positions / 1280000 * 10;
								 nowpos = data;
								 if (DOWN_TICK_MIDEST_POINT >= nowpos){
									 if (DOWN_TICK_MIDEST_POINT - nowpos <= 0.1)
									 {
										 _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
										 down_mid_check++;
									 }
									 else{
										 //if (nowpos > 0){
										 if (DOWN_TICK_MIDEST_POINT - nowpos > 5){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, 800, 100, 100);
										 }
										 else if (DOWN_TICK_MIDEST_POINT - nowpos > 2){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, 200, 100, 100);
										 }
										 else{
											 _ECAT_Slave_PV_Start_Move(0, id, 0, 50, 100, 100);
										 }
										 //}
									 }
								 }
								 else{
									 if (nowpos - DOWN_TICK_MIDEST_POINT <= 0.1)
									 {
										 _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
										 down_mid_check++;
									 }
									 else{
										 //if (nowpos > 0){
										 if (nowpos - DOWN_TICK_MIDEST_POINT > 5){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, -800, 100, 100);
										 }
										 else if (nowpos - DOWN_TICK_MIDEST_POINT > 2){
											 _ECAT_Slave_PV_Start_Move(0, id, 0, -200, 100, 100);
										 }
										 else{
											 _ECAT_Slave_PV_Start_Move(0, id, 0, -50, 100, 100);
										 }
										 //}
									 }
								 }
								 if (down_mid_check >= 240){
									 down_mid = false;
									 //up_mid_check = 1;
								 }
							 }
						 }
						 break;
		}
		case All_Trans:{
						   
						   if (down_trans){
							   
							   mSixDofAlgorithm[1].tickMotionControl();
							   for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
								   _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
								   float data = (float)Positions / 1280000 * 10;
								   nowpos = data;
								   nowPos[id] = data;
								   //PIDRatio m_PidRatio = { 150, 0.0014, 0 };
								   PIDRatio m_PidRatio = { 250, 0.0014, 0 };
								   //motioncal->Calculating(motioncal->DownPlatform);
								   float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, mSixDofAlgorithm[1].ctlOutput[id], m_PidRatio);
								   //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, motioncal->DownPlatform->Lenth[id], m_PidRatio);
								   //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, a[id - 6], m_PidRatio);
								   //PIDOutput = motioncal->DownPlatform->Lenth[id];
								   _ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
							   }
						   
						   }
						   if (up_trans){
							   
							   mSixDofAlgorithm[0].tickMotionControl();
							   for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
								   _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
								   float data = (float)Positions / 1280000 * 10;
								   nowpos = data;
								   nowPos[id] = data;
								   //PIDRatio m_PidRatio = { 150, 0.0014, 0 };
								   PIDRatio m_PidRatio = { 250, 0.0014, 0 };
								   //motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -roll, -pitch, 0);
								   //motioncal->Calculating(motioncal->UpPlatform);
								   float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, mSixDofAlgorithm[0].ctlOutput[id - 6], m_PidRatio);
								   //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, motioncal->UpPlatform->Lenth[id - 6], m_PidRatio);
								   //PIDOutput = motioncal->UpPlatform->Lenth[id];
								   _ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
							   }

						   }
						   //for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
							  // _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
							  // float data = (float)Positions / 1280000 * 10;
							  // nowpos = data;
							  // PIDRatio m_PidRatio = { 150, 0.0014, 0 };
							  // motioncal->Calculating(motioncal->DownPlatform);
							  // float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, motioncal->DownPlatform->Lenth[id], m_PidRatio);
							  // //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, a[id - 6], m_PidRatio);
							  // PIDOutput = motioncal->DownPlatform->Lenth[id];
							  // _ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
						   //}
						   break;
		}
		case Motion_Test:{
							//m = m + 0.01;
							 k = k + 0.02;
							 
							 //if (k <= 200){
								 //WX = 2 * sin(3.1415926 * 2.0 * 0.1 * k);//单位就为角度
								 //WY = -2 * sin(3.1415926 * 2.0 * 0.1 * k / 5.0);
								 //WX = 4 * sin(PI * 2.0 * 0.1 * k)+sin(PI*0.2*k/2) + 1.5*sin(0.15*k*PI*1.5);//单位就为角度
								 WX = 6 * sin(k*0.00125*PI / 10)
									 + 2 * sin(k*0.2*PI / 1)
									 + 3 * sin(k*0.4*PI / 3)
									 + 2 * sin(k*0.3*PI / 2)
									 - 2 * cos(k*0.2*PI / 3)
									 + 2;
								 //WY = 6 * sin(k*0.00125*PI / 10)
									// + 2 * sin(k*0.2*PI / 1)
									// + 3 * sin(k*0.4*PI / 3)
									// + 2 * sin(k*0.3*PI / 2)
									// - 2 * cos(k*0.2*PI / 3);
								 //WY = 4 * sin(k*0.00125*PI / 10)
									// + 1 * sin(k*0.2*PI / 1)
									// + 1.5*sin(k*0.4*PI / 3)
									// + 2 * sin(k*0.3*PI / 2)
									// - 2 * cos(k*0.2*PI / 3)
									// + 2;

								 WY = 1 * sin(k*0.5*PI / 3)
									 + 1 * sin(k*0.2*PI / 9)
									 + 1.5*sin(k*0.3*PI / 3)
									 + 1 * sin(k*0.05*PI / 9)
									 - 2 * cos(k*0.02*PI / 9)
									 - 1 * cos(k*0.002*PI / 1)
									 + 1 * sin(k*PI / 40)
									 - 0.5*sin(k*0.525*PI / 10)
									 + 3;

								 //DZ = 12 * sin(k*0.4* PI / 3)
									// + 8 * sin(k* PI / 50)
									// + 2 * sin(k* PI / 13)
									// + 2 * sin(k*2.5* PI / 6)
									// - 2 * cos(k*0.5* PI / 2.5)
									// - 8 * cos(k*0.3* PI / 15)
									// + 10 * cos(k * 1 * PI / 23);
						//		 DZ = (30 * sin(PI*2.0*0.12*k));
								 DZ = 0;
								 PIDRatio m_PidRatio = { 250, 0.0014, 0 };

								 //motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -WX, -WY, 0);
								 // 
								 motioncal->InputParam(motioncal->UpPlatform, 0, 0, -DZ, -WX, -WY, 0);
								 motioncal->NowMotion(motioncal->UpPlatform);/*存疑*/
								 // 
								 //mSixDofAlgorithm[0].setMotionParam(0,
								 //	-815.95 * sin(-WX * PI / 180 / 2) * 2 * cos(-WX * PI / 180 / 2),
								 //	815.95 * sin(-WX * PI / 180 / 2) * 2 * sin(-WX * PI / 180 / 2),
								 //	-WX* PI / 180,
								 //	0,
								 //	0);
								 mSixDofAlgorithm[0].setMotionParam(motioncal->UpPlatform->Position[0],
									 motioncal->UpPlatform->Position[1],
									 motioncal->UpPlatform->Position[2],
									 //0,
									 //0,
									 -WX* PI / 180,
									 -WY* PI / 180,
									 0);

								 //mSixDofAlgorithm[0].setMotionParam(0, 0, 0, -roll * PI/180, 0, 0);
								 //mSixDofAlgorithm[1].setMotionParam(0, 0, 0, WX * PI / 180, WY * PI / 180, 0);
								 mSixDofAlgorithm[1].setMotionParam(0, 0, DZ, WX * PI / 180, WY * PI / 180, 0);
								 mSixDofAlgorithm[1].tickMotionControl();
								 //motioncal->InputParam(motioncal->DownPlatform, 0, 0, 0,WX, 0, 0);
								 //motioncal->Calculating(motioncal->DownPlatform);
								 //motioncal->InputParam(motioncal->UpPlatform, 0, 999.7 * tan(WX * PI / 180), 999.7 / cos(abs(WX)* PI / 180) - 815.95, -WX, 0, 0);
								 //motioncal->Calculating(motioncal->UpPlatform);
								 //motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, WX, 0, 0);
								 //motioncal->Calculating(motioncal->UpPlatform);
								 //if (out.is_open())
								 //{

									// //out << pitch << ";" << WX << "\n";

									// /*out << "This is a line.\n";
									// out << "This is another line.\n";*/
									// //out.close();
								 //}
								 for (int id = DOWN_SERVO_MOTOR_BEGIN; id < DOWN_SERVO_MOTOR_END + 1; id++){
									 _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
									 float data = (float)Positions / 1280000 * 10;
									 nowpos = data;
									 current_pos[id] = nowpos;

									 
									 _ECAT_Slave_Motion_Get_Torque(0, id, 0, &raw_torque);
									 // 2. 调用刚才封装的解算函数，一步得到当前的受力 (牛顿)
									 current_thrust_N[id] = CalculateThrustForce(raw_torque);

									 //PIDRatio m_PidRatio = { 250, 0.0014, 0 };
									 //motioncal->InputParam(motioncal->DownPlatform,0, 0, 0, WX, WY, 0);
									 //motioncal->Calculating(motioncal->DownPlatform);
									 float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, mSixDofAlgorithm[1].ctlOutput[id], m_PidRatio);
									 //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, motioncal->DownPlatform->Lenth[id], m_PidRatio);
									 //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, a[id - 6], m_PidRatio);
									 //PIDOutput = motioncal->Lenth[id - 6];
									 _ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
								 }
								 
								 if (up_trans){
									 //mSixDofAlgorithm[0].setMotionParam(motioncal->UpPlatform->Position[0],
										// motioncal->UpPlatform->Position[1],
										// motioncal->UpPlatform->Position[2],
										// //0,
										// //0,
										// -WX * PI / 180,
										// -WY * PI / 180,
										// 0);
									 mSixDofAlgorithm[0].tickMotionControl();
									 for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
										 _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
										 float data = (float)Positions / 1280000 * 10;
										 nowpos = data;
										 current_pos1[id - 6] = nowpos;
										 _ECAT_Slave_Motion_Get_Torque(0, id, 0, &raw_torque);
										 // 2. 调用刚才封装的解算函数，一步得到当前的受力 (牛顿)
										 current_thrust_N1[id - 6] = CalculateThrustForce(raw_torque);

										 //PIDRatio m_PidRatio = { 250, 0.0014, 0 };
										 //motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -WX, -WY, 0); 
										 //motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -m, 0, 0);
										 float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, mSixDofAlgorithm[0].ctlOutput[id - 6], m_PidRatio);
										 //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, motioncal->UpPlatform->Lenth[id-6], m_PidRatio);
										 //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, a[id - 6], m_PidRatio);
										 //PIDOutput = motioncal->Lenth[id - 6];
										 _ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
									 }
									 p = 0;
								 }
								 else
								 {

									 if (p == 0){
										 for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
											 _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
										 }
										 p = 1;
									 }
									 
								 }

								 //mSixDofAlgorithm[0].tickMotionControl();
								 //for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
									// _ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
									// float data = (float)Positions / 1280000 * 10;
									// nowpos = data;
									// //PIDRatio m_PidRatio = { 250, 0.0014, 0 };
									// //motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -WX, -WY, 0); 
									// //motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -m, 0, 0);
									// float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, mSixDofAlgorithm[0].ctlOutput[id - 6], m_PidRatio);
									// //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, motioncal->UpPlatform->Lenth[id-6], m_PidRatio);
									// //float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, a[id - 6], m_PidRatio);
									// //PIDOutput = motioncal->Lenth[id - 6];
									// //_ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
								 //}
							 //}
							 //else{
								// for (int id = DOWN_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
								//	 _ECAT_Slave_Motion_Sd_Stop(0, id, 0, 100);
								// }
								// break;
							 //}
							break;

		}
		case IMU_control:{
							if (roll > 12) {
								nowvel = 12;
							}
							else if (roll < -12){
								nowvel = -12;
							}
							else{
								nowvel = roll;
							}
							//nowpos = mySerialPort->INS550C.angle[0];
							//nowvel = mySerialPort->INS550C.angle[1];
							//PIDOutput = mySerialPort->INS550C.angle[2];
							//motioncal->InputParam(0, 0, 0, mySerialPort->INS550C.angle[0], 0, 0);
							//motioncal->Calculating();
							//for (int id = UP_SERVO_MOTOR_BEGIN; id < UP_SERVO_MOTOR_END + 1; id++){
							//	_ECAT_Slave_Motion_Get_Position(0, id, 0, &Positions);
							//	float data = (float)Positions / 1280000 * 10;
							//	nowpos = data;
							//	PIDRatio m_PidRatio = { 150, 0.0014, 0 };
							//	motioncal->InputParam(0, 0, 0, nowvel, 0, 0 );
							//	motioncal->Calculating();
							//	float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, motioncal->Lenth[id - 6], m_PidRatio);
							//	//float outputData = m_pidControl.single_Velocity_Control_PID(nowpos, a[id - 6], m_PidRatio);
							//	PIDOutput = motioncal->Lenth[id - 6];
							//	_ECAT_Slave_PV_Start_Move(0, id, 0, outputData, 100, 100);
							//}
							break;
		}
		default:
			break;
		}
		updataInfoCounter = 0;
	}
	updataInfoCounter++;
	
}

void MainWindow::on_ALL_Mid_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = All_Mid;
}

void MainWindow::on_ALL_Trans_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = All_Trans;
}

void MainWindow::on_X_valueChanged(int value)
{
    motionparam.X = ui->X->value();
	ui->X_output->setText(QString::number(motionparam.X));
	motioncal->InputParam(motioncal->DownPlatform,motionparam.X, motionparam.Y, motionparam.Z, motionparam.ROLL, motionparam.PITCH, motionparam.YAW);
	//PIDOutput = motioncal->Lenth[0];
}

void MainWindow::on_Y_valueChanged(int value)
{
	motionparam.Y = ui->Y->value();
	ui->Y_output->setText(QString::number(motionparam.Y));
	motioncal->InputParam(motioncal->DownPlatform, motionparam.X, motionparam.Y, motionparam.Z, motionparam.ROLL, motionparam.PITCH, motionparam.YAW);
}

void MainWindow::on_Pitch_valueChanged(int value)
{
	motionparam.PITCH = (double)ui->Pitch->value()/10;
	ui->Pitch_output->setText(QString::number(motionparam.PITCH));
	//motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, motionparam.ROLL, motionparam.PITCH, 0);
	//motioncal->InputParam(motioncal->DownPlatform,motionparam.X, motionparam.Y, motionparam.Z, motionparam.ROLL, motionparam.PITCH, motionparam.YAW);
	motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -motionparam.ROLL, -motionparam.PITCH, 0);
	motioncal->NowMotion(motioncal->UpPlatform);
	//mSixDofAlgorithm[0].setMotionParam(0,
	//	815.95 * sin(motionparam.ROLL * PI / 180 / 2) * 2 * cos(motionparam.ROLL * PI / 180 / 2),
	//	815.95 * sin(motionparam.ROLL * PI / 180 / 2) * 2 * sin(motionparam.ROLL * PI / 180 / 2),
	//	-motionparam.ROLL* PI / 180,
	//	-motionparam.PITCH* PI / 180,
	//	0);
	mSixDofAlgorithm[0].setMotionParam(motioncal->UpPlatform->Position[0],
		motioncal->UpPlatform->Position[1],
		motioncal->UpPlatform->Position[2],
		-motionparam.ROLL* PI / 180,
		-motionparam.PITCH* PI / 180,
		0);
	//mSixDofAlgorithm[0].setMotionParam(0, 0, 0, -motionparam.ROLL* PI / 180, -motionparam.PITCH* PI / 180, 0);
	mSixDofAlgorithm[1].setMotionParam(0, 0, 0, motionparam.ROLL* PI / 180, motionparam.PITCH* PI / 180, 0);
}

void MainWindow::on_Roll_valueChanged(int value)
{
	motionparam.ROLL = (double)ui->Roll->value()/10;
	ui->Roll_output->setText(QString::number(motionparam.ROLL));
	motioncal->InputParam(motioncal->UpPlatform, 0, 0, 0, -motionparam.ROLL, -motionparam.PITCH, 0);
	motioncal->NowMotion(motioncal->UpPlatform);
	//mSixDofAlgorithm[0].setMotionParam(0,
	//	1000 * sin(motionparam.ROLL * PI / 180 / 2) * 2 * cos(motionparam.ROLL * PI / 180 / 2),
	//	-1000 * sin(motionparam.ROLL * PI / 180 / 2) * 2 * sin(motionparam.ROLL * PI / 180 / 2),
	//	-motionparam.ROLL* PI / 180,
	//	-motionparam.PITCH* PI / 180,
	//	0);
	mSixDofAlgorithm[0].setMotionParam(motioncal->UpPlatform->Position[0],
		motioncal->UpPlatform->Position[1],
		motioncal->UpPlatform->Position[2],
		-motionparam.ROLL* PI / 180,
		-motionparam.PITCH* PI / 180,
		0);
	mSixDofAlgorithm[1].setMotionParam(0, 0, 0, motionparam.ROLL* PI / 180, motionparam.PITCH* PI / 180, 0);
	//motioncal->InputParam(motioncal->UpPlatform, 0, 815.95 * tan(motionparam.ROLL * PI / 180), 815.95 / cos(abs(motionparam.ROLL)* PI / 180) - 815.95, -motionparam.ROLL, -motionparam.PITCH, 0);
	//motioncal->InputParam(motioncal->DownPlatform,motionparam.X, motionparam.Y, motionparam.Z, motionparam.ROLL, motionparam.PITCH, motionparam.YAW);
}

void MainWindow::on_Yaw_valueChanged(int value)
{
	motionparam.YAW = (double)ui->Yaw->value();
	ui->Yaw_output->setText(QString::number(motionparam.YAW));
	motioncal->InputParam(motioncal->DownPlatform,motionparam.X, motionparam.Y, motionparam.Z, motionparam.ROLL, motionparam.PITCH, motionparam.YAW);
}

void MainWindow::on_Z_valueChanged(int value)
{
	motionparam.Z = ui->Z->value()*10;
	ui->Z_output->setText(QString::number(motionparam.Z));
	//motioncal->InputParam(motioncal->UpPlatform, 0, 0, -motionparam.Z, -motionparam.ROLL, -motionparam.PITCH, 0);
	//motioncal->InputParam(motioncal->DownPlatform,motionparam.X, motionparam.Y, motionparam.Z, motionparam.ROLL, motionparam.PITCH, motionparam.YAW);
	motioncal->InputParam(motioncal->UpPlatform, 0, 0, -motionparam.Z, -motionparam.ROLL, -motionparam.PITCH, 0);
	motioncal->NowMotion(motioncal->UpPlatform);
	mSixDofAlgorithm[0].setMotionParam(motioncal->UpPlatform->Position[0],
		motioncal->UpPlatform->Position[1],
		motioncal->UpPlatform->Position[2],
		-motionparam.ROLL* PI / 180,
		-motionparam.PITCH* PI / 180,
		0);
	mSixDofAlgorithm[1].setMotionParam(0, 0, motionparam.Z, motionparam.ROLL* PI / 180, motionparam.PITCH* PI / 180, 0);
}

void MainWindow::on_X_0_clicked()
{
	ui->X->setValue(0);
}

void MainWindow::on_Y_0_clicked()
{
	ui->Y->setValue(0);
}

void MainWindow::on_Pitch_0_clicked()
{
	ui->Pitch->setValue(0);
}

void MainWindow::on_Roll_0_clicked()
{
	ui->Roll->setValue(0);
}

void MainWindow::on_Yaw_0_clicked()
{
	ui->Yaw->setValue(0);
}

void MainWindow::on_Z_0_clicked()
{
	ui->Z->setValue(0);
}

void MainWindow::on_Motion_Test_clicked()
{
	//ui->Z->setValue(10);
	m_ControlIntermediateQuantity.mMotorMode = Motion_Test;
	//ui->state->setText("hello");
}

void MainWindow::on_UP_Mid_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = All_Mid;
	up_mid = true;
	up_mid_check = 0;
}

void MainWindow::on_UP_STOP_clicked()
{
	
}

void MainWindow::on_UP_ZERO_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = All_Zero;
	up_zero = true;
	up_zero_check = 0;
}

void MainWindow::on_DOWN_Mid_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = All_Mid;
	down_mid = true;
	down_mid_check = 0;
}

void MainWindow::on_DOWN_STOP_clicked()
{

}

void MainWindow::on_DOWN_ZERO_clicked()
{
	m_ControlIntermediateQuantity.mMotorMode = All_Zero;
	down_zero = true;
	down_zero_check = 0;
}

void MainWindow::on_UP_Start_clicked(bool checked)
{
	if (checked)
	{
		for (SlaveNO = UP_SERVO_MOTOR_BEGIN; SlaveNO < UP_SERVO_MOTOR_END + 1; SlaveNO++){
			_ECAT_Slave_Motion_Set_Svon(0, SlaveNO, 0, Enable);
		}
		if (timer == false){
			startcount();
			timer = true;
		}
		ui->UP_Start->setText("关闭伺服-上");
	}
	else
	{
		for (SlaveNO = UP_SERVO_MOTOR_BEGIN; SlaveNO < UP_SERVO_MOTOR_END + 1; SlaveNO++){
			_ECAT_Slave_Motion_Set_Svon(0, SlaveNO, 0, Unable);
		}
		if (timer == true){
			stopcount();
			timer = false;
		}
		if (myTimer->isActive() == true)
		{
			myTimer->stop();
		}
		ui->UP_Start->setText("打开伺服-上");
	}
}

void MainWindow::on_DOWN_Start_clicked(bool checked)
{
	if (checked)
	{
		for (SlaveNO = DOWN_SERVO_MOTOR_BEGIN; SlaveNO < DOWN_SERVO_MOTOR_END + 1; SlaveNO++){
			_ECAT_Slave_Motion_Set_Svon(0, SlaveNO, 0, Enable);
		}
		if (timer == false){
			startcount();
			timer = true;
		}
		ui->DOWN_Start->setText("关闭伺服-下");
	}
	else
	{
		for (SlaveNO = DOWN_SERVO_MOTOR_BEGIN; SlaveNO < DOWN_SERVO_MOTOR_END + 1; SlaveNO++){
			_ECAT_Slave_Motion_Set_Svon(0, SlaveNO, 0, Unable);
		}
		if (timer == true){
			stopcount();
			timer = false;
		}
		if (myTimer->isActive() == true)
		{
			myTimer->stop();
		}
		ui->DOWN_Start->setText("打开伺服-下");
	}
}

void MainWindow::on_UP_Trans_clicked(bool checked)
{
	if (checked)
	{
		ui->UP_Trans->setText("上-结束补偿");
		//m_ControlIntermediateQuantity.mMotorMode = All_Trans;
		up_trans = true;
	}
	else
	{
		ui->UP_Trans->setText("上-开始补偿");
		up_trans = false;
	}
}

void MainWindow::on_DOWN_Trans_clicked(bool checked)
{
	if (checked)
	{
		ui->DOWN_Trans->setText("下-结束");
		//m_ControlIntermediateQuantity.mMotorMode = All_Trans;
		down_trans = true;
	}
	else
	{
		ui->DOWN_Trans->setText("下-开始");
		down_trans = false;
	}
}
