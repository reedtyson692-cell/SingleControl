#ifndef MAINWINDOW_H
#define MAINWINDOW_H
//qt代码+中断函数
#include <QMainWindow>
#include <QString>
#include <QTimer>
#include "MyChart.h"
#include "compatibility.h"
#include "bdaqctrl.h"
#include "DriverTransInterface.h"
#include "EtherCatbus.h"
#include "PIDController.h"
#include "MotionCal.h"
#include "serialPort.h"
#include "WS601Resolve.h"
#include"Datasolution.h"

using namespace Automation::BDaq;

#define TICK_LOWSET_POINT  0
#define UP_TICK_MIDEST_POINT  130
#define DOWN_TICK_MIDEST_POINT  230
#define REACH_ERROR 35

namespace Ui {
class MainWindow;
}

typedef enum{

	DOWN_SERVO_MOTOR_BEGIN = 0,
	DOWN_SERVO_MOTOR_0 = DOWN_SERVO_MOTOR_BEGIN,
	DOWN_SERVO_MOTOR_1 = 1,
	DOWN_SERVO_MOTOR_2 = 2,
	DOWN_SERVO_MOTOR_3 = 3,
	DOWN_SERVO_MOTOR_4 = 4,
	DOWN_SERVO_MOTOR_5 = 5,
	DOWN_SERVO_MOTOR_END = DOWN_SERVO_MOTOR_5,

	UP_SERVO_MOTOR_BEGIN = 6,
	UP_SERVO_MOTOR_0 = UP_SERVO_MOTOR_BEGIN,
	UP_SERVO_MOTOR_1 = 7,
	UP_SERVO_MOTOR_2 = 8,
	UP_SERVO_MOTOR_3 = 9,
	UP_SERVO_MOTOR_4 = 10,
	UP_SERVO_MOTOR_5 = 11,
	UP_SERVO_MOTOR_END = UP_SERVO_MOTOR_5,

	WINDLASS_SERVO_MOTOR = 12,

}SERVO_MOTORS;//����������

typedef enum
{
	Run_No_Mode,
	Single_Zero,
	Single_PID,
	All_Zero,
	Single_Sine,
	All_Sine,
	All_Mid,
	All_Trans,
	Motion_Test,
	IMU_control
}MotorControlMode;//当前使用的模式，作为显示和获取速度位置信息的凭证

typedef struct
{
	float samplingTime;
	int   currentSingleAxisId;
	bool  mDownNormalControlFlag;
	bool  mUpNormalControlFlag;
	bool  singleAxisControlFlag;
	bool  startAppendRecord;
	MotorControlMode mMotorMode;//当前使用的模式，作为显示和获取速度位置信息的凭证
	//float nowpos;
	I16 TargetPOS;
	float xmotion;
	//MotionStatus mCurrentStatus[12];
	//DOF_PlatformData m_DOF_PlatformData[2];
	//GroyAcqData m_groyAcqData;
}ControlIntermediateQuantity;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
	//��ʱ������
	//static bool up_mid;
	//static bool down_mid;
	//bool timer;
	void initialize();
	void startcount();
	void stopcount();
	static void BDAQCALL OnCounterEvent(void *sender, CntrEventArgs *args, void * userParam);
	void initSystemStruct();
	static EtherCatBus *mEtherCatBus;
	MotionParam motionparam;
	static DataLogger* dataLogger;
	static DeviceManager* manager;
	//static CSerialPort* mySerialPort;//已用Datasolution
	//static I16 targetPosition;
	
private slots:
    void on_init_clicked();
    void on_findslave_clicked();
    void on_setMotor_valueChanged();
    void on_setVelocity_valueChanged();
	void on_setPosition_valueChanged();
    void on_start_clicked(bool checked);
    void on_stop_clicked();
    void on_rotate_clicked();
	void on_reset_clicked();
    void on_ALL_UP_clicked();
    void on_ALL_STOP_clicked();
    void on_ALL_ZERO_clicked();
    void on_PID_clicked();
    void on_ALL_Mid_clicked();
    void on_ALL_Trans_clicked();
    void on_X_valueChanged(int value);
    void on_Y_valueChanged(int value);
    void on_Pitch_valueChanged(int value);
    void on_Roll_valueChanged(int value);
    void on_Yaw_valueChanged(int value);
    void on_Z_valueChanged(int value);
    void on_X_0_clicked();
    void on_Y_0_clicked();
    void on_Pitch_0_clicked();
    void on_Roll_0_clicked();
    void on_Yaw_0_clicked();
    void on_Z_0_clicked();

    void on_Motion_Test_clicked();

    void on_UP_Mid_clicked();

    void on_UP_STOP_clicked();

    void on_UP_ZERO_clicked();

    void on_DOWN_Mid_clicked();

    void on_DOWN_STOP_clicked();

    void on_DOWN_ZERO_clicked();

    void on_UP_Start_clicked(bool checked);

    void on_DOWN_Start_clicked(bool checked);

    void on_UP_Trans_clicked(bool checked);

    void on_DOWN_Trans_clicked(bool checked);
	void onStart();
	void onStop();
	void onUpdateData();

private:
    Ui::MainWindow *ui;
    EtherCatBus stewart;
    DriverTransInterface stewartDriver;
	QTimer *myTimer;
	U16 SlaveNO;
	I16 Velocity;
	U16 Enable = 1;
	U16 Unable = 0;
    U16 gESCCardNo;
	//��ʱ������
	//static EtherCatBus *mEtherCatBus;
	static TimerPulseCtrl* timerPulseCtrl;
	static MotionCal* motioncal;
	double targetFrequence;
	double currentFrequence;
	int eventCount;
	//static PIDRatio m_PidRatio;
	ErrorCode ret = Success;
	static float pos;										//ʵ��������
	//static bool isLowest;
	MyChart *m_pChart;
	QTimer *m_pTimer;
};

#endif // MAINWINDOW_H
