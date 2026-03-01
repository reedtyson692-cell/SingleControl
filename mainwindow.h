#ifndef MAINWINDOW_H
#define MAINWINDOW_H
//qt代码+中断函数
#include <QMainWindow>
#include <QString>
#include <QTimer>
#include <QThread>
#include <QDateTime>
#include <atomic>
#include <cstring>
#include <cstdio>
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

// 单帧日志快照：由高频控制回调快速填充并投递到环形队列
// 设计目标：固定大小、连续内存、可 memcpy，尽量降低 1ms 回调负担
struct SystemLogFrame
{
	DWORD timestamp;
	double down_pos[6];
	double down_thrust[6];
	double down_roll;
	double down_pitch;
	double down_yaw;
	double up_pos[6];
	double up_thrust[6];

	SystemLogFrame()
	{
		memset(this, 0, sizeof(SystemLogFrame));
	}
};

// SPSC（单生产者/单消费者）无锁环形缓冲区
// - 生产者：OnCounterEvent（高频路径）
// - 消费者：DataSaveThread（低优先级落盘线程）
// VS2013 对 std::atomic 支持可满足该轻量队列实现
template <typename T, size_t Size>
class LockFreeRingBuffer
{
private:
	T buffer[Size];
	std::atomic<size_t> head;
	std::atomic<size_t> tail;

public:
	LockFreeRingBuffer()
	{
		head.store(0);
		tail.store(0);
	}

		// 生产者入队：满队列时返回 false（丢弃当前帧，避免阻塞控制线程）
	bool push(const T& item)
	{
		size_t currentHead = head.load(std::memory_order_relaxed);
		size_t nextHead = (currentHead + 1) % Size;
		if (nextHead == tail.load(std::memory_order_acquire)) {
			return false;
		}
		buffer[currentHead] = item;
		head.store(nextHead, std::memory_order_release);
		return true;
	}

		// 消费者出队：空队列返回 false
	bool pop(T& item)
	{
		size_t currentTail = tail.load(std::memory_order_relaxed);
		if (currentTail == head.load(std::memory_order_acquire)) {
			return false;
		}
		item = buffer[currentTail];
		tail.store((currentTail + 1) % Size, std::memory_order_release);
		return true;
	}
};

// 专用落盘线程：仅负责从队列取样并写 CSV
// 注意：必须在析构前 stopThread()+wait()，避免 Qt5.8 常见的线程未退出崩溃
class DataSaveThread : public QThread
{
public:
	LockFreeRingBuffer<SystemLogFrame, 8192>* m_ringBuffer;
	std::atomic<bool> m_running;
	FILE* m_csvFile;

	explicit DataSaveThread(QObject* parent = 0);
	~DataSaveThread();

		// 打开日志文件并启动线程（低优先级）
	void startLogging(const QString& fileName);
		// 安全停线程：原子标志位置位 + wait() 阻塞等待 run() 退出
	void stopThread();

protected:
	virtual void run();
};

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
		// 全局日志队列（静态）：供中断回调与后台线程解耦传输数据
	static LockFreeRingBuffer<SystemLogFrame, 8192> g_logBuffer;
		// 后台落盘线程实例（生命周期跟随 MainWindow）
	DataSaveThread* m_saveThread;
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
