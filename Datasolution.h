#pragma once
#include <Windows.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <algorithm>
#include "6Dof.h"

// 设备类型枚举
enum DeviceType {
    DEVICE_FORCE_SENSOR,
    DEVICE_IMU
};

// 力传感器数据结构
struct ForceSensorData {
    double values[6];  // 6个通道的数据
    DWORD lastUpdate;    // 最后更新时间

    // 添加构造函数
    ForceSensorData();

    bool IsValid() const;
};

// IMU数据结构
struct IMUData {
    double angles[3];          // 姿态角
    double angularVelocity[3]; // 角速度
    double acceleration[3];    // 加速度
    DWORD lastUpdate;            // 最后更新时间

    // 添加构造函数
    IMUData();

    bool IsValid() const;
};

// 数据记录器类 - 负责存储传感器数据
class DataLogger {
public:
    DataLogger();

    ~DataLogger();

    // 初始化数据存储
    bool Initialize(size_t IMU_count);

    // 记录力传感器数据
    void LogForceData(const ForceSensorData& data);

    // 记录IMU数据
    void LogIMUData(size_t index, const IMUData& data , const double ctlOutput[], const double ctlInput[]);

    // 关闭文件
    void CloseFiles();

private:
    FILE* forceDataFile;  // 力传感器数据文件
    std::vector<FILE*>  imuDataFiles_;    // IMU数据文件
    std::string GetTimestamp();
};

// 串口设备基类
class SerialDevice {
public:
    SerialDevice(int portNumber, int baudRate, DeviceType type);

    virtual ~SerialDevice();

    // 打开串口
    bool Open();

    // 关闭串口
    void Close();

    // 启动设备线程
    virtual bool Start() = 0;

    // 停止设备线程
    virtual void Stop() = 0;

    // 获取设备类型
    DeviceType GetType() const { return type_; }

protected:
    HANDLE hComm_;              // 串口句柄
    OVERLAPPED ovRead_;         // 读操作重叠结构
    OVERLAPPED ovWrite_;        // 写操作重叠结构
    int portNumber_;            // 端口号
    int baudRate_;              // 波特率
    DeviceType type_;           // 设备类型
    bool isRunning_;            // 线程运行标志
};

// 力传感器设备类 (问答式通信)
class ForceSensorDevice : public SerialDevice {
public:
    ForceSensorDevice(int portNumber, int baudRate);

    virtual ~ForceSensorDevice();


    // 启动设备线程
    bool Start() override;

    // 停止设备线程
    void Stop() override;

    // 获取传感器数据
    ForceSensorData GetData() const {
        return data_;
    }

private:
    // 力传感器线程函数
    static DWORD WINAPI ForceSensorThreadProc(LPVOID lpParam);

    // 初始化命令列表
    void InitializeCommands();

    // 发送命令
    bool SendCommand(const std::vector<BYTE>& command);

    // 读取响应
    bool ReadResponse(BYTE* buffer, DWORD bufferSize, DWORD& bytesRead);

    // 解析响应
    void ParseResponse(int sensorIdx, const BYTE* data, DWORD size);

private:
    HANDLE hThread_;                     // 线程句柄
    std::vector<std::vector<BYTE>> commands_;    // 命令列表
    ForceSensorData data_;                       // 传感器数据
};

// IMU设备类 (连续输出)
class IMUDevice : public SerialDevice {
public:
    IMUDevice(int portNumber, int baudRate);

    virtual ~IMUDevice();

    // 启动设备线程
    bool Start() override;

    // 停止设备线程
    void Stop() override;

    // 获取IMU数据
    IMUData GetData() const {
        return data_;
    }

private:
    // IMU线程函数
    static DWORD WINAPI IMUThreadProc(LPVOID lpParam);

    // 解析IMU数据
    void ParseData(const BYTE* data, DWORD size);

private:
    HANDLE hThread_ ;  // 线程句柄
    IMUData data_;           // IMU数据
};

// 设备管理器
class DeviceManager {
public:
    ~DeviceManager();

    // 添加设备
    void AddDevice(SerialDevice* device);

    // 启动所有设备
    bool StartAllDevices();

    // 停止所有设备
    void StopAllDevices();

    // 获取力传感器数据
    ForceSensorData GetForceSensorData() const;

    // 获取IMU数据
    IMUData GetIMUData(size_t index) const;
    size_t GetIMUCount() const;

private:
    std::vector<SerialDevice*> devices_;
};
double CalculateThrustForce(short raw_torque);