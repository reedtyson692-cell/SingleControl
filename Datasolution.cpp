#include"Datasolution.h"
// ====================== 数据结构实现 ======================

    // 添加构造函数
ForceSensorData::ForceSensorData() {
    // 在构造函数中初始化数组
    for (int i = 0; i < 6; i++) values[i] = 0.0;
    lastUpdate = 0;
}

bool ForceSensorData::IsValid() const {
    return (GetTickCount() - lastUpdate) < 500; // 500ms内有效
}

IMUData::IMUData() {
    // 初始化所有数组
    for (int i = 0; i < 3; i++) {
        angles[i] = 0.0;
        angularVelocity[i] = 0.0;
        acceleration[i] = 0.0;
    }
    lastUpdate = 0;
}

bool IMUData::IsValid() const {
    return (GetTickCount() - lastUpdate) < 500; // 500ms内有效
}

// ====================== 数据记录器实现 ======================
DataLogger::DataLogger() : forceDataFile(nullptr) {}

DataLogger::~DataLogger() {
    CloseFiles();
}

std::string DataLogger::GetTimestamp() {
    std::time_t now = std::time(nullptr);
    std::tm* tm = std::localtime(&now);
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", tm);
    return std::string(timeStr);
}

    // 初始化数据存储
bool DataLogger::Initialize(size_t IMU_count) {
    // 生成基于时间的文件名
    std::time_t now = std::time(nullptr);
    std::tm* tm = std::localtime(&now);
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm);

    // 创建力传感器数据文件
    std::string forceFileName = "ForceData_" + std::string(timestamp) + ".csv";
    forceDataFile = fopen(forceFileName.c_str(), "w");
    if (!forceDataFile) {
        std::cerr << "无法创建力传感器数据文件" << std::endl;
        return false;
    }
// 写入CSV表头
    fprintf(forceDataFile, "Timestamp,SensorID,F5,F0,F1,F2,F3,F4\n");
    fflush(forceDataFile);
    std::cout << "数据存储已初始化:" << std::endl;
    std::cout << "  力传感器: " << forceFileName << std::endl;
/******************************************************************************/
    for (size_t i = 0; i < IMU_count; i++)
    {
        // 创建IMU数据文件
        std::stringstream ss;
        ss << "IMU" << (i + 1) << "_Data_" << timestamp << ".csv";
        std::string imuFileName = ss.str();
        FILE* imuDataFile = fopen(imuFileName.c_str(), "w");
        if (!imuDataFile) {
            std::cerr << "无法创建IMU数据文件" << (i + 1) << std::endl;
            return false;
        }

        // 写入CSV表头
        fprintf(imuDataFile, "Timestamp,SensorID,Roll,Pitch,Yaw,GyroX,GyroY,GyroZ,AccelX,AccelY,AccelZ,轴长1,轴长2,轴长3,轴长4,轴长5,轴长6,推力1,推力2,推力3,推力4,推力5,推力6\n");
        fflush(imuDataFile);
        imuDataFiles_.push_back(imuDataFile);
        std::cout << "  IMU: " << (i + 1) << "数据文件: " << imuFileName << std::endl;
    }
    return true;
}

    // 记录力传感器数据
void DataLogger::LogForceData(const ForceSensorData& data) {
    if (!forceDataFile || !data.IsValid()) return;

    DWORD timestamp = GetTickCount();

    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* tm = std::localtime(&now);
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", tm);

    // 写入数据到CSV
    fprintf(forceDataFile, "%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        timeStr,
        timestamp,
        data.values[0], data.values[1], data.values[2],
        data.values[3], data.values[4], data.values[5]);

    fflush(forceDataFile); // 确保数据立即写入
}

// 记录IMU数据
void DataLogger::LogIMUData(size_t index, const IMUData& data, const double ctlOutput[], const double ctlInput[]) {
    //if (index >= imuDataFiles_.size() || !imuDataFiles_[index] || !data.IsValid()) return;

    DWORD timestamp = GetTickCount();

    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* tm = std::localtime(&now);
    char timeStr[20];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", tm);

    // 写入数据到CSV
    fprintf(imuDataFiles_[index], "%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
        timeStr,
        timestamp,
        data.angles[0], data.angles[1], data.angles[2],
        data.angularVelocity[0], data.angularVelocity[1], data.angularVelocity[2],
        data.acceleration[0], data.acceleration[1], data.acceleration[2],
        ctlOutput[0], ctlOutput[1], ctlOutput[2], 
        ctlOutput[3], ctlOutput[4], ctlOutput[5],
        ctlInput[0], ctlInput[1], ctlInput[2],
        ctlInput[3], ctlInput[4], ctlInput[5]);

    fflush(imuDataFiles_[index]); // 确保数据立即写入
}

// 关闭文件
void DataLogger::CloseFiles() {
    if (forceDataFile) {
        fclose(forceDataFile);
        forceDataFile = nullptr;
    }
    for (FILE* file : imuDataFiles_) {
        if (file) {
            fclose(file);
        }
    }
    imuDataFiles_.clear();
}

// ====================== 串口设备基类实现 ======================

SerialDevice::SerialDevice(int portNumber, int baudRate, DeviceType type)
    : portNumber_(portNumber), baudRate_(baudRate), type_(type),
    hComm_(INVALID_HANDLE_VALUE), isRunning_(false) {}

SerialDevice::~SerialDevice() {
    Close();
}

// 打开串口
bool SerialDevice::Open() {
    char portName[20];
    sprintf(portName, "\\\\.\\COM%d", portNumber_);

    hComm_ = CreateFileA(
        portName,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_FLAG_OVERLAPPED,
        NULL
    );

    if (hComm_ == INVALID_HANDLE_VALUE) {
        std::cerr << "无法打开串口 COM" << portNumber_ << " 错误码: " << GetLastError() << std::endl;
        return false;
    }

    // 配置串口参数
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(hComm_, &dcb)) {
        std::cerr << "获取串口状态失败" << std::endl;
        return false;
    }

    dcb.BaudRate = baudRate_;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    if (!SetCommState(hComm_, &dcb)) {
        std::cerr << "设置串口状态失败" << std::endl;
        return false;
    }

    // 配置超时
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    if (!SetCommTimeouts(hComm_, &timeouts)) {
        std::cerr << "设置超时失败" << std::endl;
        return false;
    }

    // 初始化重叠结构
    ZeroMemory(&ovRead_, sizeof(OVERLAPPED));
    ZeroMemory(&ovWrite_, sizeof(OVERLAPPED));
    ovRead_.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (ovRead_.hEvent == NULL) {
        std::cerr << "创建读事件失败" << std::endl;
        return false;
    }

    return true;
}

// 关闭串口
void SerialDevice::Close() {
    if (hComm_ != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm_);
        hComm_ = INVALID_HANDLE_VALUE;
    }
    if (ovRead_.hEvent != NULL) {
        CloseHandle(ovRead_.hEvent);
        ovRead_.hEvent = NULL;
    }
}

// ====================== 力传感器设备实现 ======================


ForceSensorDevice::ForceSensorDevice(int portNumber, int baudRate)
    : SerialDevice(portNumber, baudRate, DEVICE_FORCE_SENSOR) {
    InitializeCommands();
}

ForceSensorDevice::~ForceSensorDevice() {
    Stop();
}

// 初始化命令列表
void ForceSensorDevice::InitializeCommands() {
    // 创建6个传感器的命令
    commands_.resize(6);

    // 传感器1
    commands_[0] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B };

    // 传感器2
    commands_[1] = { 0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x38 };

    // 传感器3
    commands_[2] = { 0x03, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC5, 0xE9 };

    // 传感器4
    commands_[3] = { 0x04, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x5E };

    // 传感器5
    commands_[4] = { 0x05, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC5, 0x8F };

    // 传感器6
    commands_[5] = { 0x06, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC5, 0xBC };
}

// 启动设备线程
bool ForceSensorDevice::Start() {
    if (hComm_ == INVALID_HANDLE_VALUE) {
        if (!Open()) return false;
    }

    isRunning_ = true;
    hThread_ = CreateThread(
        NULL,
        0,
        ForceSensorThreadProc,
        this,
        0,
        NULL
    );

    if (hThread_ == NULL) {
        std::cerr << "创建力传感器线程失败" << std::endl;
        return false;
    }

    // 设置线程优先级
    SetThreadPriority(hThread_, THREAD_PRIORITY_ABOVE_NORMAL);
    return true;
}

// 停止设备线程
void ForceSensorDevice::Stop() {
    isRunning_ = false;
    if (hThread_ != NULL) {
        WaitForSingleObject(hThread_, 1000);
        CloseHandle(hThread_);
        hThread_ = NULL;
    }
}

// 力传感器线程函数
DWORD WINAPI ForceSensorDevice::ForceSensorThreadProc(LPVOID lpParam) {
    ForceSensorDevice* device = static_cast<ForceSensorDevice*>(lpParam);
    BYTE response[256];

    while (device->isRunning_) {
        // 循环处理6个传感器
        for (int sensorIdx = 0; sensorIdx < 6; sensorIdx++) {
            if (!device->isRunning_) break;

            // 发送命令
            if (!device->SendCommand(device->commands_[sensorIdx])) {
                std::cerr << "发送命令到传感器 " << (sensorIdx + 1) << " 失败" << std::endl;
                continue;
            }
            Sleep(15);
            // 等待并读取响应
            DWORD bytesRead = 0;
            if (device->ReadResponse(response, sizeof(response), bytesRead)) {
                // 解析响应
                device->ParseResponse(sensorIdx, response, bytesRead);
            }

            // 短暂延迟，避免总线冲突
            Sleep(10);
        }

        // 控制整体轮询频率
        Sleep(5);
    }

    return 0;
}

// 发送命令
bool ForceSensorDevice::SendCommand(const std::vector<BYTE>& command) {
    DWORD bytesWritten = 0;
    OVERLAPPED ov = { 0 };
    ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    BOOL result = WriteFile(
        hComm_,
        &command[0],
        static_cast<DWORD>(command.size()),
        &bytesWritten,
        &ov
    );

    if (!result) {
        DWORD err = GetLastError();
        if (err != ERROR_IO_PENDING) {
            CloseHandle(ov.hEvent);
            return false;
        }

        // 等待写入完成
        DWORD waitResult = WaitForSingleObject(ov.hEvent, 50);
        if (waitResult != WAIT_OBJECT_0) {
            CloseHandle(ov.hEvent);
            return false;
        }

        if (!GetOverlappedResult(hComm_, &ov, &bytesWritten, FALSE)) {
            CloseHandle(ov.hEvent);
            return false;
        }
    }

    CloseHandle(ov.hEvent);
    return bytesWritten == command.size();
}

// 读取响应
bool ForceSensorDevice::ReadResponse(BYTE* buffer, DWORD bufferSize, DWORD& bytesRead) {
    bytesRead = 0;

    // 发起异步读取
    BOOL result = ReadFile(
        hComm_,
        buffer,
        bufferSize,
        NULL,
        &ovRead_
    );

    if (!result) {
        DWORD err = GetLastError();
        if (err != ERROR_IO_PENDING) {
            return false;
        }
    }

    // 等待数据到达 (最多50ms)
    DWORD waitResult = WaitForSingleObject(ovRead_.hEvent, 50);
    if (waitResult != WAIT_OBJECT_0) {
        CancelIo(hComm_); // 取消未完成操作
        return false;
    }

    // 获取实际读取字节数
    if (!GetOverlappedResult(hComm_, &ovRead_, &bytesRead, FALSE)) {
        return false;
    }

    return bytesRead > 0;
}

// 解析响应
void ForceSensorDevice::ParseResponse(int sensorIdx, const BYTE* data, DWORD size) {
    // 协议帧结构: [地址][功能码][数据长度][数据H][数据L][状态][CRC1][CRC2]
    if (size < 9) return;

    // 验证传感器地址
    if (data[0] != commands_[sensorIdx][0]) return;

    // 合并数据字节
    int rawValue = (data[3] << 8) | data[4];
    double force = 0.0;

    // 符号处理
    if (data[5] & 0x80) {
        force = (rawValue - 65536) * -0.01;  // 负值转换
    }
    else {
        force = rawValue * 0.01;             // 正值转换
    }

    // 更新数据
    data_.values[sensorIdx] = force;
    data_.lastUpdate = GetTickCount();
}


// ====================== IMU设备实现 ======================

IMUDevice::IMUDevice(int portNumber, int baudRate)
    : SerialDevice(portNumber, baudRate, DEVICE_IMU) {}

IMUDevice::~IMUDevice() {
    Stop();
}

// 启动设备线程
bool IMUDevice::Start() {
    if (hComm_ == INVALID_HANDLE_VALUE) {
        if (!Open()) return false;
    }

    isRunning_ = true;
    hThread_ = CreateThread(
        NULL,
        0,
        IMUThreadProc,
        this,
        0,
        NULL
    );

    if (hThread_ == NULL) {
        std::cerr << "创建IMU线程失败" << std::endl;
        return false;
    }

    // 设置线程优先级
    SetThreadPriority(hThread_, THREAD_PRIORITY_ABOVE_NORMAL);
    return true;
}

// 停止设备线程
void IMUDevice::Stop() {
    isRunning_ = false;
    if (hThread_ != NULL) {
        WaitForSingleObject(hThread_, 1000);
        CloseHandle(hThread_);
        hThread_ = NULL;
    }
}


// IMU线程函数
DWORD WINAPI IMUDevice::IMUThreadProc(LPVOID lpParam) {
    IMUDevice* device = static_cast<IMUDevice*>(lpParam);
    BYTE buffer[256];
    DWORD bytesRead = 0;
    DWORD totalBytes = 0;

    while (device->isRunning_) {
        // 发起异步读取
        BOOL result = ReadFile(
            device->hComm_,
            buffer + totalBytes,
            sizeof(buffer) - totalBytes,
            NULL,
            &device->ovRead_
        );

        if (!result) {
            DWORD err = GetLastError();
            if (err != ERROR_IO_PENDING) {
                Sleep(10);
                continue;
            }
        }

        // 等待数据到达 (最多100ms)
        DWORD waitResult = WaitForSingleObject(device->ovRead_.hEvent, 100);
        if (waitResult != WAIT_OBJECT_0) {
            CancelIo(device->hComm_);
            continue;
        }

        // 获取实际读取字节数
        if (!GetOverlappedResult(device->hComm_, &device->ovRead_, &bytesRead, FALSE)) {
            continue;
        }

        totalBytes += bytesRead;

        // 处理完整数据帧
        if (totalBytes >= 32) {
            device->ParseData(buffer, totalBytes);
            totalBytes = 0;
        }
    }

    return 0;
}

// 解析IMU数据
void IMUDevice::ParseData(const BYTE* data, DWORD size) {
    // 查找帧头 (0xBD, 0xDB, 0x04)
    for (DWORD i = 0; i < size - 2; i++) {
        if (data[i] == 0xBD && data[i + 1] == 0xDB && data[i + 2] == 0x04) {
            // 确保有足够数据
            if (i + 32 > size) break;

            const BYTE* frame = data + i;
            int index = 3;  // 数据起始位置

            // 解析角度数据
            for (int j = 0; j < 3; j++) {
                // 小端序转换
                short temp = static_cast<short>((frame[index + 1] << 8) | frame[index]);
                data_.angles[j] = temp * 360.0 / 32768.0;
                index += 2;
            }
            index = index + 4;
            // 解析角速度
            for (int j = 0; j < 3; j++) {
                short temp = static_cast<short>((frame[index + 1] << 8) | frame[index]);
                data_.angularVelocity[j] = temp * 150.0 / 32768.0;
                index += 2;
            }

            // 解析加速度
            for (int j = 0; j < 3; j++) {
                short temp = static_cast<short>((frame[index + 1] << 8) | frame[index]);
                data_.acceleration[j] = temp * 2.0 / 32768.0;
                index += 2;
            }

            // 更新时间戳
            data_.lastUpdate = GetTickCount();

            // 跳过已处理的数据
            i += 31;
        }
    }
}

// ====================== 设备管理器实现 ======================


DeviceManager::~DeviceManager() {
    StopAllDevices();
}

// 添加设备
void DeviceManager::AddDevice(SerialDevice* device) {
    devices_.push_back(device);
}

// 启动所有设备
bool DeviceManager::StartAllDevices() {
    for (SerialDevice* device : devices_) {
        if (!device->Start()) {
            std::cerr << "启动设备失败" << std::endl;
            return false;
        }
    }
    return true;
}

// 停止所有设备
void DeviceManager::StopAllDevices() {
    for (SerialDevice* device : devices_) {
        device->Stop();
        delete device;
    }
    devices_.clear();
}

// 获取力传感器数据
ForceSensorData DeviceManager::GetForceSensorData() const {
    for (SerialDevice* device : devices_) {
        if (device->GetType() == DEVICE_FORCE_SENSOR) {
            ForceSensorDevice* sensor = dynamic_cast<ForceSensorDevice*>(device);
            if (sensor) return sensor->GetData();
        }
    }
    return ForceSensorData();
}

// 获取IMU数据
IMUData DeviceManager::GetIMUData(size_t index) const {
    size_t count = 0;
    for (SerialDevice* device : devices_) {
        if (device->GetType() == DEVICE_IMU) {
            if (count == index){
                IMUDevice* imu = dynamic_cast<IMUDevice*>(device);
                if (imu) return imu->GetData();
            }
            count++;
        }
    }
    return IMUData();
}
// 获取IMU设备总数(未使用)
size_t DeviceManager::GetIMUCount() const {
    size_t count = 0;
    for (SerialDevice* device : devices_) {
        if (device->GetType() == DEVICE_IMU) {
            count++;
        }
    }
    return count;
}
/**
 * @brief 计算电动缸当前的直线推力 (高精度双精度版本)
 * @param raw_torque 从驱动器读取的千分比原始扭矩反馈 (0.1%)，底层传入必须为 short (I16)
 * @return double 计算出的实际直线推力，单位：牛顿 (N)
 */
double CalculateThrustForce(short raw_torque)
{
    // 1. Delta ECMA-CA1330S4 伺服电机的硬件额定参数 (双精度)
    double T_rated = 9.55; // 额定扭矩 9.55 N·m

    // 2. 将 16 位有符号整型的千分比反馈安全转换为 double，再计算实际轴端扭矩 (Nm)
    double actual_torque_Nm = ((double)raw_torque / 1000.0) * T_rated;

    // 3. 机械传动参数定义 (双精度)
    double lead_m = 10.0 / 1000.0;          // 滚珠丝杠导程 10mm -> 0.01m
    double reduction_ratio = 1.0;           // 带传动减速比 1:1
    double efficiency = 0.9;                // 机械系统综合传动效率 90%
    double pi = 3.14159265358979323846;     // 高精度圆周率

    // 4. 根据丝杠传动物理公式计算直线推力 F = (T * 2 * pi * i * η) / Ph
    double thrust_force_N = (actual_torque_Nm * 2.0 * pi * reduction_ratio * efficiency) / lead_m;

    return thrust_force_N;
}