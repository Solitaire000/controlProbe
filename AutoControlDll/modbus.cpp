//#pragma execution_character_set("utf-8")
#include <iostream>
#include <vector>
#include <windows.h>
#include <stdint.h>
#include <cmath>
#include "modbus.h"
#include "qdebug.h"
#include <regex>
#include <QSerialPortInfo>
#include <QSerialPort>

modbus::modbus(QObject* parent)
    : QObject(parent)
{
    researchCOM();
}
modbus::~modbus(){

}
int modbus::researchCOM() {

    const auto ports = QSerialPortInfo::availablePorts();
    COMList.clear();
    COMList.reserve(ports.size());

    for (const QSerialPortInfo& info : ports)
    {
        const QString portName = info.portName();
        QSerialPort serial(info);

        if (serial.open(QIODevice::ReadWrite))
        {
            COMList.push_back(portName);
            serial.close();
        }
        else
        {
            COMList.push_back(portName+"(busy)");
        }
    }
    return static_cast<int>(COMList.size());
}
int modbus::openSerial(const char* portName, const DWORD* baudRate, int type) {

    hCom = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hCom == INVALID_HANDLE_VALUE) {
        CloseHandle(hCom);
        return 3;
    }
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(dcb);
    dcb.BaudRate = *baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hCom, &dcb);

    // 设置缓冲区
    // SetupComm(hCom, 1024 * 64, 1024 * 64);
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hCom, &timeouts);

    if (type == 2) {
        return 0;
    }
    // 检验设备连接
    uint8_t request[8];
    request[0] = 0x01; // 从机地址（根据实际情况修改）
    request[1] = 0x03; // 功能码：写单寄存器，这个功能码是如何确定的
    request[2] = 0x07;
    request[3] = 0xD0;
    request[4] = 0x00;
    request[5] = 0x02;
    uint16_t crc = ModbusCRC(request, 6);
    request[6] = crc & 0xFF;        // CRC低字节
    request[7] = (crc >> 8) & 0xFF; // CRC高字节

    DWORD bytesWritten;
    if (!WriteFile(hCom, request, sizeof(request), &bytesWritten, NULL) || bytesWritten != sizeof(request)) {
        //qDebug() << "写寄存器发送失败";
        CloseHandle(hCom);
        return 1;
    }
    // 读取响应（回显写入的数据）
    uint8_t response[8];
    DWORD bytesRead;
    if (!ReadFile(hCom, response, sizeof(response), &bytesRead, NULL) || bytesRead != 8) {
        //qDebug() << "读取响应失败";
        CloseHandle(hCom);
        return 2;
    }


    return 0;
}
int modbus::closeSerial() {

    if (hCom != INVALID_HANDLE_VALUE) {
        CloseHandle(hCom);
        hCom = INVALID_HANDLE_VALUE;  // 防止误用
        return 0;
    }
    return 1;
}
// 计算 Modbus RTU CRC16，计算校验码，验证数据准确性
uint16_t modbus::ModbusCRC(uint8_t* buf, int len)
{
    uint16_t crc = 0xFFFF;

    for (int pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for (int i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
// 持续读取ASCII格式数据并缓存
int modbus::run(int SIZE) {
    // 循环读取数据，完全不处理数据，将读取的所有数据全部写入缓存区
    char buf[1024];
    DWORD bytesRead;

    while (1)
    {
        /*
        hCom：串口句柄
        buffer：接收缓冲区（char数组）
        sizeof(buffer)：最多读多少字节
        bytesRead：实际读到的字节数
        */
        
        BOOL ok = ReadFile(hCom, buf, SIZE, &bytesRead, NULL);
        if (ok)
        {
            if (bytesRead > 0)
            {   
                ring->write(buf, bytesRead); // 写入环形缓存区
            }
        }
        
    }
    return 0;
}


// 压力传感器1
// 写单寄存器函数（功能码 0x06）
bool modbus::WriteSingleRegister(HANDLE hCom, uint16_t regAddr, uint16_t value) {
    uint8_t request[8];
    request[0] = 0x01; // 从机地址（根据实际情况修改）
    request[1] = 0x06; // 功能码：写单寄存器，这个功能码是如何确定的
    request[2] = regAddr >> 8;
    request[3] = regAddr & 0xFF;
    request[4] = value >> 8;
    request[5] = value & 0xFF;

    uint16_t crc = ModbusCRC(request, 6);
    request[6] = crc & 0xFF;        // CRC低字节
    request[7] = (crc >> 8) & 0xFF; // CRC高字节

    DWORD bytesWritten;
    if (!WriteFile(hCom, request, sizeof(request), &bytesWritten, NULL) || bytesWritten != sizeof(request)) {
        //qDebug() << "写寄存器发送失败";
        return false;
    }

    // 读取响应（回显写入的数据）
    uint8_t response[8];
    DWORD bytesRead;
    if (!ReadFile(hCom, response, sizeof(response), &bytesRead, NULL) || bytesRead != 8) {
        //qDebug() << "读取响应失败";
        return false;
    }

    // 校验 CRC
    uint16_t resp_crc = (response[7] << 8) | response[6];
    uint16_t calc_crc = ModbusCRC(response, 6);
    if (resp_crc != calc_crc) {
        //qDebug() << "CRC 校验失败";
        return false;
    }

    return true;
}
// 设置主动发送间隔（1049寄存器）
// interval_ms = 0 表示停止，范围 1~1000 表示自动发送间隔（单位ms）
bool modbus::SetAutoSendInterval(HANDLE hCom, uint16_t interval_ms) {
    // 协议寄存器1049 → Modbus地址=1049-1=1048=0x0418
    uint16_t modbusAddr = 0x0418;
    return WriteSingleRegister(hCom, modbusAddr, interval_ms);
}
// 发送请求
bool modbus::SendRequest(HANDLE hCom, uint16_t startAddr, uint16_t quantity, std::vector<uint16_t>& regs) {
    // 向传感器发送请求，读取重量
    uint8_t request[8] = { 0x01, 0x03, (uint8_t)(startAddr >> 8), (uint8_t)(startAddr & 0xFF),
                           (uint8_t)(quantity >> 8), (uint8_t)(quantity & 0xFF) };
    uint16_t crc = ModbusCRC(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    DWORD bytesWritten;
    if (!WriteFile(hCom, request, sizeof(request), &bytesWritten, NULL) || bytesWritten != sizeof(request)) {
        return false;
    }

    Sleep(50);
}
// 通用读寄存器函数（支持32位和16位），包括发送请求和获取数据
bool modbus::ReadHoldingRegisters(HANDLE hCom, uint16_t startAddr, uint16_t quantity, std::vector<uint16_t>& regs) {


    // 每个寄存器 2 字节 + 地址功能字节 + 字节计数字节 + CRC2字节
    uint8_t response[256];
    DWORD bytesRead;
    if (!ReadFile(hCom, response, sizeof(response), &bytesRead, NULL) || bytesRead < 5) {
        return false;
    }

    // CRC 校验
    uint16_t resp_crc = (response[bytesRead - 1] << 8) | response[bytesRead - 2];
    uint16_t calc_crc = ModbusCRC(response, bytesRead - 2);
    if (resp_crc != calc_crc) {
        //qDebug() << "CRC failed";
        return false;
    }

    uint8_t byteCount = response[2];
    if (byteCount != quantity * 2) {
        return false;
    }

    regs.resize(quantity);
    for (int i = 0; i < quantity; i++) {
        regs[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
    }
    return true;
}
int modbus::getWeight(double* actualWeight) {

   

    std::vector<uint16_t> weightRegs;   // 数据
    std::vector<uint16_t> decimalRegs;  // 小数位
    int32_t rawWeight;
    int decimalPlaces;
    int model = 1;                      // 0: 连续获取；1：单次获取
    //  单次读取寄存器值
    if (model == 1) {

        while (1) {
            //qDebug() << "single";
        // 1. 读取重量（寄存器1 → Modbus地址0x0000，读取2个寄存器=4字节）
            SendRequest(hCom, 0x0000, 2, weightRegs);
            if (!ReadHoldingRegisters(hCom, 0x0000, 2, weightRegs)) {
                qDebug() << "failed1";
                return 1;
            }
            rawWeight = ((int32_t)weightRegs[0] << 16) | weightRegs[1];
            // 2. 读取小数点寄存器（寄存器1001 → Modbus地址1000 = 0x03E8，读取1个寄存器）
            SendRequest(hCom, 0x03E8, 2, decimalRegs);
            if (!ReadHoldingRegisters(hCom, 0x03E8, 2, decimalRegs)) {
                qDebug() << "failed2";
                return 1;
            }
            //decimalRegs存储寄存器的值，这里只读了一个寄存器
            decimalPlaces = decimalRegs[0];
            // 3. 计算实际重量
            *actualWeight = rawWeight / pow(10.0, decimalPlaces + 1);
            qDebug() << *actualWeight;
            // emit weightFinished();
        }
        
    }

    // 连续读取寄存器值
    else if (model == 0) {
        qDebug()<<"continued";
        // 配置寄存器值
        SetAutoSendInterval(hCom, 100);
        // 发送请求，数据和小数位
        // 还需要发送请求吗？待定
        SendRequest(hCom,0x0000, 2, weightRegs);
        SendRequest(hCom, 0x03E8, 2, decimalRegs);
        while (1) {
            // 1. 读取重量（寄存器1 → Modbus地址0x0000，读取2个寄存器=4字节）
            if (!ReadHoldingRegisters(hCom, 0x0000, 2, weightRegs)) {
                //qDebug() << "读取重量失败";
                return 1;
            }
            rawWeight = ((int32_t)weightRegs[0] << 16) | weightRegs[1];
            // 2. 读取小数点寄存器（寄存器1001 → Modbus地址1000 = 0x03E8，读取1个寄存器）;
            if (!ReadHoldingRegisters(hCom, 0x03E8, 2, decimalRegs)) {
                //qDebug() << "读取小数点寄存器失败";
                return 1;
            }
            //decimalRegs存储寄存器的值，这里只读了一个寄存器
            decimalPlaces = decimalRegs[0];
            // 3. 计算实际重量
            *actualWeight = rawWeight / pow(10.0, decimalPlaces+1);

            // emit weightFinished();
        }

    }
    CloseHandle(hCom);
    return 0;
}


// 压力传感器2
// 写寄存器
bool modbus::writeRegister(HANDLE hCom, uint16_t addr, uint32_t value)
{
    uint8_t frame[13];

    frame[0] = 0x01;      // 从站地址
    frame[1] = 0x10;      // 功能码
    frame[2] = addr >> 8;
    frame[3] = addr & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x02;      // 写2个寄存器(32bit)
    frame[6] = 0x04;      // 4字节数据

    frame[7] = (value >> 24) & 0xFF;
    frame[8] = (value >> 16) & 0xFF;
    frame[9] = (value >> 8) & 0xFF;
    frame[10] = value & 0xFF;

    uint16_t crc = ModbusCRC(frame, 11);
    frame[11] = crc & 0xFF;
    frame[12] = crc >> 8;

    // 寄存器操作
    DWORD written;
    return WriteFile(hCom, frame, 13, &written, NULL);
}
// 读寄存器
bool modbus::readRegister(HANDLE hCom, uint16_t addr, uint32_t& value)
{
    uint8_t frame[8];

    frame[0] = 0x01;
    frame[1] = 0x03;
    frame[2] = addr >> 8;
    frame[3] = addr & 0xFF;
    frame[4] = 0x00;
    frame[5] = 0x02;

    uint16_t crc = ModbusCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    DWORD written;
    WriteFile(hCom, frame, 8, &written, NULL);


    uint8_t recv[9];
    DWORD read;
    int iRes = ReadFile(hCom, recv, 9, &read, NULL);

    if (read < 9) return false;

    value = (recv[3] << 24) |
        (recv[4] << 16) |
        (recv[5] << 8) |
        recv[6];

    return true;
}
double modbus::ReadASCIIFrame(double* actualPressure)
{
    char ch;
    DWORD read;
    std::string data;

    while (true)
    {
        if (ReadFile(hCom, &ch, 1, &read, NULL) && read == 1)
        {
            if (ch == '=')
                break;

            data += ch;
        }
    }
    *actualPressure = std::stof(data);
    // emit weightFinished();
    return 0;

}
int modbus::zeroing() {
    //  配置代码
    // 1位 = 8字节
    uint8_t frame[13];
    frame[0] = 0x01;      // 从站地址
    frame[1] = 0x10;      // 功能码
    frame[2] = 0x0B;
    frame[3] = 0xB8;
    frame[4] = 0x00;
    frame[5] = 0x02;      // 写2个寄存器(32bit)
    frame[6] = 0x04;      // 4字节数据

    frame[7] = 0x00;
    frame[8] = 0x00;
    frame[9] = 0x00;
    frame[10] = 0x0A;

    uint16_t crc = ModbusCRC(frame, 11);
    frame[11] = crc & 0xFF;
    frame[12] = crc >> 8;
    // 寄存器操作
    DWORD written;
    bool iRes = WriteFile(hCom, frame, 13, &written, NULL);//written---校验发送的数据长度是否正确
    if (!iRes)
    {
        return -1;
    }
    else if (written != 13)
    {
        return written;
    }
    return 0;
}
int modbus::full(uint32_t reference) {
    // 校满三步骤
    // 先清零
    zeroing();

    // 写入砝码值
    //  配置代码
    uint8_t frame[13];
    frame[0] = 0x01;      // 从站地址
    frame[1] = 0x10;      // 功能码
    frame[2] = 0x0B;
    frame[3] = 0xBA;
    frame[4] = 0x00;
    frame[5] = 0x02;      // 写2个寄存器(32bit)
    frame[6] = 0x04;      // 4字节数据

    frame[7] = (uint8_t)((reference >> 24) & 0xFF);
    frame[8] = (uint8_t)((reference >> 16) & 0xFF);
    frame[9] = (uint8_t)((reference >> 8) & 0xFF);
    frame[10] = (uint8_t)((reference >> 0) & 0xFF);


    uint16_t crc = ModbusCRC(frame, 11);
    frame[11] = crc & 0xFF;
    frame[12] = crc >> 8;
    // 寄存器操作
    DWORD written;
    bool iRes = WriteFile(hCom, frame, 13, &written, NULL);

    // 发送校准指令
    frame[0] = 0x01;
    frame[1] = 0x10;
    frame[2] = 0x0B;
    frame[3] = 0xB8;
    frame[4] = 0x00;
    frame[5] = 0x02;
    frame[6] = 0x04;
    frame[7] = 0x00;
    frame[8] = 0x00;
    frame[9] = 0x00;
    frame[10] = 0x14;
    frame[11] = 0x8A;
    frame[12] = 0x42;
    iRes = WriteFile(hCom, frame, 13, &written, NULL);
    if (!iRes)
    {
        return -1;
    }
    else if (written != 13)
    {
        return written;
    }
    return 0;
}
// 该函数使用正则，必须为std::string,所以统一改为std::string
int modbus::parsePressure(const std::string& line, std::string& value) {
    // 函数参数中，引用和指针的区别
    // 当需要默认参数或者可不输入参数时，可以用指针，其他通常用引用，更加安全，引用更类似于参数变量
    static const std::regex pattern(R"(\s*(\d+(\.\d+)?))");
    // static const std::regex pattern(R"([-+]?\\d*\\.?\\d+)");
    std::smatch match;
    /*qDebug() << "line:";
    for (int j = 0;j < 8;j++) {
        qDebug() << line[j];
    }*/
    value = std::stod(line);
    bool ok = std::regex_match(line, match, pattern);
    if (1)
    {
        // double dataA = std::stod(match[1].str());
        value = match[1].str();
    }
    else
    {
        std::cout << "格式错误: " << line << std::endl;
    }


    return 0;
}
int modbus::modelChangeFun(int type) {
    uint8_t frame[13];
    uint16_t crc;
    frame[0] = 0x01;
    frame[1] = 0x10;
    frame[2] = 0x00;
    frame[3] = 0x60;
    frame[4] = 0x00;
    frame[5] = 0x02;
    frame[6] = 0x04;
    frame[7] = 0x00;
    frame[8] = 0x00;
    frame[9] = 0x00;
    switch (type)
    {
    case 0:
        frame[10] = 0x00;
        break;
    case 1:
        frame[10] = 0x01;
        break;
    }
    crc = ModbusCRC(frame, 11);
    frame[11] = crc & 0xFF;
    frame[12] = crc >> 8;
    // 寄存器操作
    DWORD written;
    WriteFile(hCom, frame, 13, &written, NULL);
    return 0;
}
int modbus::getState(uint32_t& value) {
    BOOL iRes;
    uint8_t frame[8];
    uint16_t crc;
    frame[0] = 0x01;
    frame[1] = 0x03;
    frame[2] = 0x00;
    frame[3] = 0x60;
    frame[4] = 0x00;
    frame[5] = 0x02;
    crc = ModbusCRC(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    DWORD written;
    iRes = WriteFile(hCom, frame, 8, &written, NULL);
    // 读取
    uint8_t recv[9];
    DWORD read;
    iRes = ReadFile(hCom, recv, 9, &read, NULL);
    if (read < 9) return -1;

    value = (recv[3] << 24) |
        (recv[4] << 16) |
        (recv[5] << 8) |
        recv[6];
    qDebug() << recv << " " << value << endl;
    return 0;
}


// LCR数字电桥串口通信
// 写寄存器
bool modbus::writeLCR(QString frame)
{
    QByteArray arr = frame.toUtf8();
    DWORD written;
    bool iRes = WriteFile(hCom, arr.constData(), (DWORD)arr.size(), &written, NULL);
    if (iRes) {
        return 0;
    }
    else{
        return 1;
    }
}
// 读寄存器
bool modbus::readASCIILCR(std::string& resistance)
{
    char buffer[256];
    DWORD bytesRead;
    std::string recvBuffer;

    while (true)
    {
        /*
        hCom：串口句柄
        buffer：接收缓冲区（char数组）
        sizeof(buffer)：最多读多少字节
        bytesRead：实际读到的字节数
        */
        if (ReadFile(hCom, buffer, sizeof(buffer), &bytesRead, NULL))
        {
            if (bytesRead > 0)
            {
                recvBuffer.append(buffer, bytesRead);

                size_t pos;
                while ((pos = recvBuffer.find('\n')) != std::string::npos)  // 判断是否收到一帧（以 \n 结尾）
                {
                    std::string line = recvBuffer.substr(0, pos);
                    recvBuffer.erase(0, pos + 1);

                    parseLCR(line,resistance); // 解析
                }
            }
        }
    }

    return true;
}
// 数据解析
int modbus::parseLCR(const std::string& line, std::string& value)
{
    static const std::regex pattern(
        R"(^([+-]\d\.\d{5}E[+-]\d{2}),([+-]\d\.\d{5}E[+-]\d{2}),([+-]?\d+),([+-]?\d+)$)"
    );

    std::smatch match;

    if (std::regex_match(line, match, pattern))
    {
        // double dataA = std::stod(match[1].str());
        value = match[0].str();
    }
    else
    {
        std::cout << "格式错误: " << line << std::endl;
    }
    return 0;
}