#pragma once
#include <cstdint>
// #include <winnt.h>
#include <windows.h>
#include <QObject>
#include <QSerialPortInfo>
#include <QSerialPort>
# include"RingBuffer.h"

//WinAPI Overlapped线程（生产者）
//↓
//环形缓冲区（无锁 / 低锁）
//↓
//Qt UI线程（定时器批量消费）
class modbus : public QObject
{
	Q_OBJECT
private:
	HANDLE hCom = INVALID_HANDLE_VALUE;

	RingBuffer* ring;

	static const int BUF_SIZE = 30;

public:
	explicit modbus(QObject* parent = nullptr);
	// ring(rb) ----  ring = rb
	modbus(RingBuffer* rb) : ring(rb) {}
	~modbus();
	// 串口通用函数
	int openSerial(const char* portName, const DWORD* baudRate,int type = 1);
	int closeSerial();
	// 计算 Modbus RTU CRC16，计算校验码，验证数据准确性
	uint16_t ModbusCRC(uint8_t* buf, int len);
	// 查找可用串口
	int researchCOM();
	int run(int SIZE);
	// 变量
	std::vector<QString> COMList;

	// 压力传感器1
	// 串口调用函数
	// 写单寄存器函数（功能码 0x06）
	bool WriteSingleRegister(HANDLE hCom, uint16_t regAddr, uint16_t value);
	// 设置主动发送间隔（1049寄存器）
	// interval_ms = 0 表示停止，范围 1~1000 表示自动发送间隔（单位ms）
	bool SetAutoSendInterval(HANDLE hCom, uint16_t interval_ms);
	// 获取寄存器响应值。
	bool SendRequest(HANDLE hCom, uint16_t startAddr, uint16_t quantity, std::vector<uint16_t>& regs);
	// 通用读寄存器函数（支持32位和16位）
	bool ReadHoldingRegisters(HANDLE hCom, uint16_t startAddr, uint16_t quantity, std::vector<uint16_t>& regs);
	int getWeight(double* actualWeight);

	// 压力传感器2
	bool writeRegister(HANDLE hCom, uint16_t addr, uint32_t value);
	bool readRegister(HANDLE hCom, uint16_t addr, uint32_t& value);
	double ReadASCIIFrame(double* actualWeight);
	int zeroing();
	int full(uint32_t reference);
	int parsePressure(const std::string& line, std::string& value);
	int modelChangeFun(int type);
	int getState(uint32_t& value);

	// LCR数字电桥串口通信
	bool writeLCR(QString frame);
	bool readASCIILCR(std::string& resistance);
	int parseLCR(const std::string& line, std::string& value);

};