#pragma once
#include"structs.h"
#include <QObject>
#include<atomic>
#include <chrono>
#include"modbus.h"
#include"RingBuffer.h"
#include"CirBuffer.h"
#include <array>
#include "LowLatencyFileWriter.h"
#include <iostream>
#include <string>

class parseData  : public QObject
{
	Q_OBJECT

public:
	parseData(std::atomic<bool>* stopState);
	~parseData();

	int parseRuning(RingBuffer* pressureBuffer, RingBuffer* resistanceBuffer, CirBuffer<std::array<double, 4>>* posBuffer);

	std::string GetTimeFileName()
	{
		char buf[1024];

		time_t now = time(nullptr);

		tm tm_buf;
		#ifdef _WIN32
			localtime_s(&tm_buf, &now);
		#else
			localtime_r(&now, &tm_buf);
		#endif
		strftime(buf,
			sizeof(buf),
			"../../data/autocontrol/FRData/%Y%m%d_%H%M%S.txt",
			&tm_buf);

		return std::string(buf);
	}

	// 变量
	// double parseDatas[3];
	std::atomic<ParseDatas> parseDatas;
	QElapsedTimer globalTimer;

	// 在 parseData 类中增加成员
	//LowLatencyFileWriter fileWriter_{ GetTimeFileName() };
	LowLatencyFileWriter* fileWriter_ = nullptr;
	std::atomic<bool>* save;

public slots:
	// void parseFinished();
};
