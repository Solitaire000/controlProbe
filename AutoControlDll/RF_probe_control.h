#pragma once
#include <QObject>
#include "MultiCardCPP.h"
#include "CirBuffer.h"
#include <array>
#include "structs.h"
#include <vector> 
#include <ranges>
#include "Windows.h"
class RF_probe_control : public QObject
{
	Q_OBJECT

public:
	explicit RF_probe_control(std::atomic<uint16_t>* overPressure,QObject* parent = nullptr);
	~RF_probe_control();

	//std::atomic<bool>* overPressure;
	std::atomic<uint16_t>* overPressureMask{ 0 };
	constexpr uint16_t PosLimitBit(uint8_t axis)
	{
		return static_cast<uint16_t>(1u << ((axis - 1) * 2));
	}
	constexpr uint16_t NegLimitBit(uint8_t axis)
	{
		return static_cast<uint16_t>(1u << ((axis - 1) * 2 + 1));
	}
	inline bool IsPosLimit(uint8_t axis)
	{
		return (overPressureMask->load(std::memory_order_relaxed) & PosLimitBit(axis)) != 0;
	}
	inline bool IsNegLimit(uint8_t axis)
	{
		return (overPressureMask->load(std::memory_order_relaxed) & NegLimitBit(axis)) != 0;
	}
	inline void SetPosLimit(uint8_t axis)
	{
		overPressureMask->fetch_or(PosLimitBit(axis), std::memory_order_relaxed);
	}
	inline void ClearPosLimit(uint8_t axis)
	{
		overPressureMask->fetch_and(static_cast<uint16_t>(~PosLimitBit(axis)), std::memory_order_relaxed);
	}
	inline void SetNegLimit(uint8_t axis)
	{
		overPressureMask->fetch_or(NegLimitBit(axis), std::memory_order_relaxed);
	}
	inline void ClearNegLimit(uint8_t axis)
	{
		overPressureMask->fetch_and(static_cast<uint16_t>(~NegLimitBit(axis)), std::memory_order_relaxed);
	}

	// 函数
	int connect_card();
	int connect_card_1();
	int disconnect_card();
	// 脉冲-位移转换

	//位移转换格式
	double displacement_to_pulse(double x) {
		return x/0.078125;
	}
	double pulse_to_displacement(double x) {
		return x*0.078125;
	}
	// 获取位移并返回
	int getAllPrfPos(CirBuffer<std::array<double, 4>>* posBuffer);
	
	// JOG运动模式
	int JOG_model(QString Mask, const std::vector<double>& vels);
	int Stop(QString Mask, QString Option);
	// 点动运动模式
	int Trap_model(QString Mask, const std::vector<double>& vels, const std::vector<double>& poss);
	int resetCard();
	int Get_abs_pos(QString Mask, double* dPrfPoss);

	// YZ轨迹插补运动
	int YZTrapLn(const std::vector<TrajPoint>& traj);
	int getCrdState(int crd, short& pCrdSts,long& pSegment);
	
	int home_all_axes(QString Mask);
	// 变量
	MultiCard* multiCard;
	double allPos[4];
	// CirBuffer<std::array<double, 4>>* posBuffer;

signals:
	void isLimit(QVector<long> sts_v);
};
