#pragma once
#include <QtWidgets/QWidget>
#include "RF_probe_control.h"
#include "modbus.h"
#include "control_algorithm.h"
#include "RingBuffer.h"
#include "parseData.h"
#include "NeedleMark.h"
#include <QDateTime>
#include <QObject>
#include <array>
#include <QSerialPortInfo>
#include "AutoControlDll_global.h"


enum class LogLevel
{
	Info,
	Success,
	Warning,
	Error
};
namespace Ui {
	class controlProbeClass;
}

class AUTOCONTROLDLL_API controlProbe : public QWidget
{
    Q_OBJECT

public:
	explicit controlProbe(QWidget* parent = nullptr);
    ~controlProbe();
	

	// 创建缓冲区，包括位移，重量，电阻数据的缓冲区
	RingBuffer* pressureBuffer;
	RingBuffer* resistanceBuffer;
	CirBuffer<std::array<double, 4>>* posBuffer;
	QMutex mutex;
	CirBuffer<std::array<double, 3>>* allData;


	RF_probe_control* rf_probe_control_2;
	modbus* weightPort;
	modbus* resistancePort;
	control_algorithm* controlAlgorithm;
	parseData* pData;
	// 相机参数结构体
	ImageParamsConfig* m_cameraParas;

	Mat resultImg;

	// 停止标志位置
	std::atomic<bool> stopState = false;
	std::atomic<bool> saveFile = false;
	//std::atomic<bool> overPressure = false;

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

	// 创建子线程
	QThread* controlThread;
	QThread* weightThread;
	QThread* algorithmThread;
	QThread* resistanceThread;
	QThread* getPosThread;
	QThread* parseThread;
	QThread* fileThread;
	QThread* NeedleMarkThread;


	QTimer* timer;				// UI更新计时器
	QTimer* timerParse;				// UI更新计时器
	NeedleMark* NM;

	

	bool contactState = true;
	bool overPressureState = true;
	bool m_logInited = false;

	// 各模块状态
	bool visioState = true;
	bool preState = false;
	bool risState = false;
	bool moveState = false;

	double globalPressureThreshold = 0;
	double recordPos[4] = {0,0,0,0};
	double zeroPressurePos = 9000;

	int updateUI();
	void printLog(const QString& message, LogLevel level);
	QString formatAxisLine(const QString& axisName, double position, const QString& extraNote);
	int groupSetEnabled(bool state);
	int OpenImg();
	void showMatOnLabel(const cv::Mat& mat, QLabel* label);

public slots:
	// 运动模块
	int on_moveConnect_clicked();
	int on_moveDisconnect_clicked();
	int on_home_clicked();
	int on_reset_clicked();
	int on_XN_clicked();
	int on_XP_clicked();
	int on_YN_clicked();
	int on_YP_clicked();
	int on_ZN_clicked();
	int on_ZP_clicked();
	int on_UN_clicked();
	int on_UP_clicked();
	int on_recordPos_clicked(QAbstractButton* button);

	int on_add_clicked();
	int on_remove_clicked();
	int on_planMove_clicked();

	// 压力模块
	int on_pressureConnect_clicked();
	int on_pressureDisconnect_clicked();
	int on_zero_clicked();
	int on_full_clicked();
	int on_modelChange_clicked();
	int on_save_clicked();
	int on_setPressureThreshold_clicked();

	// 自动化模块
	int on_stop_clicked();
	int on_clear_clicked();
	int on_startLevel_1_clicked();
	int on_direction_clicked();
	int on_angle_clicked();
	int on_startLevel_2_clicked();
	int on_startPressDown_1_clicked();
	int on_startPressDown_2_clicked();
	int on_autoInit_clicked();
	int on_InitProbe_clicked();
	int on_manualAngle_clicked();
	int on_initZero_clicked();
	int showMatchResultImg();
	int initTemplate();
	int on_probe1_clicked();
	int on_probe2_clicked();
	
	
	

private:
    Ui::controlProbeClass* ui;

signals:
	// 运动模块
	void rP(CirBuffer<std::array<double, 4>>* posBuffer);
	// void hS(QString Mask);
	void homeS();
	// 视觉模块
	
	// 压力模块
	void pS(int SIZE);
	// 电阻模块
	void rS(int SIZE);
	// 自动化模块

	void stop(QString Mask, QString Option);
	void L1(RF_probe_control* rf_probe_control_2, int dir, double angle);
	void Dir(QString* dir);
	void An(double* an);
	void startAutoInit(parseData* pData, RF_probe_control* rf_probe_control_2,
		NeedleMark* NM, needleMarkLevelParas configs);
	void startInit(NeedleMark* NM, needleMarkLevelParas configs);
	void L2(parseData* pData, RF_probe_control* rf_probe_control_2,
		NeedleMark* NM, needleMarkLevelParas configs);
	void P1(parseData* pData,double weight, RF_probe_control* rf_probe_control_2);
	// void P1(double* weight, RF_probe_control* rf_probe_control_2);
	void P2(parseData* pData, double weight, RF_probe_control* rf_probe_control_2);

	// 其他
	// 解析
	void startParse(RingBuffer* pressureBuffer, RingBuffer* resistanceBuffer, CirBuffer<std::array<double, 4>>* posBuffer);

	// planMove
	void startPlanMove(RF_probe_control* rf_probe_control_2,QVector<QVector<double>> values, int cycles);

	void XUAxis(bool state);
	void save(parseData* pData);

	void requestClose();

	void getImage(QString path);


protected:
	void showEvent(QShowEvent*) override {
		emit startParse(pressureBuffer, resistanceBuffer, posBuffer);
	}
};

