#pragma once
#include "RF_probe_control.h"
#include "time.h"
#include "Windows.h"
#include "parseData.h"
#include "NeedleMark.h"

class control_algorithm : public QObject
{
	Q_OBJECT
public:
	// control_algorithm();
	explicit control_algorithm(std::atomic<bool>* stopState);
	~control_algorithm();

	// 调平方案
	int level_1(RF_probe_control* rf_probe_control_2, int dir, double angle);
	int directionImage(QString* dir);
	int angleResistance(double* an);
	int level_2(parseData* pData, RF_probe_control* rf_probe_control_2, 
		NeedleMark* NM, needleMarkLevelParas configs);
	int autoInitMove(parseData* pData, RF_probe_control* rf_probe_control_2,
		NeedleMark* NM, needleMarkLevelParas configs);
	int InitProbe(NeedleMark* NM, needleMarkLevelParas configs);

	// 下压方案
	int pressDown_1(parseData* pData,double pressureThresholdvalue, RF_probe_control* rf_probe_control_2);
	int pressDown_2(parseData* pData, double pressureThresholdvalue,RF_probe_control* rf_probe_control_2);

	// 定位零点函数
	// ================== 可调参数（按实际传感器噪声水平标定）==================
	static constexpr double CONTACT_TH = 0;   // 判定"已接触"的压力阈值
	static constexpr double RELEASE_TH = 0;   // 判定"已脱离"的压力阈值（低于噪声本底之上留余量，形成迟滞区间）
	static constexpr int    POLL_MS = 5;     // 轮询间隔
	static constexpr int    TIMEOUT_MS = 5000;   // 单段运动超时（探针真脱落/传感器异常时的兜底保护）
	static constexpr int    Z_AVG_SAMPLE = 2;      // z 读数取平均的采样次数
	// 父变量
	std::atomic<bool>* sState;

	// planMove
	int planMove(RF_probe_control* rf_probe_control_2,QVector<QVector<double>> values,int cycles);
signals:
	void level_1Finished(int iRes);
	void directionImageFinished(int iRes,QString* dir);
	void angleResistanceFinished(int iRes,double* angle);
	void level_2Finished(int iRes);
	void autoInitStatus(int iRes);
	void pressDown_1Finished(int iRes);
	void pressDown_2Finished(int iRes);

	void planMoveFinished(int iRes);
	void cycleFinished(int c);

	void getOneImage(QString path);

	
};


