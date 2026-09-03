//#pragma execution_character_set("utf-8")
#include "RF_probe_control.h"
#include "qdebug.h"
#include <QObject>
#include <Windows.h>
#include <array>
#include "structs.h"
//#include "GAS_N.h"

#define BIT_SOFT_LIMIT_POS  4
#define BIT_SOFT_LIMIT_NEG  5

#ifndef AXIS_STATUS_ESTOP
#define AXIS_STATUS_ESTOP            (0x00000001)
#endif
#ifndef AXIS_STATUS_SV_ALARM
#define AXIS_STATUS_SV_ALARM         (0x00000002)
#endif
#ifndef AXIS_STATUS_POS_HARD_LIMIT
#define AXIS_STATUS_POS_HARD_LIMIT   (0x00000020)
#endif
#ifndef AXIS_STATUS_NEG_HARD_LIMIT
#define AXIS_STATUS_NEG_HARD_LIMIT   (0x00000040)
#endif
#ifndef AXIS_STATUS_RUNNING
#define AXIS_STATUS_RUNNING          (0x00000400)
#endif
#ifndef AXIS_STATUS_HOME_SWITCH
#define AXIS_STATUS_HOME_SWITCH      (0x00004000)
#endif

// 将脉冲位移转换，留给前端，该类只负责函数的调用，和转换函数的编写 
// U轴 顺时针为正，逆时针为负，且不建议更改
RF_probe_control::RF_probe_control(std::atomic<uint16_t>* OverPressureMask,QObject* parent)
	: QObject(parent)
{
	overPressureMask = OverPressureMask;
	multiCard = new MultiCard();
}
RF_probe_control::~RF_probe_control() {
	delete multiCard;
}
// 连接探针座
int RF_probe_control::connect_card() {
	int iRes = 0;
	multiCard->MC_StartDebugLog(0);

	// 设置PC端网址
	char PC_ip[] = "192.168.0.11";
	// 控制卡网址：192.168.0.1
	char Card_ip[] = "192.168.0.1";
	iRes = multiCard->MC_Open(1, PC_ip, 61005, Card_ip, 61005);

	// 重置，但坐标系会乱
	multiCard->MC_Reset();

	// ------------------------------------------------------------------
	// 【新增】脈冲输出方向（每个通道独立一位，bit0=轴1...bit3=轴4）
	// 0=不反向，1=反向。
	// 判断方法：JOG给一个正速度，看电机物理转动方向是否和你期望的"正方向"一致；
	// 不一致就把对应轴的位置为1。下面先按"四轴都不反向"给默认值，
	// 实测哪个轴反了再改对应位。
	// ------------------------------------------------------------------
	unsigned short stepSns = 0x0001;
	// 例：如果轴2方向反了，改成 stepSns = 0x0002; （bit1=1）
	int iRe= multiCard->MC_StepSns(stepSns);

	// ------------------------------------------------------------------
	// 【新增】限位开关触发电平（16位，每轴占2位：正限位、负限位）
	// 轴1：bit0=正限位，bit1=负限位
	// 轴2：bit2=正限位，bit3=负限位
	// 轴3：bit4=正限位，bit5=负限位
	// 轴4：bit6=正限位，bit7=负限位
	// 位值：1=低电平触发（开关为常开NO时用这个），0=高电平触发（开关为常闭NC时用这个，控制器默认即此）
	//
	// 判断方法：用手动方式触发限位开关（拨一下），用 MC_GetDiRaw(MC_LIMIT_POSITIVE,...)
	// 或 MC_GetSts 看对应硬限位状态位是否在触发瞬间变化；没变化说明电平接反了。
	//
	// 下面假设四轴的限位开关都是常开(NO)类型，全部设为低电平触发：
	// ------------------------------------------------------------------
	unsigned short lmtSns = 0x00FF; // 轴1~4，正负限位共8位全部设为低电平触发(常开)
	// 如果你的开关是常闭(NC)，对应位改成0；和控制器默认行为一致，可以不调用这个函数。
	multiCard->MC_LmtSns(lmtSns);

	// ------------------------------------------------------------------
	// 【新增】HOME（零位）开关触发电平（bit0~bit7对应轴1~8）
	// 0=HOME电平不取反，1=HOME电平取反
	//
	// 判断方法：手动触发零位开关，看 MC_GetSts 里 AXIS_STATUS_HOME_SWITCH
	// 状态位是否在触发瞬间置1；不置1就把对应位改成1。
	//
	// 下面假设四轴HOME开关都不需要取反（即开关类型和控制器默认假设一致）：
	// ------------------------------------------------------------------
	unsigned short homeSns = 0x0000;
	// 例：如果轴3的HOME开关需要取反，改成 homeSns = 0x0004; （bit2=1）
	multiCard->MC_HomeSns(homeSns);

	// ------------------------------------------------------------------
	// 【新增】驱动器报警输入电平（bit0~bit7对应轴1~8）
	// 具体0/1含义取决于你伺服驱动器报警输出是常开还是常闭，需要对照驱动器
	// 说明书确认（一般报警输出多为常闭型，即正常时导通/低电平，报警时断开/高电平）。
	//
	// 判断方法：人为触发一次驱动器报警（比如临时断开使能或制造一次报警），
	// 看 MC_GetSts 里 AXIS_STATUS_SV_ALARM 是否置1；不置1就改对应位。
	//
	// 下面先给默认不取反：
	// ------------------------------------------------------------------
	unsigned short alarmSns = 0x0000;
	multiCard->MC_AlarmSns(alarmSns);

	// ------------------------------------------------------------------
	// 四轴使能（原有代码，未改动）
	// ------------------------------------------------------------------
	multiCard->MC_AxisOn(1);
	multiCard->MC_AxisOn(2);
	multiCard->MC_AxisOn(3);
	multiCard->MC_AxisOn(4);

	// ------------------------------------------------------------------
	// 设置软限位（原有代码，未改动）
	// 回零前先给一个足够宽松的范围，避免还没碰到零位开关就先被软限位拦住；
	// 回零完成、确定实际行程后，可以再调用一次 MC_SetSoftLimit 收紧范围。
	// ------------------------------------------------------------------
	long lSoftLimPos = 10000000;
	long lSoftLimNeg = -10000000;
	multiCard->MC_SetSoftLimit(1, lSoftLimPos, lSoftLimNeg);
	multiCard->MC_SetSoftLimit(2, lSoftLimPos, lSoftLimNeg);
	multiCard->MC_SetSoftLimit(3, lSoftLimPos, lSoftLimNeg);
	multiCard->MC_SetSoftLimit(4, lSoftLimPos, lSoftLimNeg);

	// ------------------------------------------------------------------
	// 启用硬限位保护（原有代码，未改动）
	// 必须放在上面电平配置(MC_LmtSns)之后，否则一旦极性配错，
	// 硬限位可能在使能瞬间就被误判为"已触发"。
	// ------------------------------------------------------------------
	multiCard->MC_LmtsOn(1, -1);
	multiCard->MC_LmtsOn(2, -1);
	multiCard->MC_LmtsOn(3, -1);
	multiCard->MC_LmtsOn(4, -1);

	// ------------------------------------------------------------------
	// 【新增，可选】急停IO配置
	// 仅当现场确实接了急停按钮到某个通用输入IO时才需要配置；
	// 如果没有接，不要调用 MC_EStopOnOff(1)，否则该输入悬空/默认电平
	// 可能被误判为"急停已触发"，导致回零函数第0步直接判定所有轴急停中。
	//
	// 下面示例：主卡(nCardIndex=0)的X0输入(nIOIndex=0)作为急停信号，
	// 电平不取反(nEStopSns=0)，滤波时间10ms。按现场实际接线改IO索引。
	// ------------------------------------------------------------------
	// multiCard->MC_EStopSetIO(0, 0, 0, 10);
	// multiCard->MC_EStopOnOff(1); // 1=打开急停功能，0=关闭（默认关闭，不接急停就别打开）

	return iRes;
}
int RF_probe_control::connect_card_1() {
	int iRes = 0;
	// multiCard->MC_StartDebugLog(0);
	// 设置PC端网址
	char PC_ip[] = "192.168.0.11";
	// 控制卡网址：192.168.0.1
	char Card_ip[] = "192.168.0.1";
	iRes += multiCard->MC_Open(1, PC_ip, 61001, Card_ip, 61001);

	// 重置
	iRes += multiCard->MC_Reset();

	//unsigned short lmtSns = 0x00C0; // 轴1~4，正负限位共8位全部设为低电平触发(常开)
	//// 如果你的开关是常闭(NC)，对应位改成0；和控制器默认行为一致，可以不调用这个函数。
	//multiCard->MC_LmtSns(lmtSns);

	unsigned short stepSns = 0x0000;
	// 例：如果轴2方向反了，改成 stepSns = 0x0002; （bit1=1）
	multiCard->MC_StepSns(stepSns);

	// 四轴使能
	iRes += multiCard->MC_AxisOn(1);
	iRes += multiCard->MC_AxisOn(2);
	iRes += multiCard->MC_AxisOn(3);
	iRes += multiCard->MC_AxisOn(4);

	// 设置软限位
	long lSoftLimPos = 10000000;
	long lSoftLimNeg = -10000000;
	iRes += multiCard->MC_SetSoftLimit(1, lSoftLimPos, lSoftLimNeg);
	iRes += multiCard->MC_SetSoftLimit(2, lSoftLimPos, lSoftLimNeg);
	iRes += multiCard->MC_SetSoftLimit(3, lSoftLimPos, lSoftLimNeg);
	iRes += multiCard->MC_SetSoftLimit(4, lSoftLimPos, lSoftLimNeg);

	iRes += multiCard->MC_LmtsOn(1, -1);
	iRes += multiCard->MC_LmtsOn(2, -1);
	iRes += multiCard->MC_LmtsOn(3, -1);
	iRes += multiCard->MC_LmtsOn(4, -1);

	iRes += multiCard->MC_EncOn(1);
	iRes += multiCard->MC_EncOn(2);
	iRes += multiCard->MC_EncOn(3);
	iRes += multiCard->MC_EncOn(4);

	return iRes;
}
int RF_probe_control::disconnect_card() {
	int iRes = 0;
	iRes = multiCard->MC_Close();
	return 0;
}
// 获取四轴位置信息，入缓存
int RF_probe_control::getAllPrfPos(CirBuffer<std::array<double, 4>>* posBuffer) {
	int iRes = 0;
	std::array<double, 4> allPosArr;

	double encPos[4];
	long sts[4] = {};
	QVector<long> sts_v;
	while (1) {
		// 读取位移并写入环形缓存中
		iRes += multiCard->MC_GetPrfPos(1, allPos, 4, NULL);
		iRes += multiCard->MC_GetSts(1, sts, 4, NULL);
		
		
		if (iRes == 0)
		{
			allPosArr = { allPos[0],allPos[1],allPos[2],allPos[3] };
			posBuffer->write(allPosArr);
			sts_v = { sts[0],sts[1], sts[2], sts[3], };
			Sleep(100);
			emit isLimit(sts_v);
			
		}
		else
		{
			break;
		}
	}
	return iRes;
}
// Mask 采用二进制形式输入，在函数内部转换
// 设置速度运动模式
int RF_probe_control::JOG_model(QString Mask,const std::vector<double>& vels) {
	// Mask:"0101"---UZYX;
	// vels:{0,0,0,0}---XYZU;
	TJogPrm JogPrm;
	JogPrm.dSmooth = 0;
	JogPrm.dAcc = 0.5;
	JogPrm.dDec = 0.5;
	double pulse;

	// 按位指示需要启动点位运动的轴号bit0表示轴，bit1表示2轴，……
	long mask = Mask.toLong(nullptr, 2);
	long axis;
	int j = 0;
	for (int i = 0; i < 4; i++)  // long一般按32位处理
	{
		if (mask & (1 << i))
		{
			axis = i + 1;
			if (IsPosLimit(axis) and vels[j] > 0)
			{
				break;
			}
			if (IsNegLimit(axis) and vels[j] < 0)
			{
				break;
			}
			multiCard->MC_PrfJog(axis);
			multiCard->MC_SetJogPrm(axis, &JogPrm);
			multiCard->MC_SetVel(axis, vels[j]);
			j += 1;
		}
	}
	int iRes = multiCard->MC_Update(mask);
	return iRes;
}
// 停止运动
int RF_probe_control::Stop(QString Mask, QString Option) {
	long mask = Mask.toLong(nullptr, 2);
	long option = Option.toLong(nullptr, 2);
	int iRes = multiCard->MC_Stop(mask,option);
	return iRes;
}
// 设置点动运动模式
int RF_probe_control::Trap_model(QString Mask,const std::vector<double>& vels, const std::vector<double>& poss) {
	int iRes = 0;


	TTrapPrm TrapPrm{};
	TrapPrm.acc = 0.5;//设置点位运动加速度为0.5脉冲/毫秒^2
	TrapPrm.dec = 0.5;//设置点位运动减速度为0.5脉冲/毫秒^2
	TrapPrm.velStart = 0;//设置点位运动起始速度为0脉冲/毫秒
	TrapPrm.smoothTime = 0;//设置点位运动平滑时间为0
	double pulse;
	// 按位指示需要启动点位运动的轴号bit0表示轴，bit1表示2轴，……
	long mask = Mask.toLong(nullptr, 2);
	long axis;
	int j=0;
	iRes += multiCard->MC_GetPrfPos(1, allPos, 4, NULL);
	for (int i = 0; i < 4; i++)  // long一般按32位处理
	{

		if (mask & (1 << i))
		{
			axis = i + 1;
			if (IsPosLimit(axis) and poss[j] > allPos[i])
			{
				return 1;
			}
			if (IsNegLimit(axis) and poss[j] < allPos[i])
			{
				return 1;
			}
			iRes += multiCard->MC_PrfTrap(axis);
			iRes += multiCard->MC_SetTrapPrm(axis, &TrapPrm);
			iRes += multiCard->MC_SetVel(axis, vels[j]);
			iRes += multiCard->MC_SetPos(axis, poss[j]);
			j += 1;
		}
	}
	iRes += multiCard->MC_Update(mask);
	return iRes;
}

// 重置板卡
int RF_probe_control::resetCard() {
	int iRes = multiCard->MC_Reset();

	return iRes;
}
// 获取位置信息
int RF_probe_control::Get_abs_pos(QString Mask,double* dPrfPoss) {
	int iRes = 0;
	// 按位指示需要启动点位运动的轴号bit0表示轴，bit1表示2轴，……
	long mask = Mask.toLong(nullptr, 2);
	long axis;
	double dPrfPos;
	for (int i = 0; i < 4; i++)  // long一般按32位处理
	{
		if (mask & (1 << i))
		{
			axis = i + 1;
			multiCard->MC_GetPrfPos(axis, &dPrfPos, 1, NULL);
			dPrfPoss[i] = dPrfPos;
		}
		else {
			dPrfPoss[i] = NULL;
		}
	}
	return iRes;
}
// YZ轨迹插补运动,输入为脉冲值
int RF_probe_control::YZTrapLn(const std::vector<TrajPoint>& traj) {
	
	// multiCard->MC_Reset();
	int iRes = 0;

	if (*overPressureMask != 0) {
		return 1;
	}

	short crd = 1;
	short fifoindex = 1;
	TCrdPrm pCrdPrm = {
		2,
		// {0, 1, 2,0,0,0,0,0},	// 0x000000052befa542 {0, 1, 2, 0, 0, 0, 0, 0}
		// {1,0,0,2},
		{2, 3, 0,0,0,0,0,0},
		100,				// 最大合成速度
		5,			    // 最大加速度
		10,
		1,
		{0,0,0,0,0,0,0,0}
	};
	iRes = multiCard->MC_SetCrdPrm(crd, &pCrdPrm);
	// ----------------------------
	// 2. 初始化前瞻
	// ----------------------------
	TLookAheadPrm plookAheadPara = {
		200,        // 前瞻段数
		{100, 100, 0, 0, 0, 0} ,  // max speed
		{ 5, 5, 0, 0, 0, 0}  ,    // acc
		{ 2, 2, 0, 0, 0, 0 } , // step speed
		{ 1, 1, 1, 1, 1, 1 }   ,  // scale
	};
	iRes = multiCard->MC_InitLookAhead(crd, 0, &plookAheadPara);

	// 清空缓存
	iRes = multiCard->MC_CrdClear(crd, 0);
	// ----------------------------
	// 3. 下发轨迹
	// ----------------------------
	for (size_t i = 0; i < traj.size(); i++)
	{
		double y = traj[i].y;
		double z = traj[i].z;
		double vel = traj[i].vel;
		double acc = 1.0;  // 可优化
		iRes = multiCard->MC_LnXY(crd, (long)y, (long)z, vel, acc, 0, 1,0);
		
	}
	// ----------------------------
	// 4. 启动运动
	// ----------------------------
	iRes = multiCard->MC_CrdStart(crd, 0);

	return iRes;
}

int RF_probe_control::getCrdState(int crd, short& pCrdSts, long& pSegment)
{
	int iRes = multiCard->MC_CrdStatus(crd, &pCrdSts, &pSegment, 0);
	return iRes;
}

// 回零

//int RF_probe_control::home_all_axes(QString Mask)
//{
//	/*unsigned short homeSns = 0x0008;
//	multiCard->MC_HomeSns(homeSns);*/
//
//	int iRes = 0; // 按位标记失败轴：bit0~bit3 对应轴1~轴4，0=全部成功
//
//	const short  nAxisList[4] = { 1, 2, 3, 4 };
//
//	// ---- 每轴回零参数（对应 TAxisHomePrm / MC_HomeSetPrmSingle 的各字段）----
//	const short  nHomeMode[4] = { 1, 1, 1, 1 };   // 1=HOME回原点，最常用方式
//	short  nHomeDir[4] = { 0, 0, 0, 0 };   // 0=负向回零，1=正向回零；
//	// Z轴等方向不同就改对应值
//	const long   lOffset[4] = { 0, 0, 0, 0 };   // 回到零位后再走的偏移量，通常为0
//	const double dHomeRapidVel[4] = { 20, 20, 20, 50 };// 回零快移速度，脉冲/ms
//	const double dHomeLocatVel[4] = { 2,  2,  2,  2 }; // 回零定位（精确寻边）速度，脉冲/ms
//	const double dHomeIndexVel[4] = { 2,  2,  2,  2 }; // 寻找Index速度，nHomeMode=1时不使用，留默认值即可
//	const double dHomeAcc[4] = { 0.5,0.5,0.5,0.5 };// 回零加减速度，脉冲/ms/ms
//
//	const bool   bSequential = false; // false=四轴同时启动回零，效率高；
//	const long   lTimeoutMs = 15000; // 单轴回零总超时（固件内置流程通常比手写版快，
//	const long   lPollMs = 20;    // 状态轮询间隔，回零状态变化不需要像JOG轮询那么频繁
//
//	// U轴回零
//	double neg = 0;
//	double pos = 0;
//	double cen = 0;
//
//	// ---- 读轴状态字（用于急停/报警监控）----
//	auto getSts = [&](short nAxis) -> long {
//		long sts = 0;
//		multiCard->MC_GetSts(nAxis, &sts, 1, NULL);
//		return sts;
//		};
//
//	// ---- 启动单轴回零：配置参数 + MC_HomeStart ----
//	auto startHome = [&](int i) -> bool {
//		short nAxis = nAxisList[i];
//
//		// 急停/报警预检查，不满足条件不启动该轴回零
//		long sts = getSts(nAxis);
//		if (sts & AXIS_STATUS_ESTOP) return false;
//		if (sts & AXIS_STATUS_SV_ALARM) {
//			multiCard->MC_ClrSts(nAxis, 1);
//			if (getSts(nAxis) & AXIS_STATUS_SV_ALARM) return false;
//		}
//
//		int r = 0;
//		r += multiCard->MC_HomeSetPrmSingle(
//			nAxis, nHomeMode[i], nHomeDir[i], lOffset[i],
//			dHomeRapidVel[i], dHomeLocatVel[i], dHomeIndexVel[i], dHomeAcc[i]);
//		r += multiCard->MC_HomeStart(nAxis);
//		return (r == 0);
//		};
//
//	// ---- 等待单轴回零完成：轮询 MC_HomeGetSts，同时监控急停/报警 ----
//	// 返回 true=回零成功；false=失败或超时（失败/超时时内部已调用MC_HomeStop收尾）
//	auto waitHome = [&](short nAxis) -> bool {
//		for (long t = 0; t <= lTimeoutMs; t += lPollMs) {
//			long sts = getSts(nAxis);
//			if (sts & (AXIS_STATUS_SV_ALARM | AXIS_STATUS_ESTOP)) {
//				multiCard->MC_HomeStop(nAxis); // 异常中止，必须显式结束回零状态
//				return false;
//			}
//
//			unsigned short homeSts = 0;
//			long homeLocateAbsPos = 0;     // 回零定位完成时的绝对位置（诊断用，本函数不使用）
//			long zCaptureAbsPos = 0;       // Z相(Index)信号捕获时的绝对位置（仅Index模式相关）
//			long zCaptureDisToSensor = 0;  // Z相捕获点到HOME开关的距离（仅Index模式相关）
//			multiCard->MC_HomeGetSts(nAxis, &homeSts, &homeLocateAbsPos,
//				&zCaptureAbsPos, &zCaptureDisToSensor);
//			if (homeSts == 2) return true;  // 回零成功
//			// homeSts==1 回零中，继续等；homeSts==0 理论上不会在Start之后出现，
//			// 仍按"未完成"处理，继续轮询直到超时
//
//			Sleep(lPollMs);
//		}
//		// 超时：必须调用MC_HomeStop结束回零，否则该轴之后无法运动（文档明确要求）
//		multiCard->MC_HomeStop(nAxis);
//		return false;
//		};
//
//	// 按位指示需要回零的轴：bit0=轴1，bit1=轴2，bit2=轴3，bit3=轴4(U)
//	long mask = Mask.toLong(nullptr, 2);
//	if (mask == 0)
//	{
//		return 0;
//	}
//
//	if (bSequential) {
//		// 顺序模式：一轴启动并等待完成后，再启动下一轴
//		for (int i = 0; i < 4; ++i) {
//			if (!(mask & (1 << i))) continue; // 【修复】未勾选的轴直接跳过，绝不调用MC_HomeStart
//
//			short nAxis = nAxisList[i];
//			long  bit = (1L << (nAxis - 1));
//
//			if (!startHome(i)) { iRes |= bit; continue; }
//			if (!waitHome(nAxis)) { iRes |= bit; continue; }
//		}
//	}
//	else {
//		// 并行模式：四轴几乎同时启动，再统一轮询各自完成情况
//		bool started[4] = { false, false, false, false };
//		for (int i = 0; i < 4; ++i) {
//			if (!(mask & (1 << i))) continue;
//			started[i] = startHome(i);
//			if (!started[i]) iRes |= (1L << (nAxisList[i] - 1));
//		}
//
//		bool done[4] = { false, false, false, false };
//		for (long t = 0; t <= lTimeoutMs; t += lPollMs) {
//			bool allDone = true;
//			for (int i = 0; i < 4; ++i) {
//
//				if (mask & (1 << i))
//				{
//					if (!started[i] || done[i]) continue;
//
//					short nAxis = nAxisList[i];
//					long  bit = (1L << (nAxis - 1));
//					long  sts = getSts(nAxis);
//
//					if (sts & (AXIS_STATUS_SV_ALARM | AXIS_STATUS_ESTOP)) {
//						multiCard->MC_HomeStop(nAxis);
//						iRes |= bit;
//						done[i] = true;
//						continue;
//					}
//					// 触发限位
//					if (nAxis != 4) {
//					//if (1) {
//						if (sts & AXIS_STATUS_NEG_HARD_LIMIT) {
//							nHomeDir[i] = 0;
//							multiCard->MC_HomeStop(nAxis);
//							startHome(i);
//						}
//						if (sts & AXIS_STATUS_POS_HARD_LIMIT) {
//							nHomeDir[i] = 1;
//							multiCard->MC_HomeStop(nAxis);
//							int tem = startHome(i);
//						}
//					}
//					
//					unsigned short homeSts = 0;
//					long homeLocateAbsPos = 0;
//					long zCaptureAbsPos = 0;
//					long zCaptureDisToSensor = 0;
//					multiCard->MC_HomeGetSts(nAxis, &homeSts, &homeLocateAbsPos,
//						&zCaptureAbsPos, &zCaptureDisToSensor);
//					if (homeSts == 2) {
//						if (nAxis == 4) {
//							Trap_model("1000", { 40 }, { -66000 });
//						}
//						done[i] = true;
//						continue;
//					}
//
//					allDone = false; // 该轴还在回零中
//				}
//				// else: 未勾选的轴本来就没启动，不参与allDone判断，也不会被遗留在"回零中"状态
//			}
//			if (allDone) break;
//			Sleep(lPollMs);
//		}
//		for (int i = 0; i < 4; ++i) {
//			if (!(mask & (1 << i))) continue;
//			if (started[i] && !done[i]) {
//				short nAxis = nAxisList[i];
//				multiCard->MC_HomeStop(nAxis);
//				iRes |= (1L << (nAxis - 1));
//			}
//		}
//	}
//	if (iRes != 0) {
//		return iRes;
//	}
//	Sleep(3000);
//	for (int i = 0; i < 4; ++i) {
//		if (!(mask & (1 << i))) continue;
//		multiCard->MC_ZeroPos(i+1, 1);
//	}
//	return iRes;
//}
int RF_probe_control::home_all_axes(QString Mask)
{
	/*unsigned short homeSns = 0x0008;
	multiCard->MC_HomeSns(homeSns);*/

	int iRes = 0; // 按位标记失败轴：bit0~bit3 对应轴1~轴4，0=全部成功

	const short  nAxisList[4] = { 1, 2, 3, 4 };

	// ---- 每轴回零参数（对应 TAxisHomePrm / MC_HomeSetPrmSingle 的各字段）----
	const short  nHomeMode[4] = { 1, 1, 1, 1 };   // 1=HOME回原点，最常用方式
	short  nHomeDir[4] = { 0, 0, 0, 0 };   // 0=负向回零，1=正向回零；
	// Z轴等方向不同就改对应值
	const long   lOffset[4] = { 0, 0, 0, 0 };   // 回到零位后再走的偏移量，通常为0
	const double dHomeRapidVel[4] = { 20, 20, 20, 30 };// 回零快移速度，脉冲/ms
	const double dHomeLocatVel[4] = { 2,  2,  2,  2 }; // 回零定位（精确寻边）速度，脉冲/ms
	const double dHomeIndexVel[4] = { 2,  2,  2,  2 }; // 寻找Index速度，nHomeMode=1时不使用，留默认值即可
	const double dHomeAcc[4] = { 0.5,0.5,0.5,0.5 };// 回零加减速度，脉冲/ms/ms

	const long   lTimeoutMs = 15000; // 单轴回零总超时
	const long   lPollMs = 20;    // 状态轮询间隔

	// ---- 顺序：Z -> Y -> X -> U ----
	// nAxisList 下标：0=轴1(X)，1=轴2(Y)，2=轴3(Z)，3=轴4(U)
	// 按 Z、Y、X、U 排列对应下标
	const int axisOrder[4] = { 2, 1, 0, 3 };

	// ---- 读轴状态字（用于急停/报警监控）----
	auto getSts = [&](short nAxis) -> long {
		long sts = 0;
		multiCard->MC_GetSts(nAxis, &sts, 1, NULL);
		return sts;
		};

	// ---- 启动单轴回零：配置参数 + MC_HomeStart ----
	auto startHome = [&](int i) -> bool {
		short nAxis = nAxisList[i];

		// 急停/报警预检查，不满足条件不启动该轴回零
		long sts = getSts(nAxis);
		if (sts & AXIS_STATUS_ESTOP) return false;
		if (sts & AXIS_STATUS_SV_ALARM) {
			multiCard->MC_ClrSts(nAxis, 1);
			if (getSts(nAxis) & AXIS_STATUS_SV_ALARM) return false;
		}

		int r = 0;
		r += multiCard->MC_HomeSetPrmSingle(
			nAxis, nHomeMode[i], nHomeDir[i], lOffset[i],
			dHomeRapidVel[i], dHomeLocatVel[i], dHomeIndexVel[i], dHomeAcc[i]);
		r += multiCard->MC_HomeStart(nAxis);
		return (r == 0);
		};

	// ---- 单轴回零：启动 + 轮询等待完成，逻辑沿用原并行方案里对单轴的处理 ----
	// （急停/报警中止、硬限位触发时反向重试、U轴到位后触发 Trap_model、超时收尾）
	// 返回 true=回零成功；false=失败或超时
	auto runSequentialHome = [&](int i) -> bool {
		short nAxis = nAxisList[i];

		bool started = startHome(i);
		if (!started) return false;

		for (long t = 0; t <= lTimeoutMs; t += lPollMs) {
			long sts = getSts(nAxis);

			if (sts & (AXIS_STATUS_SV_ALARM | AXIS_STATUS_ESTOP)) {
				multiCard->MC_HomeStop(nAxis); // 异常中止，必须显式结束回零状态
				return false;
			}

			// 触发限位：反向重新回零（U轴不参与限位反向处理，沿用原逻辑）
			if (nAxis != 4) {
				if (sts & AXIS_STATUS_NEG_HARD_LIMIT) {
					nHomeDir[i] = 0;
					multiCard->MC_HomeStop(nAxis);
					startHome(i);
				}
				if (sts & AXIS_STATUS_POS_HARD_LIMIT) {
					nHomeDir[i] = 1;
					multiCard->MC_HomeStop(nAxis);
					startHome(i);
				}
			}

			unsigned short homeSts = 0;
			long homeLocateAbsPos = 0;     // 回零定位完成时的绝对位置（诊断用，本函数不使用）
			long zCaptureAbsPos = 0;       // Z相(Index)信号捕获时的绝对位置（仅Index模式相关）
			long zCaptureDisToSensor = 0;  // Z相捕获点到HOME开关的距离（仅Index模式相关）
			multiCard->MC_HomeGetSts(nAxis, &homeSts, &homeLocateAbsPos,
				&zCaptureAbsPos, &zCaptureDisToSensor);
			if (homeSts == 2) {
				if (nAxis == 4) {
					Trap_model("1000", { 40 }, { -66000 });
				}
				return true; // 回零成功
			}
			// homeSts==1 回零中，继续等；homeSts==0 理论上不会在Start之后出现，
			// 仍按"未完成"处理，继续轮询直到超时

			Sleep(lPollMs);
		}
		// 超时：必须调用MC_HomeStop结束回零，否则该轴之后无法运动（文档明确要求）
		multiCard->MC_HomeStop(nAxis);
		return false;
		};

	// 按位指示需要回零的轴：bit0=轴1，bit1=轴2，bit2=轴3，bit3=轴4(U)
	long mask = Mask.toLong(nullptr, 2);
	if (mask == 0)
	{
		return 0;
	}

	// ---- 顺序模式：按 Z -> Y -> X -> U 的顺序，等前一轴完成后再启动下一轴；未勾选的轴跳过 ----
	for (int k = 0; k < 4; ++k) {
		int i = axisOrder[k];
		if (!(mask & (1 << i))) continue; // 未勾选的轴直接跳过

		short nAxis = nAxisList[i];
		long  bit = (1L << (nAxis - 1));

		if (!runSequentialHome(i)) {
			iRes |= bit;
		}
	}

	if (iRes != 0) {
		return iRes;
	}
	Sleep(3000);
	for (int i = 0; i < 4; ++i) {
		if (!(mask & (1 << i))) continue;
		multiCard->MC_ZeroPos(i + 1, 1);
	}
	return iRes;
}