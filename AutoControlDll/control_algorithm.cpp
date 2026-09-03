//#pragma execution_character_set("utf-8")
#include "control_algorithm.h"
#include "qdebug.h"
#include <qmath.h>
#include "structs.h"
#include <fstream>
#include <sstream> 
#include "NeedleMark.h"

//BYTE* g_bufFrame;				//制作模版图像数据帧的全局变量
//tSdkFrameHead* g_bufFrameHead;	//制作模版图像帧头的全局变量
BYTE* g_bufFrame = NULL;				//制作模版图像数据帧的全局变量
tSdkFrameHead* g_bufFrameHead = NULL;	//制作模版图像帧头的全局变量
control_algorithm::control_algorithm(std::atomic<bool>* stopState) {
	sState = stopState;

};
control_algorithm::~control_algorithm() {

};

int control_algorithm::level_1(RF_probe_control* rf_probe_control_2, int dir, double angle) {

	angle = dir * angle;
	double vec = 2;// 微米每秒
	angle = rf_probe_control_2->displacement_to_pulse(angle);
	vec = rf_probe_control_2->displacement_to_pulse(vec * 1e-3);
	rf_probe_control_2->Trap_model("1000", { vec }, { angle });
	emit level_1Finished(0);
	return 0;
}
// 基于图像识别调平方向
int control_algorithm::directionImage(QString* dir) {
	*dir = "正向";
	emit directionImageFinished(0,dir);
	return 0;
}
// 基于电阻值计算调平角度
int control_algorithm::angleResistance(double* an) {

	// 角度计算，利用公式sin⁡φ=(ν⋅Δt)/L
	double v = 0;      // ν
	double delta_t = 0; // Δt
	double L = 0;


	double value = (v * delta_t) / L;

	// 检查是否合法（asin定义域 [-1, 1]）
	if (value < -1.0 || value > 1.0) {
		return 1;
	}

	double phi_rad = asin(value);  // 结果为弧度
	double phi_deg = phi_rad * 180.0 / M_PI;  // 转换为角度
	*an = phi_deg;
	emit angleResistanceFinished(0,an);
	return 0;
}

int control_algorithm::autoInitMove(parseData* pData, RF_probe_control* rf_probe_control_2,
	NeedleMark* NM, needleMarkLevelParas configs)
{
	
	// —— 定义内部 lambda：移动直到条件满足 ——
	auto moveUntil = [&](double v_mm, auto condition, int timeoutMs) -> int {
		double pulse = rf_probe_control_2->displacement_to_pulse(v_mm * 1e-3);
		rf_probe_control_2->JOG_model("0100", { pulse });

		auto t0 = std::chrono::steady_clock::now();
		double old_P = 0;
		double dP = 0;
		int threshold = 0;
		while (true) {
			if (*sState) {
				rf_probe_control_2->Stop("1111", "0000");
				return 1;  // 用户中止
			}
			auto data = pData->parseDatas.load(std::memory_order_acquire);
			dP = std::abs(data.p - old_P);
			old_P = data.p;
			// 在压力非零的状态下，变化率为0，则说明压力值异常了
			if (dP == 0 && old_P != 0 ) {
				threshold++;
				if (threshold >= 1000) {
					threshold = 0;
					rf_probe_control_2->Stop("1111", "0000");
					return 3;  // 卡住
				}
				
			}
			if (condition(data)) {
				rf_probe_control_2->Stop("1111", "0000");
				Sleep(100);   // 稳定时间
				return 0;     // 成功
			}

			auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::steady_clock::now() - t0).count();
			if (elapsedMs > timeoutMs) {
				rf_probe_control_2->Stop("1111", "0000");
				return 2;     // 超时
			}
			Sleep(POLL_MS);
		}
	};

	// —— 定义内部 lambda：取 Z 平均值（替代原 averageZReading） ——
	auto averageZ = [&]() -> double {
		const int samples = 5;   // 采样次数，可根据需要调整
		double sum = 0;
		for (int i = 0; i < samples; ++i) {
			auto data = pData->parseDatas.load(std::memory_order_acquire);
			sum += data.z;
			Sleep(POLL_MS);
		}
		return sum / samples;
		};

	// —— 条件判断 lambda ——
	auto isContact = [](const auto& d) { return d.p > CONTACT_TH; };
	auto isRelease = [](const auto& d) { return d.p <= RELEASE_TH; };

	// —— 错误报告 lambda（原 reportAndFail） ——
	auto reportAndFail = [this](int ret) -> int {
		switch (ret) {
		case 1: emit autoInitStatus(12); break;
		case 2: emit autoInitStatus(15); break;
		case 3: emit autoInitStatus(17); break;
		default: break;
		}
		return 1;
		};

	// 开始自动初始化流程
	emit autoInitStatus(7);
	/*
	# 原始方案
	v = -100;
	v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
	rf_probe_control_2->JOG_model("0100", { v });
	while (true)
	{
		if (*sState) {
			emit autoInitStatus(12);
			return 1;
			break;  // 安全退出循环
		}
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
		if (parseDatasFinished.p > 0)
		{
			rf_probe_control_2->Stop("1111", "0000");
			break;
		}
	}
	v = 50;
	v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
	rf_probe_control_2->JOG_model("0100", { v });
	while (true)
	{
		if (*sState) {
			emit autoInitStatus(12);
			return 1;
			break;  // 安全退出循环
		}
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
		if (parseDatasFinished.p <= 0)
		{
			Sleep(100);
			rf_probe_control_2->Stop("1111", "0000");
			break;
		}
	}

	v = -10;
	v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
	rf_probe_control_2->JOG_model("0100", { v });
	while (true)
	{
		if (*sState) {
			emit autoInitStatus(12);
			return 1;
			break;  // 安全退出循环
		}
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
		if (parseDatasFinished.p > 0)
		{
			Sleep(100);
			rf_probe_control_2->Stop("1111", "0000");
			break;
		}
	}
	v = 10;
	v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
	rf_probe_control_2->JOG_model("0100", { v });
	while (true)
	{
		if (*sState) {
			emit autoInitStatus(12);
			return 1;
			break;  // 安全退出循环
		}
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
		if (parseDatasFinished.p <= 0)
		{
			Sleep(100);
			rf_probe_control_2->Stop("1111", "0000");
			break;
		}
	}

	double old_z0 = 0;
	parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
	old_z0 = parseDatasFinished.z;
	double new_z0 = 0;
	//微调
	for (int i = 0;i < 3;i++) {
		if (*sState) {
			emit autoInitStatus(12);
			return 1;
			break;  // 安全退出循环
		}
		v = -5;
		v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
		rf_probe_control_2->JOG_model("0100", { v });
		while (true)
		{
			if (*sState) {
				emit autoInitStatus(12);
				return 1;
				break;  // 安全退出循环
			}
			parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
			if (parseDatasFinished.p > 0)
			{
				Sleep(100);
				rf_probe_control_2->Stop("1111", "0000");
				break;
			}
		}
		v = 1;
		v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
		rf_probe_control_2->JOG_model("0100", { v });
		while (true)
		{
			if (*sState) {
				emit autoInitStatus(12);
				return 1;
				break;  // 安全退出循环
			}
			parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
			if (parseDatasFinished.p <= 0)
			{
				Sleep(100);
				rf_probe_control_2->Stop("1111", "0000");
				break;
			}
		}
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
		new_z0 = parseDatasFinished.z;
		if (abs(new_z0-old_z0)<=3) {
			break;
		}
		else {
			old_z0 = new_z0;
		}
	};
	*/

	int ret;
	int timeoutMs = 10000;
	// 1) 快速下降找接触
	ret = moveUntil(-100, isContact, timeoutMs);
	if (ret != 0) return reportAndFail(ret);
	timeoutMs = 3000;
	// 2) 回退脱离
	ret = moveUntil(50, isRelease, timeoutMs);
	if (ret != 0) return reportAndFail(ret);
	// 3) 慢速下降精确接触
	ret = moveUntil(-10, isContact, timeoutMs);
	if (ret != 0) return reportAndFail(ret);
	// 4) 慢速回退脱离
	ret = moveUntil(10, isRelease, timeoutMs);
	if (ret != 0) return reportAndFail(ret);
	double old_z0 = averageZ();   // 采样平均
	

	// 5) 微调收敛（最多 3 次）
	timeoutMs = 4000;
	for (int i = 0; i < 3; ++i) {

		ret = moveUntil(-5, isContact, timeoutMs);
		qDebug() << "A:" << i;
		if (ret != 0) return reportAndFail(ret);

		ret = moveUntil(1, isRelease, timeoutMs);
		qDebug() << "B:" << i;
		if (ret != 0) return reportAndFail(ret);

		double new_z0 = averageZ();
		if (std::abs(new_z0 - old_z0) <= 3) {
			break;   // 已收敛
		}
		else if (i > 3) {
			emit autoInitStatus(18);
			return 1;
		}
		old_z0 = new_z0;
	}
	Sleep(500);
	auto data = pData->parseDatas.load(std::memory_order_acquire);
	if (!isRelease(data)) {
		emit autoInitStatus(16);
		return 1;
	}
	emit autoInitStatus(8);

	

	// —— 拍照及旋转等后续操作 ——
	Mat img1, img2;
	double v = 100;
	double time = configs.safeHeight / v * 1000;
	v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
	auto parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
	double finalP_p = rf_probe_control_2->displacement_to_pulse(parseDatasFinished.z + configs.safeHeight);
	int iRes = rf_probe_control_2->Trap_model("0100", { v }, { finalP_p });
	Sleep(time + 500);
	rf_probe_control_2->Stop("1111", "0000");
	Sleep(500);
	data = pData->parseDatas.load(std::memory_order_acquire);
	if (!isRelease(data)) {
		emit autoInitStatus(19);
	}

	if (configs.initStep == 1) {
		emit autoInitStatus(13);
		return 1;
	}

	// 第一张图片
	QString path = "autocontrol/photos/needleMark/needleMark_1.bmp";
	emit getOneImage(path);
	Sleep(200);
	const string path_str = "../data/" + path.toStdString();
	img1 = imread(path_str);
	if (img1.empty()) {
		emit autoInitStatus(10);
		return 1;
	}

	// 旋转到第二个角度
	v = 1000;
	time = configs.InitAngle / v * 1000;
	v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
	parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
	finalP_p = rf_probe_control_2->displacement_to_pulse(parseDatasFinished.u + configs.InitAngle);
	iRes = rf_probe_control_2->Trap_model("1000", { v }, { finalP_p });
	Sleep(time + 500);
	rf_probe_control_2->Stop("1111", "0000");

	// 第二张图片
	QString path_2 = "autocontrol/photos/needleMark/needleMark_2.bmp";
	emit getOneImage(path_2);
	Sleep(200);
	const string path_str_2 = "../data/" + path_2.toStdString();
	img2 = imread(path_str_2);
	if (img2.empty()) {
		emit autoInitStatus(11);
		return 1;
	}

	// 回旋归位
	v = 1000;
	time = configs.InitAngle / v * 1000;
	parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
	finalP_p = rf_probe_control_2->displacement_to_pulse(parseDatasFinished.u - configs.InitAngle);
	iRes = rf_probe_control_2->Trap_model("1000", { v }, { finalP_p });
	Sleep(time + 500);
	rf_probe_control_2->Stop("1111", "0000");

	emit autoInitStatus(9);
	if (img1.empty() || img2.empty()) {
		emit autoInitStatus(1);
		return 1;
	}
	iRes = NM->autoInit(configs, img1, img2);

	emit autoInitStatus(iRes);
	return 0;
}
int control_algorithm::InitProbe(NeedleMark* NM, needleMarkLevelParas configs) 
{
	int iRes = 0;
	Mat img1;
	Mat img2;
	// 第一张图片
	QString path = "autocontrol/photos/needleMark/needleMark_1.bmp";
	const string path_str = "../data/" + path.toStdString();
	//img1 = imread(path_str);
	if (img1.empty()) {
		emit autoInitStatus(10);
		return 1;
	}

	// 第二张图片
	QString path_2 = "autocontrol/photos/needleMark/needleMark_2.bmp";
	const string path_str_2 = "../data/" + path_2.toStdString();
	//img2 = imread(path_str_2);
	if (img2.empty()) {
		emit autoInitStatus(11);
		return 1;
	}

	emit autoInitStatus(9);
	if (img1.empty() || img2.empty()) {
		emit autoInitStatus(1);
		return 1;
	}
	iRes = NM->autoInit(configs, img1, img2);

	emit autoInitStatus(iRes);
	return 0;
}
int control_algorithm::level_2(parseData* pData, RF_probe_control* rf_probe_control_2, 
	NeedleMark* NM, needleMarkLevelParas configs) {
	

	int iRes = 0;
	ParseDatas parseDatasFinished;
	double currentP  = 0 ;
	double v = 0;
	double finalP = 0;
	double finalP_p= 0;
	double time = 0;
	double all_height = configs.depth + configs.safeHeight;

	int timeout = 3000; // 3秒
	int elapsed = 0;

	// 循环最多max次
	for (int i = 0; i < configs.maxCycles; i++) {
		// 2. 设置下压深度，运动参数配置下压直至停止
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
		currentP = parseDatasFinished.z;
		finalP = currentP - all_height;
		v = configs.downSpeed;
		time = all_height / v * 1000;
		v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
		finalP_p = rf_probe_control_2->displacement_to_pulse(finalP);
		rf_probe_control_2->Trap_model("0100", { v }, { finalP_p });
		// 延时等待
		timeout = time + 500;
		elapsed = 0;
		while (elapsed < timeout)
		{
			if (*sState) {
				emit level_2Finished(12);
				return 1;
			}
			QThread::msleep(10);  // 等10ms
			elapsed += 10;
		}
		rf_probe_control_2->Stop("0100", "0000");
		emit level_2Finished(5);
		
		// 停留时间
		// 延时等待
		timeout = 3000; // 3秒
		elapsed = 0;	
		while (elapsed < timeout)
		{
			if (*sState) {
				emit level_2Finished(12);
				return 1;
			}
			QThread::msleep(10);  // 等10ms
			elapsed += 10;
		}

		//3. 抬针
		v = configs.Z_UPSpeed;
		time = all_height / v * 1000;
		v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);
		finalP = parseDatasFinished.z + all_height;
		finalP_p = rf_probe_control_2->displacement_to_pulse(finalP);
		rf_probe_control_2->Trap_model("0100", { v },{ finalP_p });
		// 延时等待
		timeout = time + 500;
		elapsed = 0;
		while (elapsed < timeout)
		{
			if (*sState) {
				emit level_2Finished(12);
				return 1;
			}
			QThread::msleep(10);  // 等10ms
			elapsed += 10;
		}
		rf_probe_control_2->Stop("0100", "0000");
		emit level_2Finished(6);


		// 3.5 后撤
		currentP = parseDatasFinished.y;
		finalP = currentP + configs.Y_Disp;
		finalP_p = rf_probe_control_2->displacement_to_pulse(finalP);
		v = configs.Y_Speed;
		time = configs.Y_Disp / v * 1000;
		v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
		rf_probe_control_2->Trap_model("0010", { v }, { finalP_p });
		// 延时等待
		timeout = time + 500;
		elapsed = 0;
		while (elapsed < timeout)
		{
			if (*sState) {
				emit level_2Finished(12);
				return 1;
			}
			QThread::msleep(10);  // 等10ms
			elapsed += 10;
		}
		rf_probe_control_2->Stop("1111", "0000");
		emit level_2Finished(7);


		//4. 截图分析图像
		/*
		QString path = "../../data/autocontrol/photos/needleMark/needleMark.bmp";
		std::string stdPath = path.toStdString();
		const char* cstr = stdPath.c_str();
		iRes = Capture->captureImage(cstr);
		if (iRes != 0)
		{
			emit level_2Finished(11);
			return 1;
		}
		*/
		Mat scrImage;
		QString path = "autocontrol/photos/needleMark/needleMark.bmp";
		emit getOneImage(path);
		Sleep(200);
		const string path_str = "../data/" + path.toStdString();
		scrImage = imread(path_str);
		if (scrImage.empty()) {
			emit autoInitStatus(11);
			return 1;
		}

		// 分析计算
		double angle = 0;
		double dx = 0;
		iRes = NM->autoAngle(configs, scrImage, angle, dx);
		if (iRes != 0)
		{
			emit level_2Finished(iRes);
			Sleep(200);
			emit level_2Finished(20 + i);
			continue;
			//return 1;
		}
		emit level_2Finished(8);
		// 5. 循环下压检测调平角度是否达标
		if (abs(angle) <= configs.ratio) {
			emit level_2Finished(0);
			return 0;
		} 
		// 6. 调整角度
		double currentU = parseDatasFinished.u;
		double currentX = parseDatasFinished.x;
		v = 10;// 微米每秒
		time = max(dx, angle) / v * 1000;
		dx = 1;
		dx = rf_probe_control_2->displacement_to_pulse(currentX +dx);
		angle = rf_probe_control_2->displacement_to_pulse(currentU+angle);
		v = rf_probe_control_2->displacement_to_pulse(v * 1e-3);
		rf_probe_control_2->Trap_model("1001", { v,v }, { dx,angle });
		Sleep(time + 500);
		rf_probe_control_2->Stop("1111", "0000");
		emit level_2Finished(9);
		if (*sState) {
			emit level_2Finished(12);
			return 1;
		}
		emit level_2Finished(21+i);
	}
	emit level_2Finished(1);
	return 1;
}
// 单轴下压方案
int control_algorithm::pressDown_1(parseData* pData,double pressureThresholdvalue, RF_probe_control* rf_probe_control_2) {
	
	// 运动逻辑
	double angle = 0;
	double vec = -0.01;
	angle = rf_probe_control_2->displacement_to_pulse(angle);
	vec = rf_probe_control_2->displacement_to_pulse(vec * 1e-3);
	rf_probe_control_2->JOG_model("0100", { vec });

	ParseDatas parseDatasFinished;

	while (1)
	{
		if (*sState) {
			break;  // 安全退出循环
		}
		parseDatasFinished = pData->parseDatas.load(std::memory_order_acquire);

			// 停止逻辑
		if (parseDatasFinished.p> pressureThresholdvalue) {

			rf_probe_control_2->Stop("0100","0000");
			emit pressDown_1Finished(0);
			return 0;
			break;
		}
	}
	emit pressDown_1Finished(1);
	return 1;
}
// 双轴下压方案
int control_algorithm::pressDown_2(parseData* pData, double pressureThresholdvalue, RF_probe_control* rf_probe_control_2) {

	// 0. 生成轨迹		
	std::vector<TrajPoint> trajPoints;
	
	// 1. 在未接触阶段，先直线下压
	int iRes = 0;
	double v = rf_probe_control_2->displacement_to_pulse(-0.1 * 1e-3);
	iRes = rf_probe_control_2->JOG_model("0100", { v });
	// Sleep(1000);
	// iRes = rf_probe_control_2->YZTrapLn(trajPoints);
	// 2. 在刚有接触力时，插补运动
	ParseDatas parseDatasFinisned;
	while (1) {
		if (*sState) {
			// break;

			return 0;  // 安全退出循环
		}
		parseDatasFinisned = pData->parseDatas.load(std::memory_order_acquire);
		if (parseDatasFinisned.p > 0) {
			
			rf_probe_control_2->Stop("1111", "0000");

			// 生成轨迹
			// 测试轨迹
			double currentY = parseDatasFinisned.y;
			double currentZ = parseDatasFinisned.z;
			double y, z, v;
			double y_dis, z_dis, v_dis;

			/*y = rf_probe_control_2->displacement_to_pulse(currentY+0);
			z = rf_probe_control_2->displacement_to_pulse(currentZ+0);
			v = rf_probe_control_2->displacement_to_pulse(0.01*1e-3);
			trajPoints.push_back({y,z,v });
			y = rf_probe_control_2->displacement_to_pulse(currentY+0.05);
			z = rf_probe_control_2->displacement_to_pulse(currentZ-0.02);
			v = rf_probe_control_2->displacement_to_pulse(0.01 * 1e-3);
			trajPoints.push_back({ y,z,v });*/

			// 加载txt
			std::string filepath = "trajs/traj_arc.txt";
			std::ifstream ifs(filepath);
			if (!ifs.is_open())
				throw std::runtime_error("Cannot open file: " + filepath);
			std::string line;
			int line_no = 0;
			while (std::getline(ifs, line)) {
				++line_no;
				// Skip blank lines and comment lines
				if (line.empty() || line[0] == '#')
					continue;

				std::istringstream ss(line);
				if (!(ss >> y_dis >> z_dis >> v_dis )) {
					std::cerr << "[WARN] " << filepath
						<< "  line " << line_no << ": parse error, skipped.\n";
					continue;
				}
				y = rf_probe_control_2->displacement_to_pulse(currentY + y_dis);
				z = rf_probe_control_2->displacement_to_pulse(currentZ - z_dis);
				v = rf_probe_control_2->displacement_to_pulse(v_dis * 1e-3);
				trajPoints.push_back({ y,z,v });
			}
			rf_probe_control_2->YZTrapLn(trajPoints);
			break;
		}
	}
	// 停止逻辑
	int i = 0;
	while (1)
	{
		if (*sState) {
			emit pressDown_2Finished(1);
			return 1;
			break;  // 安全退出循环
		}
		parseDatasFinisned = pData->parseDatas.load(std::memory_order_acquire);
		short pCrdSts = 0;
		long pSegment = 0;
		rf_probe_control_2->getCrdState(1, pCrdSts, pSegment);
		// 停止逻辑
		if (parseDatasFinisned.p > pressureThresholdvalue or pCrdSts&CRDSYS_STATUS_FIFO_FINISH_0)
		{
			rf_probe_control_2->Stop("1111", "0000");
			emit pressDown_2Finished(0);
			return 0;
			break;
		}
	}

	emit pressDown_2Finished(1);
	return 1;

}


int control_algorithm::planMove(RF_probe_control* rf_probe_control_2,QVector<QVector<double>> values, int cycles) {
	
	for (int c = 1; c <= cycles; c++) {
		for (const QVector<double>& rowData : values)
		{
			double pos1 = rf_probe_control_2->displacement_to_pulse(rowData[0]);
			double pos2 = rf_probe_control_2->displacement_to_pulse(rowData[2]);
			std::vector<double> pos = { pos1,pos2 };
			double vel1 = rf_probe_control_2->displacement_to_pulse(rowData[1] * 1e-3);
			double vel2 = rf_probe_control_2->displacement_to_pulse(rowData[3] * 1e-3);
			std::vector<double> vel = { pos1,vel2 };
			rf_probe_control_2->Trap_model("0110", vel, pos);
			double allPos[2];
			while (true)
			{
				Sleep(300);
				if (*sState) {
					emit planMoveFinished(1);
					return 1;
					break;  // 安全退出循环
				}
				rf_probe_control_2->multiCard->MC_GetPrfPos(2, allPos, 2, NULL);

				if (allPos[0] == pos[0] and allPos[1] == pos[1])
				{
					rf_probe_control_2->Stop("1111", "0000");
					break;
				}
			}
			Sleep(rowData[4] * 1e3);
			if (*sState) {
				emit planMoveFinished(1);
				return 1;
				break;  // 安全退出循环
			}
		}
		emit cycleFinished(c);
	}
	emit planMoveFinished(0);
	return 0;
}
