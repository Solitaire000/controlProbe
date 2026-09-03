#pragma execution_character_set("utf-8")
#pragma once
#include "controlProbe.h"
#include "ui_controlProbe.h"
#include "QThread"
#include "qdebug.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QMessageBox>
#include <QTimer>
#include <array>
#include <QDialogButtonBox>
#include<QMouseEvent>


controlProbe::controlProbe(QWidget *parent)
	: QWidget(parent),										// 调用基类Qwidget的函数，设置子类父窗口，方便窗口管理 
	ui(new Ui::controlProbeClass)							// 若不定义，会报空指针的错误
{
	ui->setupUi(this);
	
	qRegisterMetaType<QVector<QVector<double>>>("QVector<QVector<double>>");
	qRegisterMetaType<QVector<QVector<long>>>("QVector<QVector<long>>");
	qRegisterMetaType<needleMarkLevelParas>("needleMarkLevelParas");
	m_logInited == false;
	printLog("系统初始化完成", LogLevel::Success); // 第一次调用，自动完成日志区初始化

	
	overPressureMask = new std::atomic<uint16_t>(0);

	// 初始化变量
	// 初始化缓冲区
	this->pressureBuffer = new RingBuffer(15);
	this->resistanceBuffer = new RingBuffer(1024);
	this->posBuffer = new CirBuffer<std::array<double, 4>>(1000);
	this->allData = new CirBuffer<std::array<double, 3>>(1000);

	this->globalPressureThreshold = ui->globalPressureThresholdBox->value();

	this->m_cameraParas = new ImageParamsConfig();



	// QThread 报错出现：不允许使用不完整的类型 "QThread"，引入头文件即可
	this->controlThread = new QThread();
	this->weightThread = new QThread();
	this->resistanceThread = new QThread();
	this->algorithmThread = new QThread();
	this->getPosThread = new QThread();
	this->parseThread = new QThread();
	this->fileThread = new QThread();
	this->NeedleMarkThread = new QThread();

	this->rf_probe_control_2 = new RF_probe_control(overPressureMask);
	this->weightPort = new modbus(pressureBuffer);
	this->resistancePort = new modbus(resistanceBuffer);
	this->controlAlgorithm = new control_algorithm(&stopState);
	// this->controlAlgorithm->Window = this;
	this->pData = new parseData(&saveFile);
	this->NM = new NeedleMark();

	
	// 将事件放入子线程中
	// 函数调用，仅当采用connect，则在子线程调用
	rf_probe_control_2->moveToThread(controlThread);
	weightPort->moveToThread(weightThread);
	resistancePort->moveToThread(resistanceThread);
	controlAlgorithm->moveToThread(algorithmThread);
	pData->moveToThread(parseThread);

	
	// 启动子线程
	this->controlThread->start();
	this->weightThread->start();
	this->resistanceThread->start();
	this->algorithmThread->start();
	this->getPosThread->start();
	this->parseThread->start();
	this->fileThread->start();


	
	// 利用信号和槽机制，实现多线程工作
	// 环形缓存
	connect(this, &controlProbe::pS, weightPort, &modbus::run);
	connect(this, &controlProbe::rS, resistancePort, &modbus::run);
	// connect(this, &controlProbe::rP, getPos, &RF_probe_control::getAllPrfPos);
	connect(this, &controlProbe::rP, rf_probe_control_2, &RF_probe_control::getAllPrfPos);

	// 与探针卡运动相关的子线程
	// connect(this, &controlProbe::hS, rf_probe_control_2, &RF_probe_control::Home);
	QList<QWidget*> widgetPos = {
		 ui->XPos,
		 ui->YPos,
		 ui->ZPos,
		 ui->UPos,
	};
	connect(rf_probe_control_2, &RF_probe_control::isLimit, this, [=](QVector<long> sts) {
		for (int i = 0;i < 4;i++) {
			if (sts[i] & AXIS_STATUS_NEG_HARD_LIMIT) {
				widgetPos[i]->setStyleSheet("background-color:#007AFF;");
			}
			else if (sts[i] & AXIS_STATUS_POS_HARD_LIMIT) {
				widgetPos[i]->setStyleSheet("background-color:#FF3B30;");
			}
			else {
				widgetPos[i]->setStyleSheet("background-color:white;");
			}
		}
		});


	// 与结合控制算法计算，相关的子线程
	// 调平方案
	connect(this, &controlProbe::L1, controlAlgorithm, &control_algorithm::level_1);
	connect(this, &controlProbe::Dir, controlAlgorithm, &control_algorithm::directionImage);
	connect(this, &controlProbe::An, controlAlgorithm, &control_algorithm::angleResistance);
	connect(this, &controlProbe::startAutoInit, controlAlgorithm, &control_algorithm::autoInitMove);
	connect(this, &controlProbe::startInit, controlAlgorithm, &control_algorithm::InitProbe);
	connect(this, &controlProbe::L2, controlAlgorithm, &control_algorithm::level_2);
	connect(this, &controlProbe::startPlanMove, controlAlgorithm, &control_algorithm::planMove);

	// 利用子线程的结束信号，更新对应的UI
	connect(controlAlgorithm, &control_algorithm::level_1Finished,
		this, [=](int iRes) {
			if (iRes == 0)printLog("电阻调平完成", LogLevel::Success);
			else printLog("电阻调平失败", LogLevel::Error); 
			groupSetEnabled(true); });
	connect(controlAlgorithm, &control_algorithm::directionImageFinished,
		this, [=](int iRes,const QString* dir) {
			ui->directionValue->setText(*dir); 
			if (iRes == 0)printLog("方向计算完成", LogLevel::Success);
			else printLog("方向计算失败", LogLevel::Error);
			groupSetEnabled(true); });
	connect(controlAlgorithm, &control_algorithm::angleResistanceFinished,
		this, [=](int iRes, const double* an) {
			ui->angleValue->setText(QString::number(*an));
			if (iRes == 0)printLog("角度计算完成", LogLevel::Success);
			else printLog("角度计算失败", LogLevel::Error);
			groupSetEnabled(true); });
	connect(controlAlgorithm, &control_algorithm::autoInitStatus,
		this, [=](int iRes) {

			switch (iRes)
			{
			case -1:printLog("坐标加载失败", LogLevel::Error); groupSetEnabled(true);  break;
			case 0:printLog("自动初始化完成", LogLevel::Success); 
				zeroPressurePos = ui->ZPos->text().toDouble();
				groupSetEnabled(true); 
				showMatOnLabel(g_TemplateImg, ui->visioLabel_2);
				break;
			case 1:printLog("图像获取失败！！！", LogLevel::Error); groupSetEnabled(true); break;
			case 2:printLog("探针信息文件打开失败，请检查！！！", LogLevel::Error); groupSetEnabled(true); break;
			case 3:printLog("转动前后照片大小不一致！！！", LogLevel::Error); groupSetEnabled(true); break;
			case 4:printLog("探针转动量过小！！！", LogLevel::Error); groupSetEnabled(true); break;
			case 5:printLog("图片亮度变化或抖动过大，请重新初始化！！！", LogLevel::Error); groupSetEnabled(true); break;
			case 6:printLog("开始定位", LogLevel::Info); break;
			case 7:printLog("定位零点", LogLevel::Info); break;
			case 8:printLog("截图备用", LogLevel::Info); break;
			case 9:printLog("初始化计算", LogLevel::Info); break;
			case 10:printLog("needleMark_1.bmp 获取失败", LogLevel::Error); groupSetEnabled(true); break;
			case 11:printLog("needleMark_2.bmp 获取失败", LogLevel::Error); groupSetEnabled(true); break;
			case 12:printLog("自动初始化停止", LogLevel::Warning); groupSetEnabled(true); break;
			case 13:printLog("零点位置初始化完成", LogLevel::Success);groupSetEnabled(true);break;
			case 14:printLog("构造失败：//ui.textEdit_Output->setText();", LogLevel::Warning); groupSetEnabled(true); break;
			case 15:printLog("找零超时", LogLevel::Error); groupSetEnabled(true); break;
			case 16:printLog("零点定位失败", LogLevel::Warning); groupSetEnabled(true); break;
			case 17:printLog("零点定位压力值异常", LogLevel::Warning); groupSetEnabled(true); break;
			case 18:printLog("零点位置多次未收敛", LogLevel::Warning); groupSetEnabled(true); break;
			case 19:printLog("安全位置零点异常，请压力手动校零！", LogLevel::Warning); break;
			default:
				break;
			}
		});
	connect(controlAlgorithm, &control_algorithm::level_2Finished,
		this, [=](int iRes) {
			switch (iRes)
			{
			case 0:printLog("针痕调平完成", LogLevel::Success);groupSetEnabled(true);  break;
			case 1:printLog("针痕调平失败", LogLevel::Error); groupSetEnabled(true);  break;
			case 2:printLog("初始化图像截图完成", LogLevel::Info);  break;
			case 3:printLog("探针初始化完成", LogLevel::Info);  break;
			case 4:printLog("未识别到1号探针的针痕！！！", LogLevel::Error); groupSetEnabled(true);  break;
			case 5:printLog("探针下压完成", LogLevel::Info);  break;
			case 6:printLog("探针抬针完成", LogLevel::Info);  break;
			case 7:printLog("探针Y后退完成", LogLevel::Info);  break;
			case 8:printLog("调平角度计算完成", LogLevel::Info); 
					// 图像显示逻辑：
					// 针痕识别结果图像-->探针识别结果图像-->原始采集图像
					if (!g_AngleOutput.empty()) {
						showMatOnLabel(g_AngleOutput , ui->visioLabel);
						}
					else {
						if (!g_tempResultImg.empty()) {
							showMatOnLabel(g_tempResultImg, ui->visioLabel);
						}
						else {
							if (!g_srcImg.empty()) {
								showMatOnLabel(g_srcImg, ui->visioLabel);
							}
							else {
								ui->visioLabel->clear();
							}
						}
					}
					break;
			case 9:printLog("探针U轴和X轴调整完成", LogLevel::Info);  break;
			case 10:printLog("探针转向赋值失败！！！", LogLevel::Info); groupSetEnabled(true);  break;
			case 11:printLog("needleMark.bmp 获取失败", LogLevel::Error); groupSetEnabled(true);  break;
			case 12:printLog("自动调平停止", LogLevel::Warning); groupSetEnabled(true); break;
			default:
				printLog("完成第 " + QString::number(iRes%20) + " 次循环", LogLevel::Warning);
				break;
			}
		});
	connect(controlAlgorithm, &control_algorithm::planMoveFinished,
		this, [=](int iRes) {
			if (iRes == 0)printLog("规划运动完成", LogLevel::Success);
			else printLog("规划运动失败", LogLevel::Error);
			groupSetEnabled(true); });
	connect(controlAlgorithm, &control_algorithm::cycleFinished,
		this, [=](int c) { ui->cycleFinished->setText(QString::number(c)); });

	connect(controlAlgorithm, &control_algorithm::getOneImage,
		this, [=](QString path) {

			 emit getImage(path);

			 // 在主函数中，设置更新controlProbe的m_cameraParas
		});


	// 下压方案
	connect(this, &controlProbe::P1, controlAlgorithm, &control_algorithm::pressDown_1);
	connect(this, &controlProbe::P2, controlAlgorithm, &control_algorithm::pressDown_2);
	// 利用子线程的结束信号，更新响应的UI
	connect(controlAlgorithm, &control_algorithm::pressDown_1Finished,
		this, [=](int iRes) {
			if (iRes == 0)printLog("单轴下压完成", LogLevel::Success);
			else printLog("单轴下压失败", LogLevel::Error);
			groupSetEnabled(true); });
	connect(controlAlgorithm, &control_algorithm::pressDown_2Finished,
		this, [=](int iRes) {
			if (iRes == 0)printLog("双轴下压完成", LogLevel::Success);
			else printLog("双轴下压失败", LogLevel::Error);
			groupSetEnabled(true); });

	

	// 线程结束，关闭对象和线程
	connect(controlThread, &QThread::finished, rf_probe_control_2, &QObject::deleteLater);
	connect(controlThread, &QThread::finished, controlThread, &QThread::deleteLater);
	// 线程结束，关闭对象和线程
	connect(weightThread, &QThread::finished, weightPort, &QObject::deleteLater);
	connect(weightThread, &QThread::finished, weightThread, &QThread::deleteLater);
	// 线程结束，关闭对象和线程
	connect(resistanceThread, &QThread::finished, resistancePort, &QObject::deleteLater);
	connect(resistanceThread, &QThread::finished, resistanceThread, &QThread::deleteLater);
	// 线程结束，关闭对象和线程
	connect(algorithmThread, &QThread::finished, controlAlgorithm, &QObject::deleteLater);
	connect(algorithmThread, &QThread::finished, algorithmThread, &QThread::deleteLater);

	

	// 定期刷新界面UI
	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &controlProbe::updateUI);
	timer->start(10); // 30ms刷新

	// 开始数据解析
	connect(this, &controlProbe::startParse, pData, &parseData::parseRuning);

	// buttonBox初始化
	ui->recordPos->button(QDialogButtonBox::Save)->setText("记录位置");
	ui->recordPos->button(QDialogButtonBox::Apply)->setText("加载位置");


	// 界面初始化
	// 运动控制模块初始化
	QList<QWidget*> widgets = {
		// 运动控制控件组
		ui->moveDisconnect,
		ui->home,
		ui->reset,
		ui->X,
		ui->Y,
		ui->Z,
		ui->U,
		ui->moveCard,
	};
	for (auto w : widgets) {
		w->setEnabled(false);
	}
	ui->moveConnect->setEnabled(true);

	// 压力模块初始化
	ui->pressureValue->setText("0");
	widgets = {
		// 运动控制控件组
		ui->pressureDisconnect,
		ui->zero,
		ui->full,
		ui->setPressureThreshold,
	};
	for (auto w : widgets) {
		w->setEnabled(false);
	}

	// 检查可用串口并显示
	ui->pressureCOM->clear();
	if (weightPort->researchCOM()) {
		for (const auto& str : this->weightPort->COMList)
		{
			ui->pressureCOM->addItem(str);
			if (str == "COM8")
			{
				ui->pressureCOM->setCurrentText(str);
			}
		}
	}
	// 自动化模块初始化
	ui->state->setReadOnly(true);
	ui->directionValue->setReadOnly(true);
	ui->angleValue->setReadOnly(true);
	
	// 探针模板加载
	initTemplate();
}

controlProbe::~controlProbe()
{
	timer->stop();
	// 安全释放指针
	delete weightPort;
	delete resistancePort;
	delete controlAlgorithm;
	delete timer;
}
// 定期更新页面
int controlProbe::updateUI() {
	
	const auto parseDatasFinisned = pData->parseDatas.load(std::memory_order_acquire);
	
	ui->pressureValue->setText(QString::number(parseDatasFinisned.p));


	// UI更新
	ui->XPos->setText(QString::number(parseDatasFinisned.x));
	ui->YPos->setText(QString::number(parseDatasFinisned.y));
	ui->ZPos->setText(QString::number(parseDatasFinisned.z));
	ui->UPos->setText(QString::number(parseDatasFinisned.u));

	// 防呆
	int iRes = 0;
	if (parseDatasFinisned.p > 0 and contactState==true)
	{
		ui->pressureValue->setStyleSheet("background-color:#007AFF;");
		contactState = false;
		SetPosLimit(1);
		SetNegLimit(1);
		SetPosLimit(2);
		SetNegLimit(2);
		SetPosLimit(4);
		SetNegLimit(4);
		//printLog("开始接触，限制XU轴正负向运动", LogLevel::Warning);

		//emit XUAxis(contactState);
	}
	else if (parseDatasFinisned.p <= 0 and contactState==false)
	{
		ui->pressureValue->setStyleSheet("background-color:white;");
		contactState = true;
		ClearPosLimit(1);
		ClearNegLimit(1);
		ClearPosLimit(2);
		ClearNegLimit(2);
		ClearPosLimit(4);
		ClearNegLimit(4);
		//printLog("探针分离，取消XYU轴正负向运动限制", LogLevel::Success);
		//emit XUAxis(contactState);
	}
	if (parseDatasFinisned.p >= this->globalPressureThreshold and overPressureState == true)
	{
		ui->pressureValue->setStyleSheet("background-color:#FF3B30;");
		SetNegLimit(3);
		overPressureState = false;
		this->on_stop_clicked();
	}
	else if (parseDatasFinisned.p < this->globalPressureThreshold and overPressureState == false)
	{
		//overPressure = false;
		ui->pressureValue->setStyleSheet("background-color:#007AFF;");
		ClearNegLimit(3);
		overPressureState = true;
		//printLog("压力正常，解除限制运动", LogLevel::Success);
	}

	return 0;
}

void controlProbe::printLog(const QString& message, LogLevel level)
{
	if (!m_logInited)
	{
		// ---- 首次调用：初始化日志区外观，只执行这一次 ----
		ui->state->setReadOnly(true);

		// 等宽字体：保证时间戳、数值列严格对齐，这是"看起来科学严谨"的关键
		QFont logFont("Consolas", 10);
		if (!QFontInfo(logFont).fixedPitch()) {
			logFont = QFont("Courier New", 10);
		}
		logFont.setStyleHint(QFont::Monospace);
		ui->state->setFont(logFont);

		// 不自动换行，保持每条日志在视觉上是完整一行（数值表格尤其需要）
		ui->state->setLineWrapMode(QTextEdit::NoWrap);

		// 深色终端风格背景，进一步强化"系统日志"的识别度；
		// 想保持和界面其它部分一致的浅色背景，删掉这4行调色板设置即可
		QPalette pal = ui->state->palette();
		pal.setColor(QPalette::Base, QColor(30, 30, 30));
		pal.setColor(QPalette::Text, QColor(220, 220, 220));
		ui->state->setPalette(pal);

		ui->state->clear();

		QString banner = QString(
			"<span style=\"color:#000000;\">"
			"======================================<br>"
			"启动时间：%1<br>"
			"======================================"
			"</span>"
		).arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"));
		ui->state->append(banner);

		m_logInited = true;
	}

	// ---- 每次调用都执行：格式化并输出一行 ----
	QString timeStr = QDateTime::currentDateTime().toString("hh:mm:ss");

	QString levelStr, color;
	switch (level) {
	case LogLevel::Success: levelStr = "SUCC"; color = "#4CAF50"; break; // 绿色
	case LogLevel::Warning: levelStr = "WARN"; color = "#FFA500"; break; // 橙色
	case LogLevel::Error:   levelStr = "ERRO"; color = "#FF5252"; break; // 红色
	case LogLevel::Info:
	default:                levelStr = "INFO"; color = "#000000"; break; // 黑色
	}

	QString line = QString(
		"<span style=\"color:#000000;\">[%1]</span> "
		"<span style=\"color:%2; font-weight:bold;\">[%3]</span> "
		"<span style=\"color:%2;\">%4</span>"
	).arg(timeStr, color, levelStr, message.toHtmlEscaped());

	ui->state->append(line);

	// 自动滚动到最新一行
	QTextCursor cursor = ui->state->textCursor();
	cursor.movePosition(QTextCursor::End);
	ui->state->setTextCursor(cursor);
	ui->state->ensureCursorVisible();
}
QString controlProbe::formatAxisLine(const QString& axisName, double position,const QString& extraNote)
{
	QString msg = QString("轴 %1  位置=%2 mm")
		.arg(axisName, -4)             // 轴名左对齐，宽度4
		.arg(position, 10, 'f', 4);    // 位置右对齐，宽度10，4位小数

	if (!extraNote.isEmpty()) {
		msg += "   " + extraNote;
	}
	return msg;
}
int controlProbe::groupSetEnabled(bool state) {
	QList<QWidget*> widgetGroup = {
		 ui->visio,
		 ui->move,
		 ui->Level,
		 ui->PressDown,
	};
	for (auto w : widgetGroup) {
		w->setEnabled(state);
	}
	return 0;
}
int controlProbe::OpenImg()
{

	QString filename = QFileDialog::getOpenFileName(this, tr("Open Image"),"../data/autocontrol/", tr("All Files(*)\n(*.jpg)\n(*.bmp)\n(*.png)"));

	if (filename == NULL)
	{
		printLog("未获取图片", LogLevel::Warning);
		return 1;
	}
	g_srcImg.release();
	DestroyWindow(g_hwndShowImg);//每次打开图片销毁上次窗口
	string ImgPath = filename.toLocal8Bit().toStdString();
	g_srcImg = imread(ImgPath);
	showMatOnLabel(g_srcImg, ui->visioLabel);
	return 0;

}
void controlProbe::showMatOnLabel(const cv::Mat& mat, QLabel* label)
{
	if (mat.empty() || label == nullptr)
		return;

	QImage image;

	// 根据通道数判断图像格式
	if (mat.type() == CV_8UC1) {
		// 灰度图
		image = QImage(mat.data, mat.cols, mat.rows,
			static_cast<int>(mat.step), QImage::Format_Grayscale8);
	}
	else if (mat.type() == CV_8UC3) {
		// OpenCV 默认是 BGR，需要转换成 RGB 给 QImage
		cv::Mat rgb;
		cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
		image = QImage(rgb.data, rgb.cols, rgb.rows,
			static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
		// .copy() 是因为 rgb 是局部变量，函数返回后内存会被释放，必须深拷贝
	}
	else if (mat.type() == CV_8UC4) {
		// 带alpha通道，比如BGRA
		cv::Mat rgba;
		cv::cvtColor(mat, rgba, cv::COLOR_BGRA2RGBA);
		image = QImage(rgba.data, rgba.cols, rgba.rows,
			static_cast<int>(rgba.step), QImage::Format_RGBA8888).copy();
	}
	else {
		qWarning("showMatOnLabel: 不支持的Mat类型 %d", mat.type());
		return;
	}

	if (mat.type() == CV_8UC1) {
		image = image.copy(); // 灰度图同样需要深拷贝，避免悬空指针
	}

	// 按照label大小缩放显示，保持宽高比，避免拉伸变形
	QPixmap pixmap = QPixmap::fromImage(image).scaled(
		label->size(),
		Qt::KeepAspectRatio,
		Qt::SmoothTransformation);

	label->setPixmap(pixmap);
}

// 运动控制模块
int controlProbe::on_moveConnect_clicked() {
	groupSetEnabled(false);
	int iRes = 0;
	// 控件组
	QList<QWidget*> widgets = {
		// 运动控制控件组
		ui->moveDisconnect,
		ui->home,
		ui->reset,
		ui->X,
		ui->Y,
		ui->Z,
		ui->U,
		ui->moveCard,

	};
	 //iRes = rf_probe_control_2->connect_card();
	iRes = rf_probe_control_2->connect_card_1();
	if (iRes == 0) {
		// 其余控件全部启用
		for (auto w : widgets) {
			w->setEnabled(true);
		}
		ui->moveConnect->setEnabled(false);
		printLog("运动控制器连接成功", LogLevel::Success);

		emit rP(posBuffer);
		moveState = true;
	}
	else
	{
		// 其余控件全部禁用
		for (auto w : widgets) {
			w->setEnabled(false);
		}
		ui->moveConnect->setEnabled(true);
		printLog("运动控制器连接失败", LogLevel::Error);
	}
	groupSetEnabled(true);
	return 0;
}
int controlProbe::on_moveDisconnect_clicked() {

	int iRes = 0;
	iRes = rf_probe_control_2->disconnect_card();
	// 控件组
	QList<QWidget*> widgets = {
		// 运动控制控件组
		ui->moveDisconnect,
		ui->home,
		ui->reset,
		ui->X,
		ui->Y,
		ui->Z,
		ui->U,
		ui->moveCard,
	};
	if (iRes == 0) {

		// 其余控件全部禁用
		for (auto w : widgets) {
			w->setEnabled(false);
		}
		ui->moveConnect->setEnabled(true);
		printLog("运动控制器断连", LogLevel::Success);
		moveState = false;
	}
	else
	{
		// 其余控件全部启用
		for (auto w : widgets) {
			w->setEnabled(true);
		}
		ui->moveConnect->setEnabled(false);
		 printLog("运动控制器断连失败", LogLevel::Error);
	}

	return 0;
}
int controlProbe::on_home_clicked() {
	groupSetEnabled(false);
	// 选择状态
	QString Mask = 0;
	if (ui->U->isChecked() == true) {
		Mask += "1";
	}
	else {
		Mask += "0";
	}
	if (ui->Z->isChecked() == true) {
		Mask += "1";
	}
	else {
		Mask += "0";
	}
	if (ui->Y->isChecked() == true) {
		Mask += "1";
	}
	else {
		Mask += "0";
	}
	if (ui->X->isChecked() == true) {
		Mask += "1";
	}
	else {
		Mask += "0";
	}

	printLog("运动控制器开始回零", LogLevel::Info);
	int iRes = 0;
	iRes += this->rf_probe_control_2->home_all_axes(Mask);
	if (iRes == 0) {
		printLog("运动控制器回零完成", LogLevel::Success);
		groupSetEnabled(true);
		return 0;
	}
	else {
		printLog("运动控制器回零失败", LogLevel::Error);
		groupSetEnabled(true);
		return 1;
	}
	
}
int controlProbe::on_reset_clicked() {
	int iRes = 0;
	QMessageBox::StandardButton reply;
	reply = QMessageBox::question(
		this,
		"确认",
		"是否确认重置板卡?重置后坐标系变化!",
		QMessageBox::Yes | QMessageBox::No
	);

	if (reply == QMessageBox::Yes)
	{
		iRes = this->rf_probe_control_2->resetCard();
		if (iRes == 0)
		{
			printLog("运动控制器重置成功", LogLevel::Success);
			return 0;
		}
			
	}
	printLog("运动控制器重置失败", LogLevel::Error);
	return 1;
}
// tableWidget 操作
int controlProbe::on_remove_clicked() {

	int rowCount = ui->moveTable->rowCount();

	if (rowCount > 0)
	{
		ui->moveTable->removeRow(rowCount - 1);
	}
	return 0;
}
int controlProbe::on_add_clicked() {
	int rowCount = ui->moveTable->rowCount();

	ui->moveTable->insertRow(rowCount);
	return 0;
}
int controlProbe::on_planMove_clicked() {
	groupSetEnabled(false);
	int rowCount = ui->moveTable->rowCount();
	int columnCount = ui->moveTable->columnCount();

	QVector<QVector<double>> values(rowCount, QVector<double>(columnCount, 0.0));
	for (int row = 0; row < rowCount; ++row)
	{
		for (int col = 0; col < columnCount; ++col)
		{
			if (const QTableWidgetItem* item = ui->moveTable->item(row, col))
			{
				values[row][col] = item->text().toDouble();
				// 单位转换
			}
		}
	}
	stopState = false;
	int cycles = ui->cycles->value();
	printLog("规划运动开始", LogLevel::Info);
	emit startPlanMove(rf_probe_control_2,values,cycles);

	return 0;
}

// 控制卡移动，在主线程中运行，可能会有问题
int controlProbe::on_XN_clicked() {

	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->XSpeed->value()*1e-3);
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->XPos->text().toDouble() - ui->XDisplacement->value());
	int iRes = this->rf_probe_control_2->Trap_model("0001", {speed}, {displacement});
	return 0;
}
int controlProbe::on_XP_clicked() {
	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->XSpeed->value() * 1e-3);
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->XPos->text().toDouble() + ui->XDisplacement->value());
	int iRes = this->rf_probe_control_2->Trap_model("0001", { speed }, { displacement });
	return 0;
}
int controlProbe::on_YN_clicked() {
	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->YSpeed->value() * 1e-3 );
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->YPos->text().toDouble() -ui->YDisplacement->value());
	int iRes = this->rf_probe_control_2->Trap_model("0010", { speed }, { displacement });
	return 0;
}
int controlProbe::on_YP_clicked() {
	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->YSpeed->value() * 1e-3);
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->YPos->text().toDouble() +ui->YDisplacement->value());
	int iRes = this->rf_probe_control_2->Trap_model("0010", { speed }, { displacement });

	return 0;
}
int controlProbe::on_ZN_clicked() {
	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->ZSpeed->value() * 1e-3 );
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->ZPos->text().toDouble() -ui->ZDisplacement->value());
	int iRes = this->rf_probe_control_2->Trap_model("0100", { speed }, { displacement });
	return 0;
}
int controlProbe::on_ZP_clicked() {
	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->ZSpeed->value() * 1e-3);
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->ZPos->text().toDouble() +ui->ZDisplacement->value());;
	int iRes = this->rf_probe_control_2->Trap_model("0100", { speed }, { displacement });
	return 0;
}
int controlProbe::on_UN_clicked() {
	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->USpeed->value() * 1e-3 );
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->UPos->text().toDouble() -ui->UDisplacement->value());
	int iRes = this->rf_probe_control_2->Trap_model("1000", { speed }, { displacement });
	return 0;
}
int controlProbe::on_UP_clicked() {
	double speed = this->rf_probe_control_2->displacement_to_pulse(ui->USpeed->value() * 1e-3);
	double displacement = this->rf_probe_control_2->displacement_to_pulse(ui->UPos->text().toDouble() +ui->UDisplacement->value());
	int iRes = this->rf_probe_control_2->Trap_model("1000", { speed }, { displacement });
	return 0;
}
int controlProbe::on_recordPos_clicked(QAbstractButton *button) {
	groupSetEnabled(false);
	int iRes = 0;
	double x = 0, y = 0, z = 0, u = 0;
	const QString filePath = "../data/autocontrol/recordPos.txt";		// 探针位置txt文档

	std::ifstream fin(filePath.toStdString());
	if (!fin.is_open())
	{
		printLog("文件加载失败", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	// 直接读取4个数字，自动忽略空格、回车、Tab
	fin >> x >> y >> z >> u;
	fin.close();

	if (ui->recordPos->standardButton(button)== QDialogButtonBox::Save) {
		QString recordPos = ui->XPos->text() + "\n" + ui->YPos->text() + "\n" + ui->ZPos->text() + "\n" + ui->UPos->text() + "\n";
		
		QFile file(filePath);
		// 读取信息
		
		// 弹窗确认
		QMessageBox::StandardButton ret = QMessageBox::question(
			this,
			"确认修改",
			QString("已记录坐标：\n X=%1\n Y=%2\n Z=%3\n U=%4\n 是否覆盖？").arg(QString::number(x), QString::number(y), QString::number(z), QString::number(u)),
			QMessageBox::Yes | QMessageBox::No,
			QMessageBox::No
		);
		if (ret != QMessageBox::Yes) {
			printLog("已取消坐标记录", LogLevel::Warning);
			groupSetEnabled(true);
			return 1;
		}
		
		// 以文本写入模式打开文件：文件不存在则自动创建，已存在则清空原有内容
		if (file.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream out(&file);
			out << recordPos << "\n"; // 写入QString内容并自动追加换行
			file.close();
			printLog("绝对位置记录成功", LogLevel::Success);
		}
		else
		{
			printLog("绝对位置记录失败", LogLevel::Error);
		}
	}
	else if (ui->recordPos->standardButton(button) == QDialogButtonBox::Apply) {

		
		double X = rf_probe_control_2->displacement_to_pulse(x);
		double Y = rf_probe_control_2->displacement_to_pulse(y);
		double Z = rf_probe_control_2->displacement_to_pulse(z);
		double U = rf_probe_control_2->displacement_to_pulse(u);
		iRes = rf_probe_control_2->Trap_model("0011", { 20,20 }, { X,Y });
		printLog("坐标加载中，等待停止", LogLevel::Success);
	}
	groupSetEnabled(true);
	return 0;
}

// 压力模块
int controlProbe::on_pressureConnect_clicked() {
	// 配置串口 打开串口
	int iRes = 0;
	int index = ui->pressureCOM->currentIndex();   // 当前选中序列号（索引）
	QString qvalue = ui->pressureCOM->itemText(index); // 获取对应序列号的值
	// 判断当前序列是否为空
	if (qvalue == NULL) {
		 printLog("压力串口打开失败", LogLevel::Error);
		 return 1;
	}
	// 将 QString 转 std::string
	std::string value = qvalue.toStdString();
	// value = "COM8";
	// 拼接串口路径
	std::string portName = "\\\\.\\";
	portName += value; // 拼接成 "\\\\.\\COM3"
	DWORD baudRate = 19200;
	iRes = weightPort->openSerial(portName.c_str(), &baudRate); // 串口
	QList<QWidget*> widgets = {
		// 压力控件组
		ui->pressureDisconnect,
		ui->zero,
		ui->full,
		ui->setPressureThreshold,
	};
	if (iRes != 0) {
		for (auto w : widgets) {
			w->setEnabled(false);
		}
		ui->pressureConnect->setEnabled(true);
		printLog("压力串口打开失败", LogLevel::Error);
		return 1;

	}
	else
	{
		for (auto w : widgets) {
			w->setEnabled(true);
		}
		ui->pressureConnect->setEnabled(false);
		printLog("压力串口打开成功", LogLevel::Success);
		preState = true;
	}
	uint32_t state;
	this->weightPort->getState(state);
	// 0-modbus-RTU,1-ascii主动上传, 2 - 16 进制主动上传
	qDebug() << "ASCII:" << state << endl;
	// 实时读取数据
	emit pS(8);
	return 0;
}
int controlProbe::on_pressureDisconnect_clicked() {
	// 关闭串口
	int iRes = 0;
	iRes = weightPort->closeSerial();
	QList<QWidget*> widgets = {
		// 压力控件组
		ui->pressureDisconnect,
		ui->zero,
		ui->full,
		ui->setPressureThreshold,
	};
	if (iRes == 0) {
		for (auto w : widgets) {
			w->setEnabled(false);
		}
		ui->pressureConnect->setEnabled(true);
		printLog("压力串口关闭成功", LogLevel::Success);
		preState = false;
	}
	else
	{
		for (auto w : widgets) {
			w->setEnabled(true);
		}
		ui->pressureConnect->setEnabled(false);
		printLog("压力串口关闭失败", LogLevel::Error);
	}

	return iRes;
}
int controlProbe::on_zero_clicked() {
	int iRes = weightPort->zeroing();
	if (iRes == 0) {
		printLog("压力校零成功", LogLevel::Success);
	}
	else
	{
		printLog("压力校零失败", LogLevel::Success);
	}
	return iRes;
}
int controlProbe::on_full_clicked() {
	uint32_t inference;
	inference = ui->fullValue->value();
	int iRes = weightPort->full(inference);
	if (iRes == 0) {
		printLog("压力校满成功", LogLevel::Success);
	}
	else
	{
		printLog("压力校满失败", LogLevel::Error);
	}
	return iRes;
}
int controlProbe::on_modelChange_clicked() {
	int type;
	if (ui->modelChange->isChecked())
	{
		type = 1;
		printLog("切换ASCII模式", LogLevel::Info);
	}
	else
	{
		type = 0;
		printLog("切换modbus模式", LogLevel::Info);
	}

	this->weightPort->modelChangeFun(type);
	QMessageBox::information(this, "Warring", "Restart!");

	return 0;
}
int controlProbe::on_save_clicked() {

	if (ui->save->isChecked())
	{
		saveFile = true;
	}
	else
	{
		saveFile = false;
	}
	return 0;
}
int controlProbe::on_setPressureThreshold_clicked() {
	this->globalPressureThreshold = ui->globalPressureThresholdBox->value();
	return 0;
}

// 自动化模块
int controlProbe::on_stop_clicked() {

	if (!moveState) {
		return 0;
	}
	stopState = true;
	int iRes = this->rf_probe_control_2->Stop("1111", "0000");
	if (iRes == 0) printLog("停止运动", LogLevel::Success);
	else printLog("停止运动失败", LogLevel::Error);
	groupSetEnabled(true);
	return iRes;
}
int controlProbe::on_clear_clicked() {

	ui->visioLabel->clear();
	ui->visioLabel_2->clear();
	return 0;
}
int controlProbe::on_startLevel_1_clicked() {
	groupSetEnabled(false);
	QString Dir = ui->directionValue->text();
	QString Angle = ui->angleValue->text();
	int dir = 0;
	double angle = 0;
	if (Dir == "正向")
	{
		dir = 1;
	}
	else if (Dir == "反向")
	{
		dir = -1;
	}
	Angle.number(angle);
	printLog("开启电阻调平", LogLevel::Info);
	emit L1(rf_probe_control_2, dir, angle);
	return 0;
}
int controlProbe::on_direction_clicked() {
	groupSetEnabled(false);
	QString dir;
	emit Dir(&dir);
	return 0;
}
int controlProbe::on_angle_clicked() {
	groupSetEnabled(false);
	double an;
	emit An(&an);
	return 0;
}
int controlProbe::on_startLevel_2_clicked() {
	
	groupSetEnabled(false);
	if (!preState or !moveState) {
		printLog("未开启必要模块", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	if (ui->ZPos->text().toDouble() > zeroPressurePos + 5 or ui->ZPos->text().toDouble() < zeroPressurePos - 5) {
		printLog("未初始化压力零点位置", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}

	printLog("开启针痕调平", LogLevel::Info);
	int probe = 1;
	if (ui->probe2->isChecked() and !ui->probe1->isChecked()) {
		probe = 2;
	}
	stopState = false;
	needleMarkLevelParas configs;
	configs.probe = probe;
	configs.safeHeight = ui->safeHeight->value();
	configs.depth = ui->depth->value();
	configs.ratio = ui->ratio->value();
	configs.maxCycles = ui->maxCycles->value();
	configs.downSpeed = ui->downSpeed->value();
	configs.Z_UPSpeed = ui->Z_UPSpeed->value();
	configs.Y_Disp = ui->Y_Disp->value();
	configs.Y_Speed = ui->Y_Speed->value();


	// 发送空路径，只更新相机参数
	emit getImage("");
	// m_cameraParas;
	configs.SlipDistance = ui->SlipDistance->value();
	configs.ProbeMark_Distance = ui->ProbeMark_Distance->value();
	configs.PixelSize = ui->PixelSize->value();
	emit L2(pData,rf_probe_control_2, NM,configs);
	return 0;
}
int controlProbe::on_autoInit_clicked() {
	groupSetEnabled(false);
	if (!ui->openAutoInit->isChecked())
	{
		printLog("未解锁", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	if (!preState or !moveState) {
		printLog("未开启必要模块", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	if (ui->pressureValue->text().toDouble() != 0) {
		printLog("压力未校零", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	printLog("开始自动初始化", LogLevel::Info);
	stopState = false;
	int probe = 1;
	if (ui->probe2->isChecked() and !ui->probe1->isChecked()) {
		probe = 2;
	}
	needleMarkLevelParas configs;
	configs.probe = probe;
	configs.InitAngle = ui->InitAngle->value();
	configs.PixelSize = ui->PixelSize->value();
	configs.safeHeight = ui->safeHeight->value();

	emit startAutoInit(pData, rf_probe_control_2, NM, configs);
	ui->openAutoInit->setChecked(false);
	return 0;
}
int controlProbe::on_InitProbe_clicked() {
	groupSetEnabled(false);
	if (!ui->openAutoInit->isChecked())
	{
		printLog("未解锁", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	printLog("开始初始化", LogLevel::Info);
	int probe = 1;
	if (ui->probe2->isChecked() and !ui->probe1->isChecked()) {
		probe = 2;
	}
	needleMarkLevelParas configs;
	configs.probe = probe;
	configs.InitAngle = ui->InitAngle->value();
	configs.PixelSize = ui->PixelSize->value();
	configs.safeHeight = ui->safeHeight->value();

	emit startInit(NM, configs);
	ui->openAutoInit->setChecked(false);
	return 0;
}
int controlProbe::on_manualAngle_clicked() {
	int iRes = 0;
	int probe = 1;
	if (ui->probe2->isChecked() and !ui->probe1->isChecked()) {
		probe = 2;
	}
	QString str = ui->manualAngle->text();//获取按键字符
	if (str == "识别探针")
	{
		if(OpenImg()!= 0){
			return 0;
		}
		auto t1 = std::chrono::high_resolution_clock::now();
		iRes = this->NM->IdentifyProbe(probe);
		auto t2 = std::chrono::high_resolution_clock::now();
		auto cost_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		qDebug() << cost_ms << endl;
		switch (iRes)
		{
		case 1: printLog("未找到模板图片！！！", LogLevel::Success); break;
		case 2: printLog("未检测到1号探针！！！", LogLevel::Success); break;
		case 3: printLog("探针模板尺寸大于待测图片！！！", LogLevel::Success);  break;
		case 4: printLog("未找到探针csv配置文件", LogLevel::Success); break;
		default:
			break;
		}
		if (iRes != 0)
			return 1;
		//imshow("ShowImg", g_tempResultImg);
		showMatOnLabel(g_tempResultImg, ui->visioLabel);
		ui->manualAngle->setText("识别针痕");
		ui->probe1->setEnabled(false);
		ui->probe2->setEnabled(false);
		if (probe == 1)
			printLog("已识别1号探针！！！", LogLevel::Success);
		else
			printLog("已识别2号探针！！！", LogLevel::Success);
		return 0;
	}
	else if (str == "识别针痕")
	{
		needleMarkLevelParas configs;
		configs.probe = probe;
		configs.SlipDistance = ui->SlipDistance->value();
		configs.ProbeMark_Distance = ui->ProbeMark_Distance->value();
		configs.PixelSize = ui->PixelSize->value();
		double dangle = 0;
		double dx = 0;

		auto t3 = std::chrono::high_resolution_clock::now();
		iRes = this->NM->IdentifyMarke(configs,dangle,dx);
		auto t4 = std::chrono::high_resolution_clock::now();
		auto cost_ms_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
		qDebug() << cost_ms_1 << endl;
		switch (iRes)
		{
		case 0: printLog("dangle = "+ QString::number(dangle)+" dx = "+ QString::number(dx), LogLevel::Success);  break;
		case 1: printLog("未找到待测图片！！！", LogLevel::Warning);  break;
		case 2: printLog("针痕异常，请重新扎针！！！", LogLevel::Warning);  break;
		case 3: printLog(QString::number(probe)+"号探针过于接近视野边缘，请远离后再试！！！", LogLevel::Warning);  break;
		case 4: printLog("未识别到探针的针痕！！！", LogLevel::Warning);  break;
		case 5: printLog("探针转向赋值失败！！！", LogLevel::Warning);  break;
		case 6: printLog("探针模板尺寸大于待测图片！！！", LogLevel::Warning);  break;
		default:
			break;
		}
		ui->manualAngle->setText("识别探针");
		ui->probe1->setEnabled(true);
		ui->probe2->setEnabled(true);
	}

	return 0;
}
int controlProbe::on_initZero_clicked() {
	groupSetEnabled(false);
	if (!preState or !moveState or !visioState) {
		printLog("未开启必要模块", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	if (ui->pressureValue->text().toDouble() != 0) {
		printLog("压力未校零", LogLevel::Warning);
		groupSetEnabled(true);
		return 1;
	}
	printLog("开始初始化零点", LogLevel::Info);
	stopState = false;
	int probe = 1;
	if (ui->probe2->isChecked() and !ui->probe1->isChecked()) {
		probe = 2;
	}
	needleMarkLevelParas configs;
	configs.probe = probe;
	configs.InitAngle = ui->InitAngle->value();
	configs.PixelSize = ui->PixelSize->value();
	configs.safeHeight = ui->safeHeight->value();
	configs.initStep = 1;
	emit startAutoInit(pData, rf_probe_control_2, NM, configs);

	return 0;
}
int controlProbe::on_startPressDown_1_clicked() {
	groupSetEnabled(false);
	if (!preState or !moveState) {
		printLog("未开启必要模块", LogLevel::Warning);
		groupSetEnabled(true);
		return -1;
	}
	double pressureThresholdvalue =  ui->pressureThreshold->value();
	printLog("开启单轴下压", LogLevel::Info);
	stopState = false;
	emit P1(pData, pressureThresholdvalue, rf_probe_control_2);

	return 0;
}
int controlProbe::on_startPressDown_2_clicked() {
	groupSetEnabled(false);
	if (!preState or !moveState) {
		printLog("未开启必要模块", LogLevel::Warning);
		groupSetEnabled(true);
		return -1;
	}
	double pressureThresholdvalue = ui->pressureThreshold_2->value();
	// 在子线程下压
	printLog("开启双轴下压", LogLevel::Info);
	stopState = false;
	emit P2(pData, pressureThresholdvalue,rf_probe_control_2);

	return 0;
}
int controlProbe::showMatchResultImg() {
	
	showMatOnLabel(this->resultImg, ui->visioLabel);
	return 0;
}
int controlProbe::initTemplate() {
	int probe = 1;
	if (ui->probe2->isChecked() and !ui->probe1->isChecked()) {
		probe = 2;
	}
	Mat probeTemImg;
	if (probe == 1)//选择探针1时读取探针1模板
	{
		probeTemImg = imread("../../data/autocontrol/photos/template/Probe_One_Temp.jpg");

	}
	else if (probe == 2)//选择探针2时读取探针2模板
	{
		probeTemImg = imread("../../data/autocontrol/photos/template/Probe_Two_Temp.jpg");

	}

	if (probeTemImg.data == NULL)
	{
		printLog("未找到模板图片", LogLevel::Error);
		return 1;
	}
	showMatOnLabel(probeTemImg, ui->visioLabel_2);
	return 0;
}
int controlProbe::on_probe1_clicked() {
	int iRes = 0;
	iRes = initTemplate();
	return iRes;
}
int controlProbe::on_probe2_clicked() {
	int iRes = 0;
	iRes = initTemplate();
	return iRes;
}
/*
* 
*	void matchFinished();
*   connect(this, &WB_GUI::matchFinished, m_embeddedControlProbe, &controlProbe::showMatchResultImg);
*	添加到主程序中
	Mat resultImg = srcImg.clone();//复制原图，避免污染原始数据

	// 防止ROIRect超出图像边界导致绘制异常
	Rect imgBound(0, 0, srcImg.cols, srcImg.rows);
	Rect safeRect = ROIRect & imgBound;//取交集，确保矩形在图像范围内

	if (safeRect.width > 0 && safeRect.height > 0)
	{
		// 绘制矩形框，颜色BGR(0,0,255)红色，线宽2
		rectangle(resultImg, safeRect, Scalar(0, 0, 255), 2, LINE_AA);

		// 可选：在矩形左上角标注文字信息
		String text = "ROI";
		Point textOrg(safeRect.x, safeRect.y - 5 > 0 ? safeRect.y - 5 : safeRect.y + 15);
		putText(resultImg, text, textOrg, FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
	}
	// return resultImg;
	m_embeddedControlProbe->resultImg = resultImg;
	emit matchFinished();
*/
	