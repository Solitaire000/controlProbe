#pragma once
#include <QtWidgets/QMainWindow>
#include "NeedleMarkDll.h"
#include <QFileDialog>
#include <QDebug>
#include<qmessagebox.h>
#include<QMouseEvent>
#include <QLabel>
#include <QPainter>
#include "QThread"
#include <opencv2/highgui/highgui_c.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <QTextStream>
#include <QXmlStreamReader>
#include "structs.h"

//迈德威视
#include "CameraApi.h"
#include "Global_define.h"

extern bool bOutArea;//判断框选部分是否超出区域，0正常，1超出。
extern Mat g_srcImg;//源图像
extern Mat g_tmpImg;//实时显示图像
extern Mat g_dstImg;//框选后的输出图片
extern Mat g_TemplateImg;//模板图片
extern HWND g_hwndShowImg;//显示图像窗口句柄
extern bool g_bRButton;//鼠标右键状态
extern Mat g_mInitialImg1, g_mInitialImg2;//探针朝向、转向函数使用
extern Rect g_Temp_ROI_Rect;//模板匹配后获取的ROI区域  Rect信息（可直接用于Opencv的rectangle函数）
extern vector<Point> g_ProbeSlipDirect;//探针转向的起点，探针朝向的起点坐标，终点为图片中心点坐标
extern vector<Point> g_ProbeSlipLine;//用于显示滑移距离
extern double g_dPixelSize;//单个像素的大小
extern double g_ProbeAngleMicrRatio;//探针侧移1微米时，角度变化值
extern double g_movepixel;//角度调整侧移像素
extern int g_nTurnNum;//1为顺时针，0为逆时针
extern bool g_brotaionmove;//判断是否画侧移线（探针转角）
extern bool g_bslipmove;//判断是否画滑移线
extern int g_nOutputnum;//针痕识别和转角调整时用
extern Mat g_AngleOutput;
extern Mat g_tempResultImg;
#include <QObject>

class NeedleMark : public QObject
{
	Q_OBJECT

public:
	NeedleMark(QObject *parent = nullptr);
	~NeedleMark();

	NeedleMarkDll* m_pNeedleMark;//动态库指针
	ImageParamsConfig m_ImageparamCfg;


	int GetNeddleImg(std::string imgpath);
	int FinishInitProbe(int probe);
	int GetProbeTowards(int probe);
	int IdentifyProbe(int probe);
	int Read_ImageParaXml();
	int autoAngle(needleMarkLevelParas& configs, Mat SrcImg, double& angle, double& dx);
	int IdentifyMarke(needleMarkLevelParas& configs, double& angle, double& dx);
	int autoInit(needleMarkLevelParas& configs, Mat img1, Mat img2);
	int manualInit(int probe);
	int moveAngle(int probe);
signals:
	void autoInitStatus(int status);
};

