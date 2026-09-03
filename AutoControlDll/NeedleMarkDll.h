#pragma once

#include "needlemarkdll_global.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "QString"
#include <QtGui/qimage.h>
#include <string.h>
#include <queue>
#include <windows.h>
#include <iostream>



using namespace cv;
using namespace std;

//class NEEDLEMARKDLL_EXPORT NeedleMarkDll
class  NeedleMarkDll
{
public:


public:
    NeedleMarkDll();
    ~NeedleMarkDll();
	Mat OpenImg(QString strImgPath);//打开图片
	QImage mat_to_qim(Mat& mat);//Mat格式转QImage格式
	Mat  qim_to_mat(QImage& qim);//QImage格式转Mat格式 
	int Img_Template(Mat& srcImg, Mat TemImg,Rect& ROIRect);//模板匹配,输出匹配后的ROI Rect信息
	int NeedleToward(Mat srcImg);//判断探针朝向


	//提取针痕面积（新版）
	Mat GetProbeMarkArea(Mat srcImg, int Towards, double PixelSize, vector<int>& SumArea, vector<Rect>& ActualRect);//
	vector<Point2f>GetProbeTipLine(Mat srcImg, int Towards, double PixelSize);//寻找探针针尖直线，输入图像及探针朝向，返回直线坐标点集合
	int GetArea(Mat srcImg, double PixelSize, int& SumArea);//获得针痕面积（改版）
	int NoiseRemoval(vector<float>LinePositionXY, int Towards, double PixelSize, vector<float>& LineXYOutPut);//去除针尖直线检测中的噪点


	//识别探针针尖，提取针尖前端的针痕面积，最终输出
	//输入图片，探针朝向转向向量，模板匹配ROI区域，针痕识别范围，图片像素大小，微米角度比，输出图片，输出角度
	int DstProbeMarkArea(Mat& srcImg, const vector<Point> &ProbeSlipDirect
		,const Rect& g_Temp_ROI_Rect,const double GetMarkDistance,
		const double PixelSize,Mat& outImg,double& ProbeTurnAngle);
	//将double转换成String格式
	string DoubleToStr(double num);
	//用于识别探针转动方向和针尖朝向相关函数																 //计算两张灰度图片中的并集,图片大小需一致
	// OutputImg1 = InputImg2 - InputImg1
	int Img_diff(Mat& InputImg1, Mat& InputImg2, Mat& OutputImg1);
	//用于识别探针转动方向和针尖朝向相关函数,使用大津法对帧差图片进行二值化处理																 //计算两张灰度图片中的并集,图片大小需一致
// OutputImg1 = InputImg2 - InputImg1
	int Img_difftwo(Mat& InputImg1, Mat& InputImg2, Mat& OutputImg1);
	//用于判断图1和图2的黑白像素比例
	int img_diff_estimate(const Mat& InputImg1, const Mat& InputImg2);
	//图片下采样，减少计算量
	void Imgpyrdown(Mat& Inputimg, int Numlebels);
	//根据下采样情况进行开运算，超过2层自动跳过
	void Imgpyrdown_MORPH_OPEN(Mat& InpuImg, int pyrDownNum);
	//检测图像特征像素点情况（二值化特征）
	//输出 轮廓外接矩形框左上角坐标，轮廓外接矩形框右上角坐标(非探针ROI模板)
	void Img_diff_contourparam(const Mat& InputImg, vector<Point>& OutputParam);
	//根据Img_diff_contourparam的ROI大小判断图片是否抖动，该函数只能用于Img_diff_ROI_One中。
	//函数中输入图片和点容器与Img_diff_contourparam一致
	void Img_diff_contourparam_estimate(const Mat& InputImg,vector<Point>& InputParam);
	//将MImg_diff函数输出的Inputimg导入
	// 输出 个点坐标
	//第一 和 第两个点坐标 分别为Rect的 左上角点，右下角点（MImg_diff中InputImg1中的ROI）
	//第三 和 第四个点坐标 分别为 图像上探针的移动方向起始点，终点（图像中心点）
	//第五 个坐标点为 探针朝向起点（向量终点为第四个点坐标）
	void Img_diff_ROI_Two(const Mat& InputImg1, const Mat& InputImg2, vector<Point>& OutputParam);
	//将MImg_diff函数输出的Inputimg导入
	// 输出 个点坐标
	//第一 和 第两个点坐标 分别为Rect的 左上角点，右下角点（MImg_diff中InputImg1中的ROI）
	//第三 和 第四个点坐标 分别为 图像上探针的移动方向起始点，终点（图像中心点）
	//第五 个坐标点为 探针朝向起点（向量终点为第四个点坐标）
	void Img_diff_ROI_One(const Mat& InputImg1, const Mat& InputImg2, vector<Point>& OutputParam);
	//导入MImg_diff_ROI_One或MImg_diff_ROI_Two计算得出的坐标点参数OutputParam
	//导入图片为使用OutputParam中Rect从初始图片截取的ROI区域
	//第一 和 第两个点坐标 分别为Rect的 左上角点，右下角点（模板区域）
	//第三 和 第四个点坐标 分别为 图像上探针的移动方向起始点，终点（图像中心点）
	//第五 个坐标点为 探针朝向起点（向量终点为第四个点坐标）
	void Img_diff_ROI_Three(const Mat& InputImg1, vector<Point>& OutputParam);

	//匹配探针实际转向 默认确定顺时针方向
	//确定探针朝向,转向，转动角度比
	int Img_rotat_direct(Mat& InputImg1, Mat& InputImg2, vector<Point>& OutputParam,
		vector<double>& Pixel_Angle_ratio);

	//设置探针滑移距离
//1.滑移前模板匹配位置（Rect格式）,2. 探针针尖朝向（Vector<Point>起点，终点）
// 3.滑移距离，4.当前图片的像素实际尺寸（单位:mm）
//输出5.坐标（滑移线起点，滑移线终点，起始线起点，起始线终点，图像局部放大ROI左上角，图像局部放大ROI右下角）
	void Define_Slip_Distance(const Rect& ROIRect, const vector<Point>& ProbeDirrectStartPoint
		, double InputSlipDistance, double ImgPixelSize, vector<Point>& SlipDistanceInformation);
//设置探针侧移滑移距离
	int Probe_Angle_move(const Mat& Img,const vector<Point>& ProbeSlipDirect,
		const Rect& g_Temp_ROI_Rect, const double movepixel, vector<Point>& rotaionmoveline);

public:
	QString m_InputImgPath;//储存传入图片路径
	Mat MatchImg;//匹配所得图片

	//识别针痕变量
	bool turn;//0表示左侧面积大，1则表示右侧面积大
	float ratio;//两侧面积之比，小比大
	bool bShowRatio;//三根针痕时显示针痕比例，值为1，否则为0

	vector<Rect>SumRect;//针痕合并矩形框

	vector<vector<Point>>contours_temp;//用于存放相邻区域轮廓
	vector<int>BigRectArea;//定义大矩形框内轮廓的总面积，向量格式，容量为10

};
