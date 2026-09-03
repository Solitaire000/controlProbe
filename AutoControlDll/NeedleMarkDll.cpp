#include "NeedleMarkDll.h"

NeedleMarkDll::NeedleMarkDll()
{
	
}
NeedleMarkDll:: ~NeedleMarkDll()
{
	
}




Mat NeedleMarkDll::OpenImg(QString strImgPath)
{
	string ImgPath = strImgPath.toLocal8Bit().toStdString();
	Mat Img = imread(ImgPath);
	return Img;
}

QImage NeedleMarkDll::mat_to_qim(Mat& mat)
{
	cvtColor(mat, mat, COLOR_BGR2RGB);

	switch (mat.type())
	{
	case CV_8UC1:
	{
		QImage qim((uchar*)mat.data, mat.cols, mat.rows, mat.cols * 1, QImage::Format_Grayscale8);
		return qim;
	}
	case CV_8UC3:
	{
		QImage qim((uchar*)mat.data, mat.cols, mat.rows, mat.cols * 3, QImage::Format_RGB888);
		return qim;
	}
	case CV_8UC4:
	{
		QImage qim((uchar*)mat.data, mat.cols, mat.rows, mat.cols * 4, QImage::Format_RGBA8888);
		return qim;
	}


	default:
	{
		QImage qim;
		return qim;
	}
	break;
	}
}



Mat NeedleMarkDll::qim_to_mat(QImage& qim)
{
	Mat mat = Mat::zeros(qim.height(), qim.width(), qim.format());

	switch (qim.format())
	{
	case QImage::QImage::Format_Grayscale8: //灰度图
	{
		mat = Mat(qim.height(), qim.width(), CV_8UC1, (void*)qim.constBits(), qim.bytesPerLine());
		return mat;
	}
	case QImage::Format_RGB888: //3通道彩色
	{
		mat = Mat(qim.height(), qim.width(), CV_8UC3, (void*)qim.constBits(), qim.bytesPerLine());
		return mat;
	}
	case QImage::Format_ARGB32: //4通道彩色
	{
		mat = Mat(qim.height(), qim.width(), CV_8UC4, (void*)qim.constBits(), qim.bytesPerLine());
		return mat;
	}

	default:
		return mat;
		break;
	}
}

int  NeedleMarkDll::Img_Template(Mat& srcImg, Mat TemImg, Rect& ROIRect)
{

	
	Mat srcImage = srcImg.clone();//复制原图
	Mat srcImgPydown = srcImg.clone();
	Mat TemImgPydown = TemImg.clone();;
	Mat resultImg;
	
	int PyrdownNum = 3;
	int modelpictureNum = pow(2, PyrdownNum);//根据金字塔下采样坐标确定未下采样坐标，如3层金字塔缩小倍数为2*2*2
	Imgpyrdown(srcImgPydown,PyrdownNum);
	Imgpyrdown(TemImgPydown, PyrdownNum);

	//进行模板匹配，参数分别为原图，模板，匹配范围，匹配方法
	matchTemplate(srcImgPydown, TemImgPydown, resultImg, 5);
	double minValue, maxValue;
	Point minLocation, maxLocation, matchLocation;
	minMaxLoc(resultImg, &minValue, &maxValue, &minLocation, &maxLocation);
	if (maxValue > 0.5 )
	{
		Point Point_A = maxLocation * modelpictureNum;//左上角
		//矩形长宽
		Point Point_B = Point(maxLocation.x * modelpictureNum + TemImg.cols, maxLocation.y * modelpictureNum + TemImg.rows);
		ROIRect = Rect(Point_A, Point_B);//以Rect 形式储存
		return 1;
	}
	else
		return 0;
}


int  NeedleMarkDll::NeedleToward(Mat srcImg)
{

	Mat ImgGray;
	if (srcImg.type() == CV_8UC1)//如果不是灰度图则转为灰度图
	{
		ImgGray = srcImg;
	}
	else
	{
		cvtColor(srcImg, ImgGray, COLOR_BGR2GRAY);
	}

	int height = srcImg.rows;
	int width = srcImg.cols;
	//求出图片中心坐标
	int cX = width / 2;
	int cY = height / 2;

	float TopGray, BottomGray, LeftGray, RightGray;//定义上半边，下半边，左半边，右半边灰度值。

	TopGray = 0;
	for (int i = 0; i < cY; i++)//计算上半边灰度值
	{
		for (int j = 0; j < width; j++)
		{
			float gray = ImgGray.at<uchar>(i, j);
			TopGray = gray + TopGray;
		}
	}
	TopGray = 2 * TopGray / (height * width);

	BottomGray = 0;
	for (int i = cY + 1; (cY < i) & (i < height); i++)//计算下半边灰度值
	{
		for (int j = 0; j < width; j++)
		{
			float gray = ImgGray.at<uchar>(i, j);
			BottomGray = gray + BottomGray;
		}
	}
	BottomGray = 2 * BottomGray / (height * width);

	LeftGray = 0;
	for (int i = 0; i < height; i++)//计算左半边灰度值
	{
		for (int j = 0; j < cX; j++)
		{
			float gray = ImgGray.at<uchar>(i, j);
			LeftGray = gray + LeftGray;
		}
	}
	LeftGray = 2 * LeftGray / (height * width);


	RightGray = 0;
	for (int i = 0; i < height; i++)//计算上半边灰度值
	{
		for (int j = cX + 1; (cX < j) & (j < width); j++)
		{
			float gray = ImgGray.at<uchar>(i, j);
			RightGray = gray + RightGray;
		}
	}
	RightGray = 2 * RightGray / (height * width);


	float TBratio, LRratio;//定义上下灰度值比例，左右灰度值比例
	int TBdirection = 0;//定义探针朝向，TBdirection=0探针向下,LRdirection=0探针向右。
	int LRdirection = 0;

	if (TopGray < BottomGray)
	{
		TBratio = TopGray / BottomGray;
		TBdirection = 0;
	}
	else if (BottomGray < TopGray)
	{
		TBratio = BottomGray / TopGray;
		TBdirection = 1;
	}
	else
	{
		TBratio = 1;
	}

	if (LeftGray < RightGray)
	{
		LRratio = LeftGray / RightGray;
		LRdirection = 0;
	}
	else if (RightGray < LeftGray)
	{
		LRratio = RightGray / LeftGray;
		LRdirection = 1;
	}
	else
	{
		LRratio = 1;
	}
	cout << "TBratio= " << TBratio << endl;
	cout << "LRratio= " << LRratio << endl;



    //判断探针朝向函数返回值
	//0：函数运行错误
	//1：探针针尖朝下
	//2：探针针尖朝上
	//3：探针针尖朝右
	//4：探针针尖朝左

	if (TBratio < LRratio)//判断探针朝向
	{
		if (TBdirection == 0)
		{
			//putText(srcImg, string("Down"), Point(cX, cY), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
			return 1;
		}
		else
		{
			//putText(srcImg, string("Up"), Point(cX, cY), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
			return 2;
		}
	}
	if (LRratio < TBratio)
	{
		if (LRdirection == 0)
		{
			//putText(srcImg, string("Right"), Point(cX, cY), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
			return 3;
		}
		else
		{
			//putText(srcImg, string("Left"), Point(cX, cY), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
			return 4;
		}
	}



	return 0;

}




int NeedleMarkDll::DstProbeMarkArea(Mat& srcImg, const vector<Point>& ProbeSlipDirect
	, const Rect& g_Temp_ROI_Rect, const double GetMarkDistance,
	const double PixelSize, Mat& outImg, double& ProbeTurnAngle)
{
	int Towards = 0;//探针朝向 1下 2上 3右 4左
	int Turndirect = 0;//探针的顺时针转向 指向朝向向量(⇑)的左侧←为1 (⇑←) 右侧为2(→⇑)
	int MarkTurndirect = 0;//指向两侧针痕中面积大的针痕，指向朝向向量(⇑)的左侧←为1 (⇑←) 右侧为2(→⇑)
	double imgprobeMarkROInum =  GetMarkDistance; //用于调节识别针痕的区域（从针尖出发，移动多少个滑移距离）
	vector<int> ProbeArea;//存针痕面积
	vector<Rect> ProbeMarkROI;//存针痕ROI的Rect信息
	//通过探针朝向,确定针痕识别区域
	Rect Rect2;
	Mat img1 = srcImg(g_Temp_ROI_Rect);//探针区域
	//垂直方向
	if (ProbeSlipDirect[1].x == ProbeSlipDirect[2].x)
	{//朝向向量朝下
		if (ProbeSlipDirect[1].y < ProbeSlipDirect[2].y)
		{	//探针朝向
			Towards = 1;
			//指向朝向向量(⇑)的左侧←为1(⇑←)
			if (ProbeSlipDirect[1].x > ProbeSlipDirect[0].x)
				Turndirect = 1;
			//指向朝向向量(⇑)的右侧为2(→⇑)
			else
				Turndirect = 2;
			//针痕识别区域
			Rect2 = Rect(Point(g_Temp_ROI_Rect.tl().x, g_Temp_ROI_Rect.br().y)
				, Point(g_Temp_ROI_Rect.br().x, int(g_Temp_ROI_Rect.br().y + imgprobeMarkROInum)));//针痕区域
		}
	//朝向向量朝上
		else
		{	//探针朝向
			Towards = 2;
			//指向朝向向量(⇑)的右侧为2(→⇑)
			if (ProbeSlipDirect[1].x > ProbeSlipDirect[0].x)
				Turndirect = 2;
			//指向朝向向量(⇑)的左侧←为1(⇑←)
			else
				Turndirect = 1;
			Rect2 = Rect(Point(g_Temp_ROI_Rect.tl().x, int(g_Temp_ROI_Rect.tl().y - imgprobeMarkROInum))
				, Point(g_Temp_ROI_Rect.br().x, g_Temp_ROI_Rect.tl().y));//针痕区域	
		}
	}
	//水平方向
	else if (ProbeSlipDirect[1].y == ProbeSlipDirect[2].y)
	{
		//向量朝右
		if (ProbeSlipDirect[1].x < ProbeSlipDirect[2].x)
		{	//探针朝向
			Towards = 3;
			//指向朝向向量(⇑)的右侧为2(→⇑)
			if (ProbeSlipDirect[1].y > ProbeSlipDirect[0].y)
				Turndirect = 2;
			//指向朝向向量(⇑)的左侧←为1(⇑←)
			else
				Turndirect = 1;
			Rect2 = Rect(Point(g_Temp_ROI_Rect.br().x, g_Temp_ROI_Rect.tl().y)
				, Point(int(g_Temp_ROI_Rect.br().x + imgprobeMarkROInum), g_Temp_ROI_Rect.br().y));//针痕区域	
		}
		//向量朝左
		else
		{   //探针朝向
			Towards = 4;
			//指向朝向向量(⇑)的左侧←为1(⇑←)
			if (ProbeSlipDirect[1].y > ProbeSlipDirect[0].y)
				Turndirect = 1;
			//指向朝向向量(⇑)的右侧为2(→⇑)
			else
				Turndirect = 2;
			Rect2 = Rect(Point(int(g_Temp_ROI_Rect.tl().x - imgprobeMarkROInum), g_Temp_ROI_Rect.tl().y)
				, Point(g_Temp_ROI_Rect.tl().x, g_Temp_ROI_Rect.br().y));//针痕区域
		}
	}
	Rect NewROI = g_Temp_ROI_Rect | Rect2;
	if (NewROI.tl().x < 0 || NewROI.tl().y < 0
		|| NewROI.br().x > srcImg.cols || NewROI.br().y > srcImg.rows)
		return -2;
	Mat img2 = srcImg(NewROI);
	//提取针痕面积，手动调节阈值
	if (!img2.empty() && Towards!=0)
	 {	//输出针痕排序根据朝向
		 //当朝向垂直时，针痕按水平（x）方向从小到大排列 （水平时按y，从小到大排列）
		 //即 ProbeArea[0]的x坐标 < ProbeArea[1]的x坐标 < ProbeArea[2]的x坐标
		 outImg = GetProbeMarkArea(img2,Towards, PixelSize,ProbeArea,ProbeMarkROI);
	  }

	double GGRatio,SGRatio;
	double ProbeArea1,ProbeArea2,ProbeArea3;
	//判断针痕是不是为三个
	if (ProbeArea.size() == 3 && !(ProbeArea[0]==0 && ProbeArea[1] == 0 && ProbeArea[2] == 0))
	{//求针痕面积比例
		ProbeArea1 = ProbeArea[0];
		ProbeArea2 = ProbeArea[1];
		ProbeArea3 = ProbeArea[2];
		
		if (ProbeArea2 > ProbeArea1 && ProbeArea2 > ProbeArea3 && (ProbeArea1 == 0 || ProbeArea3 == 0))
			return -3;
		if (ProbeArea1 > ProbeArea3)
		{
			GGRatio = ProbeArea3 / ProbeArea1;
			SGRatio = ProbeArea2 / ProbeArea1;
		}
	    else if (ProbeArea1 < ProbeArea3)
		{
			GGRatio = ProbeArea1 / ProbeArea3;
			SGRatio = ProbeArea2 / ProbeArea3;
		}
		else 
			GGRatio = 1;
	}
	 //不是,返回-1
	else
		 return -1;
	//计算探针转动角度
	//两侧针尖比例为零，中间与外侧比例不为零时
	if (GGRatio == 0 && SGRatio != 0)
	{
		ProbeTurnAngle = -(1.785 * SGRatio) + 0.917;
	}
	//两侧针尖比例不为零时
	else if(GGRatio != 0)
	{
		ProbeTurnAngle =(0.4014 * GGRatio * GGRatio) - (0.782 * GGRatio) + 0.3688;
	}
	//都为零时，角度大于0.917°
	else if (GGRatio == 0 && SGRatio == 0)
	{
		ProbeTurnAngle = -(1.785 * SGRatio) + 0.917;
	}
	 switch (Towards)
	{//指向两侧针痕中面积大的针痕，指向朝向向量(⇑)的左侧←为1 (⇑←) 右侧为2(→⇑)
		 case 1://探针针尖朝下
			 if (ProbeArea[0] > ProbeArea[2])//指向朝向向量右侧
				 MarkTurndirect = 2;
			 else
				 MarkTurndirect = 1;
			 break;
		 case 2://探针针尖朝上
			 if (ProbeArea[0] > ProbeArea[2])//指向朝向向量左侧
				 MarkTurndirect = 1;
			 else
				 MarkTurndirect = 2;
			 break;
		 case 3://探针针尖朝右
			 if (ProbeArea[0] > ProbeArea[2])//指向朝向向量左侧
				 MarkTurndirect = 1;
			 else
				 MarkTurndirect = 2;
			 break;
		 case 4://探针针尖朝左
			 if (ProbeArea[0] > ProbeArea[2])//指向朝向向量左侧
				 MarkTurndirect = 2;
			 else
				 MarkTurndirect = 1;
			 break;
		 default:
			 break;
	}
	 if (MarkTurndirect != 0 && Turndirect != 0)
	 {
		 int Compareturndircetabs = abs(Turndirect - MarkTurndirect);//值为零，说明顺时针转向与所需转向一致
		 if (Compareturndircetabs == 0 )//转向一致，需要顺时针
			 return 1;//顺时针转动
		 else
			 return 2;//逆时针转动
	 }
	 else
		 return 0;//转向赋值失败
}


int NeedleMarkDll::Img_diff(Mat& InputImg1, Mat& InputImg2, Mat& OutputImg1)
{
	if (InputImg1.size != InputImg2.size && InputImg1.size != OutputImg1.size)
	{
		return -1;
	}


	subtract(InputImg2, InputImg1, OutputImg1);//picture3为picture2减去picture1(求交集)

////将两图片交集像素设置为255
	int Img255_Pixel_Num = 0;
	for (int i = 0; i < OutputImg1.rows; i++)
	{
		uchar* imgptr = OutputImg1.ptr<uchar>(i);//指针遍历
		for (int j = 0; j < OutputImg1.cols; j++)
		{

			if (imgptr[j] > 5)
			{
				imgptr[j] = 255;
				Img255_Pixel_Num++;
			}
		}
	}
//1600X1200像素时，小于5000个255点返回0
//其他大小按像素个数换算 : 5000 * double((OutputImg1.cols)/1600.0 ) * double((OutputImg1.cols) / 1600.0)
	int Img255_PixelStd = 5000 * double((OutputImg1.cols) / 1600.0) * double((OutputImg1.cols) / 1600.0);
	if (Img255_Pixel_Num < Img255_PixelStd)
	{
		return 0;
	}
	return 1;
}
	
int NeedleMarkDll::Img_difftwo(Mat& InputImg1, Mat& InputImg2, Mat& OutputImg1)
{
	if (InputImg1.size != InputImg2.size && InputImg1.size != OutputImg1.size)
	{
		return 0;
	}
	subtract(InputImg2, InputImg1, OutputImg1);//picture3为picture2减去picture1(求交集)
	threshold(OutputImg1, OutputImg1, 0, 255, THRESH_OTSU);
	////将两图片交集像素设置为255
	int Img255_Pixel_Num = 0;
	for (int i = 0; i < OutputImg1.rows; i++)
	{
		uchar* imgptr = OutputImg1.ptr<uchar>(i);//指针遍历
		for (int j = 0; j < OutputImg1.cols; j++)
		{
			if (imgptr[j] == 255)
				Img255_Pixel_Num++;
		}
	}
	//1600X1200像素时，小于5000个255点返回0
	//其他大小按像素个数换算 : 5000 * double((OutputImg1.cols)/1600.0 ) * double((OutputImg1.cols) / 1600.0)
	int Img255_PixelStd = 5000 * double((OutputImg1.cols) / 1600.0) * double((OutputImg1.cols) / 1600.0);
	if (Img255_Pixel_Num < Img255_PixelStd)
	{
		return 0;
	}

	return 1;
}

int NeedleMarkDll::img_diff_estimate(const Mat& InputImg1, const Mat& InputImg2)
{
	double imgnum1 = 0;
	double imgnum2 = 0;
	if (InputImg1.size()==InputImg2.size())
	{
		for (int i = 0;i<InputImg1.rows;i++)
		{
			const uchar* imgptr1 = InputImg1.ptr<uchar>(i);
			const uchar* imgptr2 = InputImg2.ptr<uchar>(i);
			for (int j = 0;j< InputImg1.cols;j++)
			{
				if (imgptr1[j] == 255)
					imgnum1++;
				if (imgptr2[j] == 255)
					imgnum2++;
			}
		}
		if (imgnum1 == 0 || imgnum2 == 0 || imgnum1 > 0.5 * InputImg1.cols * InputImg1.rows 
			|| imgnum2 > 0.5 * InputImg2.cols * InputImg2.rows)
			return -1;
		if (imgnum1 > imgnum2 && (imgnum2 / imgnum1) < 0.2)
			return -1;
		if (imgnum1 < imgnum2 && (imgnum1 / imgnum2) < 0.2)
			return -1;
		return 1;		
	}
}

void NeedleMarkDll::Imgpyrdown(Mat& Inputimg, int Numlebels)
{
	for (int i = 0; i < Numlebels; i++)
	{
		pyrDown(Inputimg, Inputimg);
	}
}

void NeedleMarkDll::Imgpyrdown_MORPH_OPEN(Mat& InpuImg, int pyrDownNum)
{
	///////先腐蚀后膨胀，去掉噪点
	Mat element;
	if (pyrDownNum == 0)
		element = getStructuringElement(MORPH_ELLIPSE, Size(9, 9), Point(-1, -1));
	else if (pyrDownNum == 1)
		element = getStructuringElement(MORPH_ELLIPSE, Size(7, 7), Point(-1, -1));
	else if (pyrDownNum == 2)
		element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
	if (!element.empty())
		morphologyEx(InpuImg, InpuImg, MORPH_OPEN, element);//开运算
}

void NeedleMarkDll::Img_diff_contourparam(const Mat& InputImg, vector<Point>& OutputParam)
{
	int x_ymin = InputImg.rows;//存按列扫描时，有255像素点的最小行数
	int x_ymax = 0;//存按列扫描时，有255像素点的最大行数
	int y_xmin = InputImg.cols;//存按行扫描时，有255像素点的最小列数
	int y_xmax = 0;//存按行扫描时，有255像素点的最大列数
	///////按行扫描
	for (int i = 0; i < InputImg.rows; i++)
	{
		const uchar* imgptr = InputImg.ptr<uchar>(i);//指针遍历
		for (int j = 0; j < InputImg.cols; j++)
		{
			if (imgptr[j] == 255)//判断图片该像素点是否等于255
			{
				if (j < y_xmin)//将255像素点最小列数存入
					y_xmin = j;
				if (j > y_xmax)//将255像素点最大列数存入
					y_xmax = j;
			}
		}
	}

	// 	///////按列扫描
	for (int j = y_xmin; j < y_xmax; j++)
	{

		for (int i = 0; i < InputImg.rows; i++)
		{
			const uchar* imgptr = InputImg.ptr<uchar>(i);
			if (imgptr[j] == 255)
			{
				if (i < x_ymin)//将255像素点最小行数存入
					x_ymin = i;
				if (i > x_ymax)//将255像素点最大行数存入
					x_ymax = i;
			}
		}
	}
	OutputParam.push_back(Point(y_xmin, x_ymin));//存像素轮廓左上角点坐标
	OutputParam.push_back(Point(y_xmax, x_ymax));//存像素轮廓右上角点坐标
}




void NeedleMarkDll::Img_diff_contourparam_estimate(const Mat& InputImg,vector<Point>& InputParam)
{	//判断抖动的方法是通过判断ROI区域占整张图片的大小而定
	//Img_diff获得的ROI占整张图片的二分之一，则将ROI容器清空
	if (!InputParam.empty())
	{
		int  ROI_x = abs(InputParam[1].x- InputParam[0].x);
		int ROI_y = abs(InputParam[1].y - InputParam[0].y);
		if (ROI_y == InputImg.rows)
			if (ROI_x > 0.5 * InputImg.cols)
				InputParam.clear();
		if (ROI_x == InputImg.cols)
			if (ROI_y > 0.5 * InputImg.rows)
				InputParam.clear();
		if (ROI_x * ROI_y > 0.5 * InputImg.cols * InputImg.rows)
			InputParam.clear();
	}
}

void NeedleMarkDll::Img_diff_ROI_Two(const Mat& InputImg1, const Mat& InputImg2, vector<Point>& OutputParam)
{
	vector<Point>InputParam1, InputParam2;
	Img_diff_contourparam(InputImg1, InputParam1);
	Img_diff_contourparam(InputImg2, InputParam2);
	if (!InputParam1.empty() && !InputParam2.empty())
	{
		if (abs(InputParam1[0].y - InputParam2[0].y) < 10 || abs(InputParam1[1].y - InputParam2[1].y) < 10)//判断方向是否为垂直方向
		{
			//判断图片上探针移动方向：向左或向右
			if (InputParam1[0].x < InputParam2[0].x && InputParam1[1].x < InputParam2[1].x)//图片上由左向右
			{	//获取探针ROI区域(提取初始图片)
				OutputParam.push_back(InputParam1[0]);//ROI的左上角
				//寻找右下角（直接使用InputParam2[1]做右下脚会多框背景）
				int point_x_max = 0;//寻找Inputimg2内轮廓x最大值
				for (int i = 0; i < InputImg2.rows; i++)//按行扫描
				{
					const uchar* imgptr = InputImg2.ptr<uchar>(i);//指针遍历
					for (int j = 0; j < InputImg2.cols; j++)//按列扫描
					{
						if (imgptr[j] == 255)
						{
							if (j > point_x_max)
								point_x_max = j;
							break;
						}
					}
				}
				OutputParam.push_back(Point(point_x_max, InputParam2[1].y));//ROI的右下角
				//图片上由左向右
				//存向量方向
				OutputParam.push_back(Point(0, InputImg2.rows / 2));//起点
				OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
			}
			else if (InputParam1[0].x > InputParam2[0].x && InputParam1[1].x > InputParam2[1].x)//图片上由右向左
			{	//获取探针ROI区域
				//寻找左上角（直接使用InputParam2[1]做右下脚会多框背景）
				int point_x_min = InputImg2.cols;//寻找inputimg2内轮廓x最小值
				for (int i = 0; i < InputImg2.rows; i++)//按行扫描
				{
					const uchar* imgptr = InputImg2.ptr<uchar>(i);//指针遍历
					for (int j = 0; j < InputImg2.cols; j++)//按列扫描
					{
						if (imgptr[InputImg2.cols - j] == 255)
						{
							if ((InputImg2.cols - j) < point_x_min)
								point_x_min = (InputImg2.cols - j);
							break;
						}
					}
				}
				OutputParam.push_back(Point(point_x_min, InputParam2[0].y));//ROI的左上角
				OutputParam.push_back(InputParam1[1]);//ROI的右下角
				//图片上由右向左
				//存向量方向			
				OutputParam.push_back(Point(InputImg2.cols, InputImg2.rows / 2));//起点
				OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
			}
			//判断探针针尖朝向
			if (OutputParam[0].y < abs(OutputParam[1].y - InputImg1.rows))
				OutputParam.push_back(Point(InputImg1.cols / 2, 0));//探针朝向为由上朝下
			else
				OutputParam.push_back(Point(InputImg1.cols / 2, InputImg1.rows));//探针朝向为由下朝上
		}

		if (abs(InputParam1[0].x - InputParam2[0].x) < 10 || abs(InputParam1[1].x - InputParam2[1].x) < 10)//判断是否为水平方向向量
		{
			//判断图片上探针移动方向：向上或向下
			if (InputParam1[0].y < InputParam2[0].y && InputParam1[1].y < InputParam2[1].y)//图片上由上向下
			{	//获取探针ROI区域
				OutputParam.push_back(InputParam1[0]);//ROI的左上角
				//寻找右下角（直接使用InputParam2[1]做右下脚会多框背景）
				int point_y_max = 0;//寻找inputimg2内轮廓y最大值
					// 	///////按列扫描
				for (int j = 0; j < InputImg2.cols; j++)
				{

					for (int i = 0; i < InputImg2.rows; i++)
					{
						const uchar* imgptr = InputImg2.ptr<uchar>(i);
						if (imgptr[j] == 255)
						{
							if (i > point_y_max)
								point_y_max = i;
							break;
						}
					}
				}
				OutputParam.push_back(Point(InputParam2[1].x, point_y_max));//ROI的右下角
				//图片上由上向下
				//存向量方向
				OutputParam.push_back(Point(InputImg2.cols / 2, 0));//起点
				OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
			}
			else if (InputParam1[0].y > InputParam2[0].y && InputParam1[1].y > InputParam2[1].y)//图片上由下朝上
			{	//获取探针ROI区域
				//寻找右下角（直接使用InputParam2[1]做右下脚会多框背景）
				int point_y_min = InputImg2.rows;//寻找inputimg2内轮廓y最小值
					// 	///////按列扫描
				for (int j = 0; j < InputImg2.cols; j++)
				{

					for (int i = 0; i < InputImg2.rows; i++)
					{
						int k = InputImg2.rows - i - 1;
						const uchar* imgptr = InputImg2.ptr<uchar>(k);
						if (imgptr[j] == 255)
						{
							if (k < point_y_min)
								point_y_min = k;
							break;
						}
					}
				}
				OutputParam.push_back(Point(InputParam2[0].x, point_y_min));//ROI的左上角
				OutputParam.push_back(InputParam1[1]);//ROI的右下角
				//图片上由下向上
				//存向量方向
				OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows));//起点
				OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
			}
			//判断探针针尖朝向
			if (OutputParam[0].x < abs(OutputParam[1].x - InputImg1.cols))
				OutputParam.push_back(Point(0, InputImg1.rows / 2));//探针朝向为由左朝右
			else
				OutputParam.push_back(Point(InputImg1.cols, InputImg1.rows / 2));//探针朝向为由右朝左

		}

	}
}

void NeedleMarkDll::Img_diff_ROI_One(const Mat& InputImg1, const Mat& InputImg2, vector<Point>& OutputParam)
{
	vector<Point>InputParam1, InputParam2;
	Img_diff_contourparam(InputImg1, InputParam1);
	Img_diff_contourparam_estimate(InputImg1, InputParam1);//判断图片是否抖动

	Img_diff_contourparam(InputImg2, InputParam2);
	Img_diff_contourparam_estimate(InputImg2, InputParam2);//判断图片是否抖动
	if (!InputParam1.empty() && !InputParam2.empty())
	{

		//判断方向是否为垂直方向
		if (abs(InputParam1[0].y - InputParam2[0].y) < 10 || abs(InputParam1[1].y - InputParam2[1].y) < 10)
		{//判断探针是否侧移大于等于整个探针轮廓
		//大于等于
			if (InputParam1[1].x <= InputParam2[0].x && InputParam2[1].x <= InputParam1[0].x)
			{

			}
			//小于
			else
			{
				//判断图片上探针转动方向：向左或向右
				//由左向右
				if (InputParam1[0].x < InputParam2[0].x && InputParam1[1].x < InputParam2[1].x)
				{	//获取探针ROI区域(提取初始图片)
				//探针朝向为由上朝下
					if (InputParam1[0].y < abs(InputParam1[1].y - InputImg1.rows))
					{
						OutputParam.push_back(InputParam1[0]);//ROI的左上角
						OutputParam.push_back(Point(InputParam2[1].x, InputParam2[1].y + 2));//ROI的右下角
					}
					//探针朝向为由下朝上
					else
					{
						OutputParam.push_back(Point(InputParam1[0].x, InputParam1[0].y - 2));//ROI的左上角
						OutputParam.push_back(InputParam2[1]);//ROI的右下角
					}
					//图片上转动向量由左向右
					OutputParam.push_back(Point(0, InputImg2.rows / 2));//起点
					OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
				}
				//由右向左
				else if (InputParam1[0].x > InputParam2[0].x && InputParam1[1].x > InputParam2[1].x)
				{	//获取探针ROI区域
				//探针朝向为由上朝下
					if (InputParam1[0].y < abs(InputParam1[1].y - InputImg1.rows))
					{
						OutputParam.push_back(InputParam2[0]);//ROI的左上角
						OutputParam.push_back(Point(InputParam1[1].x, InputParam1[1].y + 2));//ROI的右下角
					}
					//探针朝向为由下朝上
					else
					{
						OutputParam.push_back(Point(InputParam2[0].x, InputParam2[0].y - 2));//ROI的左上角
						OutputParam.push_back(InputParam1[1]);//ROI的右下角
					}
					//图片上由右向左
					//存向量方向			
					OutputParam.push_back(Point(InputImg2.cols, InputImg2.rows / 2));//起点
					OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
				}
			}
			//判断探针针尖朝向
			if (OutputParam[0].y < abs(OutputParam[1].y - InputImg1.rows))
				OutputParam.push_back(Point(InputImg1.cols / 2, 0));//探针朝向为由上朝下
			else
				OutputParam.push_back(Point(InputImg1.cols / 2, InputImg1.rows));//探针朝向为由下朝上
		}
		//判断是否为水平方向向量
		else if (abs(InputParam1[0].x - InputParam2[0].x) < 10 || abs(InputParam1[1].x - InputParam2[1].x) < 10)
		{//判断探针是否侧移大于等于整个探针轮廓
		//大于等于
			if (InputParam2[1].y <= InputParam1[0].y && InputParam1[1].y <= InputParam2[0].y)
			{

			}
			//小于
			else
			{
				//判断图片上探针转动方向：向上或向下
				//转动方向由上向下
				if (InputParam1[0].y < InputParam2[0].y && InputParam1[1].y < InputParam2[1].y)
				{	//获取探针ROI区域
					//针尖朝右
					if (InputParam1[0].x < abs(InputParam1[1].x - InputImg1.cols))
					{
						OutputParam.push_back(InputParam1[0]);//ROI的左上角
						OutputParam.push_back(Point(InputParam2[1].x + 2, InputParam2[1].y));//ROI的右下角
					}
					//针尖朝左
					else
					{
						OutputParam.push_back(Point(InputParam1[0].x - 2, InputParam1[0].y));//ROI的左上角
						OutputParam.push_back(InputParam2[1]);//ROI的右下角
					}
					//转动方向由上向下
					OutputParam.push_back(Point(InputImg2.cols / 2, 0));//起点
					OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
				}
				//转动方向由下向上
				else if (InputParam1[0].y > InputParam2[0].y && InputParam1[1].y > InputParam2[1].y)
				{	//获取探针ROI区域
					//针尖朝右
					if (InputParam1[0].x < abs(InputParam1[1].x - InputImg1.cols))
					{
						OutputParam.push_back(InputParam2[0]);//ROI的左上角
						OutputParam.push_back(Point(InputParam1[1].x + 2, InputParam1[1].y));//ROI的右下角
					}
					//针尖朝左
					else
					{
						OutputParam.push_back(Point(InputParam2[0].x - 2, InputParam2[0].y));//ROI的左上角
						OutputParam.push_back(InputParam1[1]);//ROI的右下角
					}
					//转动方向由下向上
					OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows));//起点
					OutputParam.push_back(Point(InputImg2.cols / 2, InputImg2.rows / 2));//终点
				}
			}
			//判断探针针尖朝向
			if (OutputParam[0].x < abs(OutputParam[1].x - InputImg1.cols))
				OutputParam.push_back(Point(0, InputImg1.rows / 2));//探针朝向为由左朝右
			else
				OutputParam.push_back(Point(InputImg1.cols, InputImg1.rows / 2));//探针朝向为由右朝左
		}
	}
}

// 探针初始化用于制作模板
//功能介绍：
void NeedleMarkDll::Img_diff_ROI_Three(const Mat& InputImg1, vector<Point>& OutputParam)
{
	Mat img = InputImg1.clone();
	double factor = 0.9;
	int factornum = 100;
	threshold(img, img, 15, 255, THRESH_BINARY_INV);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(-1, -1));
	dilate(img, img, element);
	erode(img, img, element);
	vector<Point>EdgePoint1, EdgePoint2, NewPoint;
	double Point1_x, Point1_y, Point2_x, Point2_y, forcirculnum1, forcirculnum2;
	//垂直方向
	if (OutputParam[4].x == OutputParam[3].x)
	{	//按行扫描每一列
		for (int i = 0; i < img.rows; i = i + 2)
		{
			const uchar* imgptr = img.ptr<uchar>(i);//指针遍历
			if (EdgePoint1.size() > 1000 && EdgePoint2.size() > 1000)
				break;
			//按列从左往右
			for (int j = 0; j < img.cols; j++)
			{
				if (imgptr[j] == 255)//判断图片该像素点是否等于255
				{
					EdgePoint1.push_back(Point(j, i));
					break;
				}
			}
			//按列从右往左
			for (int j = 0; j < img.cols; j++)
			{
				if (imgptr[img.cols - j - 1] == 255)//判断图片该像素点是否等于255
				{
					EdgePoint2.push_back(Point(img.cols - j - 1, i));
					break;
				}
			}
		}
		//由下向上
		if (OutputParam[4].y > OutputParam[3].y)
		{
			//按行扫，针尖朝上，所以由上往下扫
			for (int i = 0; i < EdgePoint1.size(); i++)
			{
				const uchar* imgptr = img.ptr<uchar>(EdgePoint1[i].y);
				forcirculnum1 = 0;
				forcirculnum2 = 0;
				for (int j = EdgePoint1[i].x; j < EdgePoint2[i].x; j++)
				{
					if (imgptr[j] == 255)
						forcirculnum1++;
					else
						forcirculnum2++;
				}
				if (forcirculnum1 / (forcirculnum1 + forcirculnum2) > factor && forcirculnum1 > factornum)
				{
					//保存ROI区域左上角和右下角点
					NewPoint.push_back(Point(OutputParam[0].x + EdgePoint1[i].x, OutputParam[0].y));
					NewPoint.push_back(Point(OutputParam[0].x + EdgePoint2[i].x, OutputParam[0].y + EdgePoint2[i].y));
					break;
				}
			}
		}
		//由上向下
		else
		{
			//按行扫，针尖朝下，所以由下往上扫
			for (int i = 0; i < EdgePoint1.size(); i++)
			{
				const uchar* imgptr = img.ptr<uchar>(EdgePoint1[EdgePoint1.size() - i - 1].y);
				forcirculnum1 = 0;
				forcirculnum2 = 0;
				for (int j = EdgePoint1[EdgePoint1.size() - i - 1].x;
					j < EdgePoint2[EdgePoint1.size() - i - 1].x; j++)
				{
					if (imgptr[j] == 255)
						forcirculnum1++;
					else
						forcirculnum2++;
				}
				if (forcirculnum1 / (forcirculnum1 + forcirculnum2) > factor && forcirculnum1 > factornum)
				{
					//保存ROI区域左上角和右下角点
					NewPoint.push_back(Point(OutputParam[0].x + EdgePoint1[EdgePoint1.size() - i - 1].x,
						OutputParam[0].y + EdgePoint1[EdgePoint1.size() - i - 1].y));
					NewPoint.push_back(Point(OutputParam[0].x + EdgePoint2[EdgePoint1.size() - i - 1].x,
						OutputParam[1].y));
					break;
				}
			}
		}


	}
	//水平方向
	else if (OutputParam[4].y == OutputParam[3].y)
	{
		// 	///////按列扫描每一行
		for (int j = 0; j < img.cols; j = j + 2)
		{
			if (EdgePoint1.size() > 1000 || EdgePoint2.size() > 1000)
				break;
			//按列，行由上往下扫
			for (int i = 0; i < img.rows; i++)
			{
				const uchar* imgptr = img.ptr<uchar>(i);
				if (imgptr[j] == 255)
				{
					EdgePoint1.push_back(Point(j, i));
					break;
				}
			}
			//按列，行由下往上扫
			for (int i = 0; i < img.rows; i++)
			{
				const uchar* imgptr = img.ptr<uchar>(img.rows - i - 1);
				if (imgptr[j] == 255)
				{
					EdgePoint2.push_back(Point(j, img.rows - i - 1));
					break;
				}
			}
		}
		//由右向左
		if (OutputParam[4].x > OutputParam[3].x)
		{
			//按列扫，针尖朝左，所以由左往右扫
			for (int j = 0; j < EdgePoint1.size(); j++)
			{
				forcirculnum1 = 0;
				forcirculnum2 = 0;
				for (int i = EdgePoint1[j].y; i < EdgePoint2[j].y; i++)
				{
					const uchar* imgptr = img.ptr<uchar>(i);
					if (imgptr[EdgePoint1[j].x] == 255)
						forcirculnum1++;
					else
						forcirculnum2++;
				}
				if (forcirculnum1 / (forcirculnum1 + forcirculnum2) > factor && (forcirculnum1 + forcirculnum2) > factornum)
				{
					//保存ROI区域左上角和右下角点
					NewPoint.push_back(Point(OutputParam[0].x, OutputParam[0].y + EdgePoint1[j].y));
					NewPoint.push_back(Point(OutputParam[0].x + EdgePoint2[j].x, OutputParam[0].y + EdgePoint2[j].y));
					break;
				}
			}
		}
		//由左向右
		else
		{//按列扫，针尖朝右，所以由右往左扫
			for (int j = 0; j < EdgePoint1.size(); j++)
			{
				forcirculnum1 = 0;
				forcirculnum2 = 0;
				for (int i = EdgePoint1[EdgePoint1.size() - j - 1].y;
					i < EdgePoint2[EdgePoint2.size() - j - 1].y; i++)
				{
					const uchar* imgptr = img.ptr<uchar>(i);
					if (imgptr[EdgePoint1[EdgePoint1.size() - j - 1].x] == 255)
						forcirculnum1++;
					else
						forcirculnum2++;
				}
				if (forcirculnum1 / (forcirculnum1 + forcirculnum2) > factor && (forcirculnum1 + forcirculnum2) > factornum)
				{
					//保存ROI区域左上角和右下角点
					NewPoint.push_back(Point(OutputParam[0].x + EdgePoint1[EdgePoint1.size() - j - 1].x,
						OutputParam[0].y + EdgePoint1[EdgePoint1.size() - j - 1].y));
					NewPoint.push_back(Point(OutputParam[1].x,
						OutputParam[0].y + EdgePoint2[EdgePoint1.size() - j - 1].y));
					break;
				}
			}
		}
	}

	if (!NewPoint.empty())
	{
		OutputParam.erase(OutputParam.begin());//将旧的ROI左上角点丢弃
		OutputParam.erase(OutputParam.begin());//将旧的ROI右下角角点丢弃
		OutputParam.insert(OutputParam.begin(), NewPoint[1]);//从容器前端，存入新的ROI右下角角点
		OutputParam.insert(OutputParam.begin(), NewPoint[0]);//从容器前端，存入新的ROI右上角角点
	}
}

int NeedleMarkDll::Img_rotat_direct(Mat& InputImg1, Mat& InputImg2, 
	vector<Point>& OutputParam,vector<double>& Pixel_Angle_ratio)
{
	if (InputImg1.size == InputImg2.size)
	{
		vector<Mat>imgvect1, imgvect2, imgvect3, imgvect4;
		Mat img1, img2, img3, img4;
		//存入原图
		imgvect1.push_back(InputImg1);
		imgvect2.push_back(InputImg2);
		int pyrDownNum = 4;//金字塔层数
	//图像金字塔下采样，并将采样后的图片存入Vector容器中
	//Vector[0]为原图，Vector[1]为一层金子塔，Vector[2]为二层
		for (int i = 0; i < pyrDownNum; i++)
		{
			pyrDown(InputImg1, InputImg1);
			pyrDown(InputImg2, InputImg2);
			imgvect1.push_back(InputImg1);
			imgvect2.push_back(InputImg2);
		}
		//ROI区域，第一次的区域为最小金字塔层的图片尺寸
		Rect imgrect = Rect(0, 0, imgvect1[pyrDownNum].cols, imgvect1[pyrDownNum].rows);
		//用于循环计算
		Point NewRectPoint1, NewRectPoint2;
		//从最小金子塔开始计算，例如 第4层->第2层->第0层
		OutputParam.clear();//清除旧数据，防止数据混乱
		vector<Point> ImgParam;//用于函数内循环使用
		Rect Img1ROI,Img2ROI;//用于函数内循环使用
		for (int j = 0; j < 5; j = j + 2)
		{
			int imgvectnum = pyrDownNum - j;//由于Vector中的图片依此变小，所以从容器末端向前端读取
			if (j > 0)//第一次已定义
			{//NewRectPoint1和NewRectPoint2“在循环末端已恢复”至原图尺寸
			//在框定下采样图片ROI时，需要根据“下采样次数”缩减尺寸
				imgrect = Rect(NewRectPoint1 / pow(2, imgvectnum), NewRectPoint2 / pow(2, imgvectnum));
			}
			//从容器中的图片截取ROI区域，增加运算速度
			img1 = imgvect1[imgvectnum](imgrect);
			img2 = imgvect2[imgvectnum](imgrect);

			if (imgvectnum == 0)
			{
				// ERROR
				Img_diff_ROI_Three(img1, OutputParam);//获得探针模板朝向转向等信息
				//制作模板
				Mat temp = imgvect1[0](Rect(OutputParam[0], OutputParam[1]));
				if (temp.empty())
					return -3;
 				Img_Template(img1, temp, Img1ROI);//匹配图1
 				Img_Template(img2, temp, Img2ROI);//匹配图2
				//计算差值（转动角度）
				double num1 = abs(Img2ROI.br().x - Img1ROI.br().x);
				double num2 = abs(Img2ROI.br().y - Img1ROI.br().y);
				if (num1 != 0 || num2 != 0)
				{
					if (num1 > num2)
						Pixel_Angle_ratio.push_back(Pixel_Angle_ratio[0]*num1);
					else if (num1 < num2)
						Pixel_Angle_ratio.push_back(Pixel_Angle_ratio[0]*num2);
				}
				else
					return 0;
				break;
			}
			else
			{
				ImgParam.clear();
				if (Img_difftwo(img1, img2, img3) == 0)
				{
					return 0;
				}
				Img_difftwo(img2, img1, img4);//求差
				if (img_diff_estimate(img3,img4) == -1)//根据img3和img4的255像素点的比值判断
					return -2;
				Imgpyrdown_MORPH_OPEN(img3, imgvectnum);//开运算
				Imgpyrdown_MORPH_OPEN(img4, imgvectnum);
				if (j == 0)
				{
					Img_diff_ROI_One(img3, img4, ImgParam);//获得探针模板朝向转向等信息
					if (!ImgParam.empty())
						OutputParam.assign(ImgParam.begin(), ImgParam.end());//将第一次获得的ROI和转向和朝向信息赋值
					else 
						return -2;
				}
				else
					Img_diff_ROI_Two(img3, img4, ImgParam);
			}
			//由于计算的图片是通过ROI裁剪过的，且为下采样图片，需要将坐标恢复
			//（计算获得ROI坐标+ROI裁剪框的左上角坐标）* （2^下采样次数）
			NewRectPoint1 = (ImgParam[0] + imgrect.tl()) * pow(2, imgvectnum);
			NewRectPoint2 = (ImgParam[1] + imgrect.tl()) * pow(2, imgvectnum);
			OutputParam.erase(OutputParam.begin());//将旧的ROI左上角点丢弃
			OutputParam.erase(OutputParam.begin());//将旧的ROI右下角角点丢弃
			OutputParam.insert(OutputParam.begin(), NewRectPoint2);//从容器前端，存入新的ROI右下角角点
			OutputParam.insert(OutputParam.begin(), NewRectPoint1);//从容器前端，存入新的ROI右上角角点
		}
		return 1;
	}
	return -1;
}

void NeedleMarkDll::Define_Slip_Distance(const Rect& ROIRect, 
	const vector<Point>& ProbeDirrectStartPoint, double InputSlipDistance
	, double ImgPixelSize, vector<Point>& SlipDistanceInformation)
{
	SlipDistanceInformation.clear();//清空之前获得的信息
		//获取像素真实大小，求出需要滑移多少像素点
		int InputSlipPixel = InputSlipDistance / (1000 * ImgPixelSize);//滑移距离单位为微米
		int Markshownumber = 2;//显示针痕的区域，Markshownumber 倍的滑移距离
		
		//求出滑移方向
		int IProbeDerictNum = 0;//用于保存方向，向图片某侧画滑移线，1为图片右侧，2为图片上侧，3为图片左侧，4为图片下侧；
		//根据朝向设置滑移距离
		if (ProbeDirrectStartPoint[1].x == ProbeDirrectStartPoint[2].x)//垂直方向
			if (ProbeDirrectStartPoint[1].y < ProbeDirrectStartPoint[2].y)
				IProbeDerictNum = 4;//向量朝下
			else
				IProbeDerictNum = 2;//向量朝上
		else if (ProbeDirrectStartPoint[1].y == ProbeDirrectStartPoint[2].y)//水平方向
			if (ProbeDirrectStartPoint[1].x < ProbeDirrectStartPoint[2].x)
				IProbeDerictNum = 1;//向量朝右
			else
				IProbeDerictNum = 3;//向量朝左
	//提取ROIRect信息
		Point ROIRectA = ROIRect.tl();//左上角坐标
		Point ROIRectB = ROIRect.br();//右下角坐标

		int turnright_x = ROIRectB.x + Markshownumber * InputSlipPixel;
		int turnup_y = ROIRectA.y - Markshownumber * InputSlipPixel;
		int turnleft_x = ROIRectA.x - Markshownumber * InputSlipPixel;
		int turndown_y = ROIRectB.y + Markshownumber * InputSlipPixel;
		if (turnright_x > 0 && turnup_y > 0 && turnleft_x > 0 && turndown_y)
		{		
			switch (IProbeDerictNum)
			{
			case 1://向右
				SlipDistanceInformation.push_back(Point(ROIRectB.x + InputSlipPixel, ROIRectA.y));//滑移线起点
				SlipDistanceInformation.push_back(Point(ROIRectB.x + InputSlipPixel, ROIRectB.y)); //滑移线终点
				SlipDistanceInformation.push_back(Point(ROIRectB.x , ROIRectA.y));//起始线起点
				SlipDistanceInformation.push_back(Point(ROIRectB.x , ROIRectB.y));//起始线终点
				SlipDistanceInformation.push_back(ROIRectA);//滑移线显示界面ROI区域左上角
				SlipDistanceInformation.push_back(Point(turnright_x, ROIRectB.y));//滑移线显示界面ROI区域右下角
				break;
			case 2://向上
				SlipDistanceInformation.push_back(Point(ROIRectA.x, ROIRectA.y - InputSlipPixel));//滑移线起点
				SlipDistanceInformation.push_back(Point(ROIRectB.x, ROIRectA.y - InputSlipPixel));//滑移线终点
				SlipDistanceInformation.push_back(Point(ROIRectA.x, ROIRectA.y));//起始线起点
				SlipDistanceInformation.push_back(Point(ROIRectB.x, ROIRectA.y));//起始线终点
				SlipDistanceInformation.push_back(Point(ROIRectA.x, turnup_y));//滑移线显示界面ROI区域左上角
				SlipDistanceInformation.push_back(ROIRectB);//滑移线显示界面ROI区域右下角
				break;
			case 3://向左
				SlipDistanceInformation.push_back(Point(ROIRectA.x - InputSlipPixel, ROIRectA.y));//滑移线起点
				SlipDistanceInformation.push_back(Point(ROIRectA.x - InputSlipPixel, ROIRectB.y));//滑移线终点
				SlipDistanceInformation.push_back(Point(ROIRectA.x , ROIRectA.y));//起始线起点
				SlipDistanceInformation.push_back(Point(ROIRectA.x , ROIRectB.y));//起始线终点
				SlipDistanceInformation.push_back(Point(turnleft_x, ROIRectA.y));//滑移线显示界面ROI区域左上角
				SlipDistanceInformation.push_back(ROIRectB);//滑移线显示界面ROI区域右下角
				break;
			case 4://向下
				SlipDistanceInformation.push_back(Point(ROIRectA.x, ROIRectB.y + InputSlipPixel));//滑移线起点
				SlipDistanceInformation.push_back(Point(ROIRectB.x, ROIRectB.y + InputSlipPixel));//滑移线终点
				SlipDistanceInformation.push_back(Point(ROIRectA.x, ROIRectB.y ));//起始线起点
				SlipDistanceInformation.push_back(Point(ROIRectB.x, ROIRectB.y ));//起始线终点
				SlipDistanceInformation.push_back(ROIRectA);//滑移线显示界面ROI区域左上角
				SlipDistanceInformation.push_back(Point(ROIRectB.x, turndown_y));//滑移线显示界面ROI区域右下角
				break;
			default:
				break;
			}
		}

}


int NeedleMarkDll::Probe_Angle_move(const Mat& Img, const vector<Point>& ProbeSlipDirect,
	const Rect& g_Temp_ROI_Rect,const double movepixel, vector<Point>& rotaionmoveline)
{
	if (ProbeSlipDirect.empty() || movepixel == 0 || g_Temp_ROI_Rect.empty() || Img.empty())
		return 0;
	//转向向量x方向相等时,往y方向画侧移线
	if (ProbeSlipDirect[0].x == ProbeSlipDirect[2].x)
	{	//移动像素为正，表示位移与转动向量一致
		if (movepixel > 0)
		{
			//向上画线
			if (ProbeSlipDirect[0].y > ProbeSlipDirect[2].y)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.tl().y - movepixel) < 4)
				{
					if ((g_Temp_ROI_Rect.tl().y - 0.5 * movepixel) > 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.tl().y - 0.5 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.tl().y - 0.5 * movepixel));
					//转动起使直线
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.tl().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.tl().y));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.tl().y - 0.25 * movepixel) > 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.tl().y - 0.25 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.tl().y - 0.25 * movepixel));
						//转动起使直线
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.tl().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.tl().y));
						return 4;
					}
					else
						return -1;
				}
				//在范围内，画一次线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(0, g_Temp_ROI_Rect.tl().y - movepixel));
					rotaionmoveline.push_back(
						Point(Img.cols, g_Temp_ROI_Rect.tl().y - movepixel));
					//转动起使直线
					rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.tl().y));
					rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.tl().y));
					return 1;
				}

			}
			//向下画线
			else if (ProbeSlipDirect[0].y < ProbeSlipDirect[2].y)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.br().y + movepixel) > (Img.rows - 4))
				{
					if ((g_Temp_ROI_Rect.br().y + 0.5 * movepixel) < (Img.rows - 4))
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.br().y + 0.5 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.br().y + 0.5 * movepixel));
					//转动起点直线
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.br().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.br().y ));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.br().y + 0.25 * movepixel) < (Img.rows - 4))
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.br().y + 0.25 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.br().y + 0.25 * movepixel));
						//转动起点直线
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.br().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.br().y));
						return 4;
					}
					else
						return -1;
				}
				//未超出范围，一次画线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(0, g_Temp_ROI_Rect.br().y + movepixel));
					rotaionmoveline.push_back(
						Point(Img.cols, g_Temp_ROI_Rect.br().y + movepixel));
					//转动起点直线
					rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.br().y));
					rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.br().y));
					return 1;
				}
			}
		}
		//移动像素为负，表示位移与转动向量相反
		else
		{
			//向下画线
			if (ProbeSlipDirect[0].y > ProbeSlipDirect[2].y)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.br().y - movepixel) < Img.rows - 4)
				{
					if ((g_Temp_ROI_Rect.br().y - 0.5 * movepixel) < Img.rows - 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.br().y - 0.5 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.br().y - 0.5 * movepixel));
					//转动起点直线
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.br().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.br().y));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.br().y - 0.25 * movepixel) < Img.rows - 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.br().y - 0.25 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.br().y - 0.25 * movepixel));
						//转动起点直线
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.br().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.br().y));
						return 4;
					}
					else
						return -1;
				}
				//在范围内，只画一次线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(0, g_Temp_ROI_Rect.br().y - movepixel));
					rotaionmoveline.push_back(
						Point(Img.cols, g_Temp_ROI_Rect.br().y - movepixel));
					//转动起点直线
					rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.br().y));
					rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.br().y));
					return 1;
				}

			}
			//向上画线
			else if (ProbeSlipDirect[0].y < ProbeSlipDirect[2].y)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.tl().y + movepixel) < 4)
				{
					if ((g_Temp_ROI_Rect.tl().y + 0.5 * movepixel) < 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.tl().y + 0.5 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.tl().y + 0.5 * movepixel));
					//转动起点直线	
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.tl().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.tl().y ));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.tl().y + 0.25 * movepixel) < 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(0, g_Temp_ROI_Rect.tl().y + 0.25 * movepixel));
						rotaionmoveline.push_back(
							Point(Img.cols, g_Temp_ROI_Rect.tl().y + 0.25 * movepixel));
						//转动起点直线	
						rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.tl().y));
						rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.tl().y));
						return 4;
					}
					else
						return -1;
				}
				//未超出范围，一次画线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(0, g_Temp_ROI_Rect.tl().y + movepixel));
					rotaionmoveline.push_back(
						Point(Img.cols, g_Temp_ROI_Rect.tl().y + movepixel));
					//转动起点直线	
					rotaionmoveline.push_back(Point(0, g_Temp_ROI_Rect.tl().y));
					rotaionmoveline.push_back(Point(Img.cols, g_Temp_ROI_Rect.tl().y));
					return 1;
				}
			}
		}
	}
	//转向向量y方向相等时，往x方向画侧移线
	else if (ProbeSlipDirect[0].y == ProbeSlipDirect[2].y)
	{//移动像素为正，表示位移与转动向量一致
		if (movepixel > 0)
		{
			//向左画线
			if (ProbeSlipDirect[0].x > ProbeSlipDirect[2].x)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.tl().x - movepixel) < 4)
				{
					if ((g_Temp_ROI_Rect.tl().x - 0.5 * movepixel) > 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x - 0.5 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x - 0.5 * movepixel, Img.rows));
					//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, Img.rows));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.tl().x - 0.25 * movepixel) > 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x - 0.25 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x - 0.25 * movepixel, Img.rows));
						//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, Img.rows));
						return 4;
					}
					else
						return -1;
				}
				//在范围内，画一次线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.tl().x - movepixel, 0));
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.tl().x - movepixel, Img.rows));
					//转动起点直线
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, 0));
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, Img.rows));
					return 1;
				}

			}
			//向右画线
			else if (ProbeSlipDirect[0].x < ProbeSlipDirect[2].x)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.br().x + movepixel) > (Img.cols - 4))
				{
					if ((g_Temp_ROI_Rect.br().x + 0.5 * movepixel) < (Img.cols - 4))
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x + 0.5 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x + 0.5 * movepixel, Img.rows));
						//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, Img.rows));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.br().x + 0.25 * movepixel) < (Img.cols - 4))
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x + 0.25 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x + 0.25 * movepixel, Img.rows));
						//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, Img.rows));
						return 4;
					}
					else
						return -1;
				}
				//在范围内，画一次线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.br().x + movepixel, 0));
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.br().x + movepixel, Img.rows));
					//转动起点直线
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, 0));
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, Img.rows));
					return 1;
				}

			}
		}
		//移动像素为负，表示位移与转动向量相反
		else
		{
			//向右画线
			if (ProbeSlipDirect[0].x > ProbeSlipDirect[2].x)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.br().x - movepixel) < Img.cols -4)
				{
					if ((g_Temp_ROI_Rect.br().x - 0.5 * movepixel) < Img.cols - 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x - 0.5 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x - 0.5 * movepixel, Img.rows));
						//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, Img.rows));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.br().x - 0.25 * movepixel) < Img.cols - 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x - 0.25 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.br().x - 0.25 * movepixel, Img.rows));
						//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, Img.rows));
						return 4;
					}
					else
						return -1;
				}
				//在范围内，画一次线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.br().x - movepixel, 0));
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.br().x - movepixel, Img.rows));
					//转动起点直线
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, 0));
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.br().x, Img.rows));
					return 1;
				}

			}
			//向左画线
			else if (ProbeSlipDirect[0].x < ProbeSlipDirect[2].x)
			{	//判断画线是否会超出范围
				if ((g_Temp_ROI_Rect.tl().x + movepixel) < 4)
				{
					if ((g_Temp_ROI_Rect.tl().x + 0.5 * movepixel) < 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x + 0.5 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x + 0.5 * movepixel, Img.rows));
					//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, Img.rows));
						return 2;
					}
					else if ((g_Temp_ROI_Rect.tl().x + 0.25 * movepixel) < 4)
					{//转动目标直线
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x + 0.25 * movepixel, 0));
						rotaionmoveline.push_back(
							Point(g_Temp_ROI_Rect.tl().x + 0.25 * movepixel, Img.rows));
						//转动起点直线
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, 0));
						rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, Img.rows));
						return 4;
					}
					else
						return -1;
				}
				//在范围内，画一次线
				else
				{//转动目标直线
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.tl().x + movepixel, 0));
					rotaionmoveline.push_back(
						Point(g_Temp_ROI_Rect.tl().x + movepixel, Img.rows));
					//转动起点直线
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, 0));
					rotaionmoveline.push_back(Point(g_Temp_ROI_Rect.tl().x, Img.rows));
					return 1;
				}

			}
		}
	}

}

Mat NeedleMarkDll::GetProbeMarkArea(Mat srcImg, int Towards, double PixelSize, vector<int>& SumArea, vector<Rect>& ActualRect)
{

	Mat resultImg;//输出图像
	int height = srcImg.rows;
	int width = srcImg.cols;

	//寻找探针针尖边界直线
	vector<Point2f>SumPosition;//存放直线点坐标
	SumPosition = GetProbeTipLine(srcImg, Towards, PixelSize);
	if (SumPosition.size() == 0)
	{
		return srcImg;//可改为返回错误
	}

	vector<float> SumLinePositionX;//存放探针直线点X坐标(去噪前)
	vector<float> SumLinePositionY;//存放探针直线点Y坐标(去噪前)
	float num = SumPosition.size();

	for (int i = 0; i < num; i++)
	{
		SumLinePositionX.push_back(SumPosition[i].x);
		SumLinePositionY.push_back(SumPosition[i].y);
	}

	sort(SumLinePositionX.begin(), SumLinePositionX.end());//将X坐标从小到大排序
	sort(SumLinePositionY.begin(), SumLinePositionY.end());//将Y坐标从小到大排序

	// 针痕相对分布
	//int Sub_1 = 75 / (1000 * PixelSize);//根据差值判断针尖,0-75为针尖一（待调）
	//int Sub_2 = 125 / (1000 * PixelSize);//根据差值判断125-235为针尖二区域（待调）
	//int Sub_3 = 235 / (1000 * PixelSize);
	//int Sub_4 = 265 / (1000 * PixelSize); //根据差值判断265-375为针尖二区域（待调）
	//int Sub_5 = 375 / (1000 * PixelSize);
	int Sub_1 = 75 / (1000 * PixelSize);//根据差值判断针尖,0-75为针尖一（待调）
	int Sub_2 = 105 / (1000 * PixelSize);//根据差值判断125-235为针尖二区域（待调）
	int Sub_3 = 195 / (1000 * PixelSize);
	int Sub_4 = 215 / (1000 * PixelSize); //根据差值判断265-375为针尖二区域（待调）
	int Sub_5 = 375 / (1000 * PixelSize);

	if (Towards == 1 || Towards == 2)//探针竖直方向，针尖直线为水平方向，根据X坐标判断针尖位置
	{
		vector<float>LinePositionX;
		NoiseRemoval(SumLinePositionX, Towards, PixelSize, LinePositionX);
		float VectorSize = LinePositionX.size();
		if (LinePositionX[VectorSize - 1] - LinePositionX[0] < Sub_4)
		{
			return srcImg;//未检测到三根针尖，返回
		}

		//寻找探针针尖竖直方向边界,三根针尖分开计算
		vector<float> vPositionXone;     //存储直线坐标点，共三组
		vector<float> vPositionXtwo;
		vector<float> vPositionXthree;

		//三根针尖六条边界坐标
		int x1, x2, x3, x4, x5, x6;
		x1 = x3 = x5 = 999;
		x2 = x4 = x6 = 0;
		int Tip1, Tip2, Tip3;
		Tip1 = Tip2 = Tip3 = 0;


		for (int i = 1; i < LinePositionX.size(); i++)
		{
			int SubX = abs(LinePositionX[i] - LinePositionX[0]);//从直线坐标中取出第一点的横坐标X，比较后续坐标横坐标大小


			if (SubX < Sub_1)//差值小于Sub_1，判断为和第一点相同针尖
			{
				vPositionXone.push_back(LinePositionX[i]);
				if (LinePositionX[i] < x1)
				{
					x1 = LinePositionX[i];
				}
				else if (LinePositionX[i] > x2)
				{
					x2 = LinePositionX[i];
				}
				Tip1 = 1;
			}
			else if (SubX > Sub_2 && SubX < Sub_3)//差值处于Sub_2到Sub_3则判断为针尖2
			{

				vPositionXtwo.push_back(LinePositionX[i]);
				if (LinePositionX[i] < x3)
				{
					x3 = LinePositionX[i];
				}
				else if (LinePositionX[i] > x4)
				{
					x4 = LinePositionX[i];
				}
				Tip2 = 1;
			}
			else if (SubX > Sub_4 && SubX < Sub_5)//差值处于Sub_4到Sub_5则判断为针尖3
			{

				vPositionXthree.push_back(LinePositionX[i]);
				if (LinePositionX[i] < x5)
				{
					x5 = LinePositionX[i];
				}
				else if (LinePositionX[i] > x6)
				{
					x6 = LinePositionX[i];
				}
				Tip3 = 1;
			}
			//else return srcImg;//若首坐标点不满足上述条件，函数运行错误，返回
		}

		if (!(Tip1 == 1 && Tip2 == 1 && Tip3 == 1))
		{
			return srcImg;
		}


		float y0 = SumLinePositionY[0];//探针直线纵坐标
		float h0, y1;//h0探针针尖区域图像高度,y1抓取区域左上角顶点纵坐标
		if (Towards == 1)//探针朝下
		{
			h0 = height - y0;//探针朝下则是图像高度减直线纵坐标
			y1 = y0;
		}
		else
		{
			h0 = y0 + 1;//探针朝上则刚好为直线纵坐标,+1防止高度为0
			y1 = 0;
		}
		srcImg.copyTo(resultImg);

		Rect Rect1, Rect2, Rect3;
		Mat img1, img2, img3;
		if (x1 > 10 && x6 < width - 10)//防止多出部分超过边界
		{
			//抓取针尖前端区域
			Rect1 = Rect(x1 - 10, y1, x2 - x1 + 20, h0);//针尖1前端矩形区域
			img1 = resultImg(Rect1);

			Rect2 = Rect(x3 - 10, y1, x4 - x3 + 20, h0);//针尖2前端矩形区域
			img2 = resultImg(Rect2);

			Rect3 = Rect(x5 - 10, y1, x6 - x5 + 20, h0);//针尖3前端矩形区域
			img3 = resultImg(Rect3);
		}
		else
		{
			//抓取针尖前端区域
			Rect1 = Rect(x1, y1, x2 - x1 + 10, h0);//针尖1前端矩形区域
			img1 = resultImg(Rect1);

			Rect2 = Rect(x3 - 10, y1, x4 - x3 + 20, h0);//针尖2前端矩形区域
			img2 = resultImg(Rect2);

			Rect3 = Rect(x5 - 10, y1, x6 - x5, h0);//针尖3前端矩形区域
			img3 = resultImg(Rect3);
		}






		//获取抓取到的三个区域针痕面积，及区域矩形框坐标

		int area1, area2, area3;

		GetArea(img1, PixelSize, area1);
		ActualRect.push_back(Rect1);
		SumArea.push_back(area1);

		GetArea(img2, PixelSize, area2);
		ActualRect.push_back(Rect2);
		SumArea.push_back(area2);

		GetArea(img3, PixelSize, area3);
		ActualRect.push_back(Rect3);
		SumArea.push_back(area3);



		for (int i = 0; i < ActualRect.size(); i++)//显示合并后的大矩形框，序号以及针痕的面积
		{
			rectangle(resultImg, ActualRect[i].tl(), ActualRect[i].br(), Scalar(0, 0, 255), 2, 8, 0);//画出大矩形框
			if (Towards == 1)
			{
				putText(resultImg, to_string(SumArea[i]), Point(ActualRect[i].x - 10, ActualRect[i].y - 10), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 2);//显示针痕面积
			}
			else
				putText(resultImg, to_string(SumArea[i]), Point(ActualRect[i].x - 10, ActualRect[i].y + ActualRect[i].height), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 2);//显示针痕面积
		}




	}
	if (Towards == 3 || Towards == 4)//探针水平方向，针尖直线为竖直方向，根据y坐标判断针尖位置
	{
		vector<float>LinePositionY;
		NoiseRemoval(SumLinePositionY, Towards, PixelSize, LinePositionY);

		float VectorSize = LinePositionY.size();
		if (LinePositionY[VectorSize - 1] - LinePositionY[0] < Sub_3)
		{
			return srcImg;//未检测到三根针尖，返回
		}


		//寻找探针针尖竖直方向边界,三根针尖分开计算
		vector<float> vPositionYone;     //存储直线坐标点，共三组
		vector<float> vPositionYtwo;
		vector<float> vPositionYthree;

		//三根针尖六条边界坐标
		int y1, y2, y3, y4, y5, y6;
		y1 = y3 = y5 = 999;
		y2 = y4 = y6 = 0;

		//判断是否识别到三根针尖
		int Tip1, Tip2, Tip3;
		Tip1 = Tip2 = Tip3 = 0;

		for (int i = 1; i < LinePositionY.size(); i++)
		{
			int SubY = abs(LinePositionY[i] - LinePositionY[0]);//从直线坐标中取出第一点的纵坐标Y，比较后续坐标纵坐标大小

			if (SubY < Sub_1)//差值小于Sub_1，判断为和第一点相同针尖
			{
				vPositionYone.push_back(LinePositionY[i]);
				if (LinePositionY[i] < y1)
				{
					y1 = LinePositionY[i];
				}
				else if (LinePositionY[i] > y2)
				{
					y2 = LinePositionY[i];
				}
				Tip1 = 1;
			}
			else if (SubY > Sub_2 && SubY < Sub_3)//差值处于Sub_2到Sub_3则判断为针尖2
			{
				vPositionYtwo.push_back(LinePositionY[i]);
				if (LinePositionY[i] < y3)
				{
					y3 = LinePositionY[i];
				}
				else if (LinePositionY[i] > y4)
				{
					y4 = LinePositionY[i];
				}
				Tip2 = 1;
			}
			else if (SubY > Sub_4 && SubY < Sub_5)//差值处于Sub_4到Sub_5则判断为针尖3
			{

				vPositionYthree.push_back(LinePositionY[i]);
				if (LinePositionY[i] < y5)
				{
					y5 = LinePositionY[i];
				}
				else if (LinePositionY[i] > y6)
				{
					y6 = LinePositionY[i];
				}
				Tip3 = 1;
			}
			else return srcImg;//若首坐标点不满足上述条件，函数运行错误，返回
		}


		if (!(Tip1 == 1 && Tip2 == 1 && Tip3 == 1))
		{
			return srcImg;
		}


		float x0 = SumLinePositionX[0];//探针直线横坐标
		float w0, x1;//w0探针针尖区域图像宽度,x1抓取区域左上角顶点横坐标
		if (Towards == 3)//探针朝右
		{
			w0 = width - x0;//探针朝右则是图像宽度减直线横坐标
			x1 = x0;
		}
		else
		{
			w0 = x0 + 1;//探针朝左则刚好为直线横坐标,+1防止宽度为0
			x1 = 0;
		}
		srcImg.copyTo(resultImg);

		Rect Rect1, Rect2, Rect3;
		Mat img1, img2, img3;
		if (y1 > 10 && y6 < height - 10)//防止多出部分超过边界
		{
			//抓取针尖前端区域
			Rect1 = Rect(x1, y1 - 10, w0, y2 - y1 + 20);//针尖1前端矩形区域
			img1 = resultImg(Rect1);

			Rect2 = Rect(x1, y3 - 10, w0, y4 - y3 + 20);//针尖2前端矩形区域
			img2 = resultImg(Rect2);

			Rect3 = Rect(x1, y5 - 10, w0, y6 - y5 + 20);//针尖3前端矩形区域
			img3 = resultImg(Rect3);
		}
		else
		{
			//抓取针尖前端区域
			Rect1 = Rect(x1, y1, w0, y2 - y1 + 10);//针尖1前端矩形区域
			img1 = resultImg(Rect1);

			Rect2 = Rect(x1, y3 - 10, w0, y4 - y3 + 20);//针尖2前端矩形区域
			img2 = resultImg(Rect2);

			Rect3 = Rect(x1, y5 - 10, w0, y6 - y5);//针尖3前端矩形区域
			img3 = resultImg(Rect3);
		}





		//将抓取到的三个区域针痕面积及矩形框位置转换到原图像上

		int area1, area2, area3;

		GetArea(img1, PixelSize, area1);
		ActualRect.push_back(Rect1);
		SumArea.push_back(area1);

		GetArea(img2, PixelSize, area2);
		ActualRect.push_back(Rect2);
		SumArea.push_back(area2);

		GetArea(img3, PixelSize, area3);
		ActualRect.push_back(Rect3);
		SumArea.push_back(area3);


		if (SumArea.size() == 3 && !(SumArea[0] == 0 && SumArea[1] == 0 && SumArea[2] == 0))
		{
			for (int i = 0; i < ActualRect.size(); i++)//显示抓取区域矩形框以及针痕的面积
			{
				rectangle(resultImg, ActualRect[i].tl(), ActualRect[i].br(), Scalar(0, 0, 255), 2, 8, 0);//画出大矩形框
				if (Towards == 3)
				{
					putText(resultImg, to_string(SumArea[i]), Point(ActualRect[i].x - ActualRect[i].width, ActualRect[i].y + 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 2);//显示针痕面积
				}
				else
					putText(resultImg, to_string(SumArea[i]), Point(ActualRect[i].x + ActualRect[i].width, ActualRect[i].y + 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 2);//显示针痕面积
			}
		}


	}


	return resultImg;

}


std::string NeedleMarkDll::DoubleToStr(double num)
{
	ostringstream out;
	//设置精度
	out.precision(3);
	out << num;
	return out.str();
}

int NeedleMarkDll::GetArea(Mat srcImg, double PixelSize, int& TemArea)
{


	Mat dstImg;
	srcImg.copyTo(dstImg);

	int imgCols = srcImg.cols;
	int imgRows = srcImg.rows;
	Mat BlackImg = Mat(imgRows, imgCols, CV_8UC1, 120);//120为灰度值，根据实际情况调整
	//imshow("BlackImg", BlackImg);


	Mat imgGray, imgBlur, imgSub, imgThr, imgMorOpen, imgMorClose, imgGauBlur, imgSubThr;
	cvtColor(dstImg, imgGray, COLOR_BGR2GRAY);//灰度化
	GaussianBlur(imgGray, imgGauBlur, Size(7, 7), 3, 3);//高斯滤波
	addWeighted(BlackImg, -1, imgGauBlur, 1, 0, imgSub);//做差 
	threshold(imgSub, imgThr, 80, 255, THRESH_BINARY);//二值化,阈值根据实际情况调整
	Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
	morphologyEx(imgThr, imgThr, MORPH_OPEN, element1);//开操作，先腐蚀再膨胀


	//绘制轮廓
	vector<vector<Point>>contours;
	vector<Vec4i>hierachy;
	vector<Rect>rects;

	findContours(imgThr, contours, hierachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//根据图像找到轮廓

	TemArea = 0;

	for (size_t t = 0; t < contours.size(); t++)//计算图像上大于15的轮廓面积
	{
		int area = contourArea(contours[t]);

		if (area < 35 / (1000 * PixelSize))
		{
			TemArea = TemArea;
		}
		else TemArea = TemArea + area;
	}


	return 0;
}

vector<Point2f> NeedleMarkDll::GetProbeTipLine(Mat srcImg, int Towards, double PixelSize)
{
	//寻找探针针尖边界直线
	Mat imgGray, imgGauBlur, imgThr, imgOpen, imgCanny;
	cvtColor(srcImg, imgGray, COLOR_BGR2GRAY);
	GaussianBlur(imgGray, imgGauBlur, Size(7, 7), 7, 7);//高斯滤波
	threshold(imgGauBlur, imgThr, 90, 255, THRESH_BINARY);//二值化
	Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
	morphologyEx(imgThr, imgOpen, MORPH_OPEN, element1);//开操作，先腐蚀再膨胀
	//Canny(imgOpen, imgCanny, 25, 75);


	//寻找探针针尖直线参数
	int height = srcImg.rows;
	int width = srcImg.cols;
	vector<Point2f>SumPosition;//存放最终直线点坐标
	vector<Point2f>TemPosition;//存放临时直线点坐标
	int temline = 0;//存放疑似边界的直线个数，连续两条直线满足要求则判断为边界

	int LineNum = 4;//n=5

	int numberToLine = 150 / (1000 * PixelSize);//一行或一列超过此个数判断为探针针尖直线(探针针尖宽度约为50um)

	if (Towards == 1)////探针朝下
	{
		for (int i = height - 1; i > 0; i--)
		{
			for (int j = 0; j < width; j++)
			{
				float value = imgOpen.at<uchar>(i, j);
				if (value == 0)
				{
					TemPosition.push_back(Point(j, i));
				}

			}
			if (TemPosition.size() < numberToLine)
			{
				TemPosition.clear();
			}
			else
			{
				SumPosition.insert(SumPosition.end(), TemPosition.begin(), TemPosition.end());
				TemPosition.clear();
				temline = temline + 1;
			}
			if (temline == LineNum)
			{
				break;
			}

		}


	}
	else if (Towards == 2)////探针朝上
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				float value = imgOpen.at<uchar>(i, j);
				if (value == 0)
				{
					TemPosition.push_back(Point(j, i));
				}

			}
			if (TemPosition.size() < numberToLine)
			{
				TemPosition.clear();
			}
			else
			{
				SumPosition.insert(SumPosition.end(), TemPosition.begin(), TemPosition.end());
				TemPosition.clear();
				temline = temline + 1;
			}
			if (temline == LineNum)
			{
				break;
			}

		}

	}
	else if (Towards == 3)////探针朝右
	{
		for (int j = width - 1; j > 0; j--)
		{
			for (int i = 0; i < height; i++)
			{
				float value = imgOpen.at<uchar>(i, j);
				if (value == 0)
				{
					TemPosition.push_back(Point(j, i));
				}

			}
			if (TemPosition.size() < numberToLine)
			{
				TemPosition.clear();
			}
			else
			{
				SumPosition.insert(SumPosition.end(), TemPosition.begin(), TemPosition.end());
				TemPosition.clear();
				temline = temline + 1;
			}
			if (temline == LineNum)
			{
				break;
			}
		}

	}
	else if (Towards == 4)////探针朝左
	{
		for (int j = 0; j < width; j++)
		{
			for (int i = 0; i < height; i++)
			{
				float value = imgOpen.at<uchar>(i, j);
				if (value == 0)
				{
					TemPosition.push_back(Point(j, i));
				}

			}
			if (TemPosition.size() < numberToLine)
			{
				TemPosition.clear();
			}
			else
			{
				SumPosition.insert(SumPosition.end(), TemPosition.begin(), TemPosition.end());
				TemPosition.clear();
				temline = temline + 1;
			}
			if (temline == LineNum)
			{
				break;
			}

		}


	}

	return SumPosition;
}

int NeedleMarkDll::NoiseRemoval(vector<float>LinePositionXY, int Towards, double PixelSize, vector<float>& LineXYOutPut)
{
	if (LinePositionXY.size() == 0 || PixelSize == 0)//若传入参数有问题返回1
	{
		return 1;
	}

	vector<vector<int>>PoisitionGroup;
	vector<int>PoisitionTem;
	float sub = 0;

	PoisitionTem.push_back(LinePositionXY[0]);
	for (int i = 1; i < LinePositionXY.size(); i++)
	{
		sub = LinePositionXY[i] - LinePositionXY[i - 1];
		if (sub <= 1)
		{
			PoisitionTem.push_back(LinePositionXY[i]);
		}
		else
		{
			PoisitionGroup.push_back(PoisitionTem);
			PoisitionTem.clear();
			PoisitionTem.push_back(LinePositionXY[i]);
		}
	}

	PoisitionGroup.push_back(PoisitionTem);


	for (int i = 0; i < PoisitionGroup.size(); i++)
	{
		if (PoisitionGroup[i].size() > (50 / (1000 * PixelSize)))//如果一组内点数大于规定值，判定为有效值，否则为噪点
		{
			LineXYOutPut.insert(LineXYOutPut.end(), PoisitionGroup[i].begin(), PoisitionGroup[i].end());
		}
	}

	return 0;
}

