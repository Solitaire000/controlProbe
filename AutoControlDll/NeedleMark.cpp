#pragma execution_character_set("utf-8")
#include "NeedleMark.h"
//#include "Global_define.h"



//模板匹配相关变量
bool bOutArea = true;//判断框选部分是否超出区域，0正常，1超出。
Mat g_srcImg;//源图像
Mat g_tmpImg;//实时显示图像
Mat g_dstImg;//框选后的输出图片
Mat g_TemplateImg;//模板图片
Rect g_Temp_ROI_Rect;//模板匹配后获取的ROI区域  Point信息（可直接用于Opencv的rectangle函数）
vector<Point> g_ProbeSlipDirect;//探针转向的起点，探针朝向的起点坐标，终点为图片中心点坐标
vector<Point> g_ProbeSlipLine;//滑移参考线hqw
vector<Point>g_rotaionmoveline;//角度调整参考线hqw
double g_ProbeAngleMicrRatio;//角度微米比
double g_movepixel;//角度调整侧移像素
int g_nTurnNum;//1为顺时针，0为逆时针
double g_dPixelSize;//单位mm	,		g_dPixelSizeX
HWND g_hwndShowImg;//显示图像窗口句柄
bool g_bRButton = 0;//鼠标右键状态
bool b_bFinishInitProbe = 0;
Mat g_mInitialImg1, g_mInitialImg2;
bool g_brotaionmove = false;//判断是否画侧移线（探针转角）
bool g_bslipmove = false;//判断是否画滑移线
int g_nOutputnum;//针痕识别和转角调整时用
Mat g_AngleOutput;
Mat g_tempResultImg;



NeedleMark::NeedleMark(QObject *parent)
	: QObject(parent)
{
	m_pNeedleMark = new NeedleMarkDll;
}

NeedleMark::~NeedleMark()
{
	if (m_pNeedleMark != NULL)
	{
		m_pNeedleMark = NULL;
		delete m_pNeedleMark;
	}
}
int NeedleMark::GetNeddleImg(std::string imgpath)
{
	g_srcImg = imread(imgpath);

	if (g_srcImg.empty())
	{
		return 1;
	}
	return 0;

}
int NeedleMark::FinishInitProbe(int probe)
{
	vector<Point> Orient_rotat_param;//存点坐标，ROI坐标，探针朝向，转向
	vector<double> Pixel_Angle_ratio;
	Pixel_Angle_ratio.push_back(g_dPixelSize);//存入单个像素大小
	g_mInitialImg2 = g_srcImg.clone();	//获取一帧照片
	if (!g_mInitialImg2.empty())//如果为空则不进行操作
	{
		Mat InitialProbe = g_mInitialImg1.clone();//将探针初始位置图片赋值
		Mat Imgshow = g_mInitialImg1.clone();
		cvtColor(InitialProbe, InitialProbe, COLOR_BGR2GRAY);
		cvtColor(g_mInitialImg2, g_mInitialImg2, COLOR_BGR2GRAY);
		int rotat_direct_num = m_pNeedleMark->Img_rotat_direct(InitialProbe, g_mInitialImg2,
			Orient_rotat_param, Pixel_Angle_ratio);
		if (rotat_direct_num == 1)
		{
			//根据探针编号给模板变量赋值
			string ProbeInformationAddress, ProbeTempAddress;
			QString ProbeName;
			if (probe == 1)
			{
				ProbeName = "1号探针初始化完毕";
				ProbeName = "1号探针初始化完毕";
				ProbeInformationAddress =
					"../data/autocontrol/photos/template/Probe_One_Information.csv";
				ProbeTempAddress =
					"../data/autocontrol/photos/template/Probe_One_Temp.jpg";
			}
			else if (probe == 2)
			{
				ProbeName = "2号探针初始化完毕";
				ProbeInformationAddress =
					"../data/autocontrol/photos/template/Probe_Two_Information.csv";
				ProbeTempAddress =
					"../data/autocontrol/photos/template/Probe_Two_Temp.jpg";
			}
			//存探针信息
			ofstream ProbeInformationFile(ProbeInformationAddress, ios::trunc);
			if (!ProbeInformationFile.is_open()) {
				//ui.textEdit_Output->setText("探针信息文件打开失败，请检查！！！");
				return 2;
			}
			else
			{
				ProbeInformationFile << Orient_rotat_param[2].x
					<< "," << Orient_rotat_param[2].y
					<< "," << "Starting point of Rotation direction" << endl;//转动方向向量起点
				ProbeInformationFile << Orient_rotat_param[4].x
					<< "," << Orient_rotat_param[4].y
					<< "," << "Starting point of towards direction" << endl;//探针朝向起点
				ProbeInformationFile << Orient_rotat_param[3].x
					<< "," << Orient_rotat_param[3].y
					<< "," << "ending point of Rotation and towards direction" << endl;//向量终点
				ProbeInformationFile << Pixel_Angle_ratio[1] << "," << "degree/per pixel" << "," <<
					"Pixel_Angle_ratio(how degree per pixel)" << endl;
			}
			ProbeInformationFile.close();
			//存模板
			Mat ProbeTemp = Imgshow(Rect(Orient_rotat_param[0], Orient_rotat_param[1]));
			g_TemplateImg = ProbeTemp.clone();
			imwrite(ProbeTempAddress, ProbeTemp);

			QImage q_ProbeTemp = m_pNeedleMark->mat_to_qim(g_TemplateImg);
			return 0;
		}
		else if (rotat_direct_num == -1)
		{
			//ui.textEdit_Output->setText("转动前后照片大小不一致！！！");
			return 3;
		}
		else if (rotat_direct_num == 0)
		{
			//ui.textEdit_Output->setText("探针转动量过小！！！");
			return 4;
		}
		else if (rotat_direct_num == -2)
		{
			//ui.textEdit_Output->setText("图片亮度变化或抖动过大，请重新初始化！！！");
			return 5;
		}
		else if (rotat_direct_num == -3)
		{
			//ui.textEdit_Output->setText("Mat temp = imgvect1[0](Rect(OutputParam[0], OutputParam[1])); 构造失败");
			return 14;
		}
	}
	else {
		//ui.textEdit_Output->setText("探针初始化失败！！！");
		return 6;
	}
		
}
int NeedleMark::GetProbeTowards(int probe)
{
	g_ProbeSlipDirect.clear();//存入前将向量清空
	//读取探针朝向信息
	string ProbeInformationAddress;
	Point ProbeSlipPoint;
	string Probe_Information_line, Probe_Information_Str;
	istringstream Probe_Information_Sin;
	if (probe ==1)
		ProbeInformationAddress = "../data/autocontrol/photos/template/Probe_One_Information.csv";
	else
		ProbeInformationAddress = "../data/autocontrol/photos/template/Probe_Two_Information.csv";

	ifstream ProbeInformationFile(ProbeInformationAddress, ios::in);
	if (!ProbeInformationFile.is_open()) {
		return 1;
	}
	else
	{
		// ------------读取数据-----------------
		// 先找到数据所在行，以字符串读取
		// 使用getline按","进行分割
		// 使用istringstream将分割好的字符数据存入			
		for (int i = 0; i < 4; i++)
		{
			if (i != 3)
			{
				getline(ProbeInformationFile, Probe_Information_line);//从数据流中获取string数据，以换行符/n为中止				
				//将字符串流Probe_Information_line中的字符读到字符串数组Probe_Information_Str中，以逗号为分隔符
				Probe_Information_Sin.str(Probe_Information_line);
				//将字符串数据按“,”，进行分割
				getline(Probe_Information_Sin, Probe_Information_Str, ',');
				//存入x
				ProbeSlipPoint.x = atoi(Probe_Information_Str.c_str());
				//将字符串数据按“,”，进行分割
				getline(Probe_Information_Sin, Probe_Information_Str, ',');
				//存入y
				ProbeSlipPoint.y = atoi(Probe_Information_Str.c_str());
				g_ProbeSlipDirect.push_back(ProbeSlipPoint);
			}
			else
			{
				getline(ProbeInformationFile, Probe_Information_line);//从数据流中获取string数据，以换行符/n为中止				
				//将字符串流Probe_Information_line中的字符读到字符串数组Probe_Information_Str中，以逗号为分隔符
				Probe_Information_Sin.str(Probe_Information_line);
				//将字符串数据按“,”，进行分割
				getline(Probe_Information_Sin, Probe_Information_Str, ',');
				//存入x
				g_ProbeAngleMicrRatio = atof(Probe_Information_Str.c_str());
			}
		}
	}
	ProbeInformationFile.close();
}
int NeedleMark::IdentifyProbe(int probe)
{
	Mat TemplateImg = g_srcImg.clone();//原图显示匹配区域
	if (TemplateImg.empty()) {
		return 5;
	}
	Mat srcImg, temImg;//原图像及原有模板图像
	g_srcImg.copyTo(srcImg);

	if (probe ==1)//选择探针1时读取探针1模板
	{
		temImg = imread("../data/autocontrol/photos/template/Probe_One_Temp.jpg");
		g_dstImg = temImg.clone();
		if (GetProbeTowards(probe) == 1)
			return 4;

	}
	else if (probe ==2)//选择探针2时读取探针2模板
	{
		temImg = imread("../data/autocontrol/photos/template/Probe_Two_Temp.jpg");
		g_dstImg = temImg.clone();
		if (GetProbeTowards(probe) == 1)
			return 4;
	}

	if (temImg.data == NULL)
	{
		//ui.textEdit_Output->setText("未找到模板图片！！！");
		return 1;
	}

	if (srcImg.cols > temImg.cols && srcImg.rows > temImg.rows)
		if (m_pNeedleMark->Img_Template(srcImg, temImg, g_Temp_ROI_Rect))
		{
			rectangle(TemplateImg, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
			//imshow("ShowImg", TemplateImg);//显示所框出的感兴趣区域
			g_tempResultImg = TemplateImg;
			return 0;
		}
		else
		{
			return 2;
		}
	else
	{
		return 3;
	}

}
int NeedleMark::Read_ImageParaXml()
{
	// QString strPath = GetDataDir();
	QString strPath = "";
	QFile file(strPath + "/ImagePara.xbt");
	QString strPos = "";
	QStringRef str;
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		return -1;
	}
	else
	{
		QXmlStreamReader stream(&file);
		stream.readNext();
		while (!stream.atEnd())
		{
			str = stream.name();
			////////////////工艺参数读取////////////////////

			if (str == "CCDCenterPosX")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.dCenterCCDPosX = strPos.toDouble();
			}
			if (str == "CCDCenterPosY")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.dCenterCCDPosY = strPos.toDouble();
			}

			if (str == "Pixel2PulseX")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.dPixel2PulseX = strPos.toDouble();
			}
			if (str == "Pixel2PulseY")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.dPixel2PulseY = strPos.toDouble();
			}
			if (str == "WaferAlignDis")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.dWaferAlignDistace = strPos.toDouble();
			}
			if (str == "CrossReferenceLine")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.nCrossReferenceLine = strPos.toInt();
			}
			if (str == "TemplateSavePath")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.strTemplateSavePath = strPos;
			}
			if (str == "RedGain")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.nRedGain = strPos.toInt();
			}
			if (str == "GreenGain")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.nGreenGain = strPos.toInt();
			}
			if (str == "BlueGain")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.nBlueGain = strPos.toInt();
			}
			if (str == "Saturation")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.nSaturation = strPos.toInt();
			}
			if (str == "Sharpness")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.nSharpness = strPos.toInt();
			}
			if (str == "Gain")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.nGain = strPos.toInt();
			}

			if (str == "ImageSaveFormat")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.strImageSaveFormat = strPos;
			}
			if (str == "CaptureInterval")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.dCaptureInterval = strPos.toDouble();
			}
			if (str == "ImageSavePath")
			{
				strPos = stream.readElementText();
				m_ImageparamCfg.strImageSavePath = strPos;
			}

			stream.readNext();
		}
	}
	return 0;
}

int NeedleMark::autoAngle(needleMarkLevelParas& configs, Mat SrcImg, double& angle, double& dx) {
	int probe = configs.probe;
	g_srcImg = SrcImg;
	/* 识别探针*/
	if (IdentifyProbe(probe) != 0)
		return 1;

	/* 识别针痕*/
	if (g_dstImg.data == NULL)
	{
		//ui.textEdit_Output->setText("未找到待测图片！！！");
		return 1;
	}
	//字符串转换
	QString Angleinform, probeNumber;
	Mat InputImg = g_srcImg.clone();
	Mat OutputImg;
	double ProbeMarkinform = 0;
	if (probe == 1)//根据选择提示探针号码
		probeNumber = QString::number(1);//1号
	else if (probe ==2)//根据选择提示探针号码
		probeNumber = QString::number(2);//2号
	//控制探针针痕识别范围 = 滑移距离 * 识别系数 * 单个像素大小
	Read_ImageParaXml();
	g_dPixelSize = configs.PixelSize; //m_ImageparamCfg.dPixel2PulseX;
	double GetMarkDistance = 
		configs.SlipDistance * configs.ProbeMark_Distance / (1000 * configs.PixelSize);//滑移距离 和 像素大小
	g_nOutputnum = m_pNeedleMark->DstProbeMarkArea(InputImg, g_ProbeSlipDirect,
		g_Temp_ROI_Rect, GetMarkDistance, g_dPixelSize, OutputImg, ProbeMarkinform);
	if (g_nOutputnum == -3)
	{
		return 2;
	}
	if (g_nOutputnum == -2)
	{
		return 3;
	}
	if (!OutputImg.empty())
		g_AngleOutput = OutputImg;

	// 字符串转换
	Angleinform = QString::number(abs(ProbeMarkinform), 'd', 5);
	if (g_ProbeAngleMicrRatio != 0 && g_dPixelSize != 0)
		g_movepixel = abs(ProbeMarkinform) / (g_ProbeAngleMicrRatio * g_dPixelSize * 1000);

	if (g_nOutputnum == -1)
	{
		return 4;
	}
	else if (g_nOutputnum == 0)
	{
		return 10;
	}
	else if (g_nOutputnum == 1)
	{
		angle = -Angleinform.toDouble();
		return 0;
	}
	else if (g_nOutputnum == 2)
	{
		angle = Angleinform.toDouble();
		g_movepixel = -g_movepixel;
		return 0;
	}
	/* 角度调整*/
	//Mat Img = g_srcImg.clone();//原图显示匹配区域
	//ui.pushButton_IdentifyProbeMark->setText("识别探针");
	//int return_num = m_pNeedleMark->Probe_Angle_move(Img, g_ProbeSlipDirect, g_Temp_ROI_Rect, g_movepixel, g_rotaionmoveline);
	//g_bslipmove = false;//将滑移线设为否
	//g_brotaionmove = true;//副屏显示转角线
	//QString proberotaiondirect, textshow;
	//if (ui.radioButton_No1_Probe->isChecked())//根据选择提示探针号码
	//	probeNumber = QString::number(1);//1号
	//else if (ui.radioButton_No2_Probe->isChecked())//根据选择提示探针号码
	//	probeNumber = QString::number(2);//2号	
	//if (g_nOutputnum == 1)
	//	proberotaiondirect = "请顺时针转动" + probeNumber + "号探针";
	//else if (g_nOutputnum == 2)
	//	proberotaiondirect = "请逆时针转动" + probeNumber + "号探针";
	//switch (return_num)
	//{
	//case -1:
	//	ui.textEdit_Output->setText("图像视野范围过小，1/4侧移线超出视野！！！");
	//	break;
	//case 0:
	//	ui.textEdit_Output->setText("滑移线绘制失败！！！");
	//	break;
	//case 1:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	textshow = +"移动至线段处！！";
	//	ui.textEdit_Output->setText(textshow);
	//	ui.pushButton_DrawReferLine->setEnabled(true);
	//	break;
	//case 2:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	textshow = +"两次移动至线段处！！";
	//	ui.textEdit_Output->setText(textshow);
	//	ui.pushButton_DrawReferLine->setEnabled(true);
	//	break;
	//case 4:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	ui.pushButton_DrawReferLine->setEnabled(true);
	//	textshow = +"四次移动至线段处！！";
	//	ui.textEdit_Output->setText(textshow);
	//	break;
	//default:
	//	break;
	//}


}
int NeedleMark::IdentifyMarke(needleMarkLevelParas& configs, double& angle, double& dx) {

	if (g_dstImg.data == NULL)
	{
		return 1;
	}
	//字符串转换
	QString Angleinform, probeNumber;
	Mat InputImg = g_srcImg.clone();
	Mat OutputImg;
	double ProbeMarkinform = 0;
	//控制探针针痕识别范围 = 滑移距离 * 识别系数 * 单个像素大小
	Read_ImageParaXml();
	g_dPixelSize = configs.PixelSize; //m_ImageparamCfg.dPixel2PulseX;
	double GetMarkDistance =  
		configs.SlipDistance * configs.ProbeMark_Distance/ (1000 * configs.PixelSize);//滑移距离 和 像素大小
	g_nOutputnum = m_pNeedleMark->DstProbeMarkArea(InputImg, g_ProbeSlipDirect,
		g_Temp_ROI_Rect, GetMarkDistance, g_dPixelSize, OutputImg, ProbeMarkinform);
	if (g_nOutputnum == -3)
	{

		return 2;
	}
	if (g_nOutputnum == -2)
	{
		return 3;
	}
	if (!OutputImg.empty())
		g_AngleOutput = OutputImg;

	// 字符串转换
	Angleinform = QString::number(abs(ProbeMarkinform), 'd', 5);
	if (g_ProbeAngleMicrRatio != 0 && g_dPixelSize != 0)
		g_movepixel = abs(ProbeMarkinform) / (g_ProbeAngleMicrRatio * g_dPixelSize * 1000);

	if (g_nOutputnum == -1)
	{
		return 4;
	}
	else if (g_nOutputnum == 0)
	{
		return 5;
	}
	else if (g_nOutputnum == 1)
	{
		angle = -Angleinform.toDouble();
		return 0;
	}
	else if (g_nOutputnum == 2)
	{
		angle = Angleinform.toDouble();
		g_movepixel = -g_movepixel;
		return 0;
	}
	/* 角度调整*/
	//Mat Img = g_srcImg.clone();//原图显示匹配区域
	//int return_num = m_pNeedleMark->Probe_Angle_move(Img, g_ProbeSlipDirect, g_Temp_ROI_Rect, g_movepixel, g_rotaionmoveline);
	//g_bslipmove = false;//将滑移线设为否
	//g_brotaionmove = true;//副屏显示转角线
	//QString proberotaiondirect, textshow;
	//if (ui.radioButton_No1_Probe->isChecked())//根据选择提示探针号码
	//	probeNumber = QString::number(1);//1号
	//else if (ui.radioButton_No2_Probe->isChecked())//根据选择提示探针号码
	//	probeNumber = QString::number(2);//2号	
	//if (g_nOutputnum == 1)
	//	proberotaiondirect = "请顺时针转动" + probeNumber + "号探针";
	//else if (g_nOutputnum == 2)
	//	proberotaiondirect = "请逆时针转动" + probeNumber + "号探针";
	//switch (return_num)
	//{
	//case -1:
	//	ui.textEdit_Output->setText("图像视野范围过小，1/4侧移线超出视野！！！");
	//	break;
	//case 0:
	//	ui.textEdit_Output->setText("滑移线绘制失败！！！");
	//	break;
	//case 1:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	textshow = +"移动至线段处！！";
	//	ui.textEdit_Output->setText(textshow);
	//	ui.pushButton_DrawReferLine->setEnabled(true);
	//	break;
	//case 2:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	textshow = +"两次移动至线段处！！";
	//	ui.textEdit_Output->setText(textshow);
	//	ui.pushButton_DrawReferLine->setEnabled(true);
	//	break;
	//case 4:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	ui.pushButton_DrawReferLine->setEnabled(true);
	//	textshow = +"四次移动至线段处！！";
	//	ui.textEdit_Output->setText(textshow);
	//	break;
	//default:
	//	break;
	//}


}
int NeedleMark::moveAngle(int probe) {
	//Mat Img = g_srcImg.clone();//原图显示匹配区域
	//int return_num = m_pNeedleMark->Probe_Angle_move(Img, g_ProbeSlipDirect, g_Temp_ROI_Rect, g_movepixel, g_rotaionmoveline);
	//g_bslipmove = false;//将滑移线设为否
	//g_brotaionmove = true;//副屏显示转角线
	//QString probeNumber, proberotaiondirect, textshow;
	//if (probe == 1)//根据选择提示探针号码
	//	probeNumber = QString::number(1);//1号
	//else if (probe == 2)//根据选择提示探针号码
	//	probeNumber = QString::number(2);//2号	
	//if (g_nOutputnum == 1)
	//	proberotaiondirect = "请顺时针转动" + probeNumber + "号探针";
	//else if (g_nOutputnum == 2)
	//	proberotaiondirect = "请逆时针转动" + probeNumber + "号探针";
	//switch (return_num)
	//{
	//case -1:
	//	//ui.textEdit_Output->setText("图像视野范围过小，1/4侧移线超出视野！！！");

	//	break;
	//case 0:
	//	//ui.textEdit_Output->setText("滑移线绘制失败！！！");
	//	break;
	//case 1:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	textshow = +"移动至线段处！！";
	//	//ui.textEdit_Output->setText(textshow);
	//	//ui.pushButton_DrawReferLine->setEnabled(true);
	//	break;
	//case 2:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	textshow = +"两次移动至线段处！！";
	//	//ui.textEdit_Output->setText(textshow);
	//	//ui.pushButton_DrawReferLine->setEnabled(true);
	//	break;
	//case 4:
	//	rectangle(Img, g_Temp_ROI_Rect, Scalar(0, 0, 255), 3);
	//	line(Img, g_rotaionmoveline[0], g_rotaionmoveline[1], Scalar(255, 0, 0), 3);
	//	imshow("ShowImg", Img);//显示所框出的感兴趣区域
	//	ui.pushButton_DrawReferLine->setEnabled(true);
	//	textshow = +"四次移动至线段处！！";
	//	ui.textEdit_Output->setText(textshow);
	//	break;
	//default:
	//	break;
	//}
	return 0;
}


int NeedleMark::autoInit(needleMarkLevelParas& configs,Mat img1,Mat img2) {

	g_srcImg = img1.clone();
	g_mInitialImg1 = g_srcImg.clone();

	g_srcImg = img2.clone();
	// 初始化
	g_dPixelSize = configs.PixelSize;
	int iRes = FinishInitProbe(configs.probe);
	return iRes;
}

int NeedleMark::manualInit(int probe)
{
	return 0;
}
