#pragma once
#include <vector>
#include <QMutex>
#include <qmutex.h>
#include <QMutexLocker>
#include <Qstring>

// YZ轨迹插补参数
struct TrajPoint {
    double y;
    double z;
    double vel;
};
struct ParseDatas { 
    
    double t;
    double p;
    double r;
    double X;
    double x;
    double y;
    double z;
    double u;
};

struct AxisCoordinateData
{
    double encPos[4];      // 编码器位置
    double prfPos[4];      // 规划位置

    long originPos[4];     // 坐标系原点

    short dimension;       // 坐标系维度

    short profile[4];      // 坐标系映射

    bool valid;            // 数据有效
};

struct needleMarkLevelParas {
    int probe = 0;
    int initStep = 0;
    double SlipDistance = 0;
    double ProbeMark_Distance = 0;
    double PixelSize = 0;
    double safeHeight = 0;
    double InitAngle = 0;
    double depth = 0;
    double ratio = 0;
    double maxCycles = 0;
    double downSpeed = 0;
    double Z_UPSpeed = 0;
    double Y_Disp = 0;
    double Y_Speed = 0;

};

struct CameraParams
{
    // 图像参数
    int width = 1920;          // 图像宽度（像素）
    int height = 1080;         // 图像高度（像素）
    double fps = 30.0;         // 目标帧率

    // 曝光与控制
    double exposureTime_us = 10000.0;   // 曝光时间（微秒）
    double gain = 0.0;                 // 增益（dB 或线性值，按厂家定义）
    double gamma = 1.0;                // 伽马校正值

    // 白平衡（RGB 增益）
    double redGain = 1.0;
    double greenGain = 1.0;
    double blueGain = 1.0;

    // 对焦 / 光圈（若有）
    double focus = 0.0;                // 对焦位置（0~1 或微米）
    double aperture = 0.0;             // 光圈值（F 数）

    // 触发模式
    enum TriggerMode { Internal, External, Software };
    TriggerMode triggerMode = Internal;

    // 其他自定义
    QString customProfile;             // 可存放配置文件名称或 JSON 字符串

    // 判断两个结构体是否相等（便于比较）
    bool operator==(const CameraParams& other) const {
        return width == other.width &&
            height == other.height &&
            fps == other.fps &&
            exposureTime_us == other.exposureTime_us &&
            gain == other.gain &&
            gamma == other.gamma &&
            redGain == other.redGain &&
            greenGain == other.greenGain &&
            blueGain == other.blueGain &&
            focus == other.focus &&
            aperture == other.aperture &&
            triggerMode == other.triggerMode &&
            customProfile == other.customProfile;
    }
    bool operator!=(const CameraParams& other) const {
        return !(*this == other);
    }
};
