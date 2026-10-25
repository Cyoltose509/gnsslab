#pragma once
#include <array>
#include <string>
#include <utility>
#include <vector>

#include "core/InsMath.h"

// 误差标定
struct CalibReport {
    ImuCalibration calib;
    Eigen::Vector3d accelScaleCounts = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelScaleG = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelScaleSigma = Eigen::Vector3d::Zero(); // 比例因子 1σ（counts/(m/s²)，同 accelScaleCounts 单位）
    double accelRms = 0.0; // 六位置拟合残差 RMS（counts）
    double dt = 0.005; // 采样间隔 (s)
    Eigen::Vector3d gyroKeff = Eigen::Vector3d::Zero(); // counts/rad（各轴）
    double gyroKmean = 0.0;
    Eigen::Vector3d gyroDeltaS = Eigen::Vector3d::Zero(); // 比例因子误差 δs
    int nPos[6] = {0, 0, 0, 0, 0, 0};
    int nRot[6] = {0, 0, 0, 0, 0, 0};
    bool ok = false; // 标定是否成功（覆盖度足够）
    std::string errorMsg; // ok=false 时的原因（覆盖不足等）
};

class ImuCalibrator {
public:
    enum Pos { XUP, XDN, YUP, YDN, ZUP, ZDN }; // 对应 [X上,X下,Y上,Y下,Z上,Z下]
    enum Rot { XFWD, XREV, YFWD, YREV, ZFWD, ZREV }; // 对应 [X下正转,X下逆转,...]

    struct CalibSegments {
        std::vector<std::pair<Pos, ImuLog> > pos; // 静置段
        std::vector<std::pair<Rot, ImuLog> > rot; // 旋转段
    };

    // —— 分段检测：标签 / 算法单一真源（纯算法，输入已解析 ImuLog）——
    struct DetectParams {
        int winSamples = 50; // 滑窗长度（样本）
        double minSegSeconds = 0.15; // 段最短时长(s)
        int minSegSamples = 30; // 兜底
        double rotThrCounts = 30.0; // 旋转判定绝对下限（counts）
        double rotFrac = 0.4; // 旋转阈值
    };

    // 单段分类结果。i0/i1 为 [i0, i1) 样本区间，用于从原日志抽取子段。
    struct CalibSegment {
        bool isStatic = false; // true=静置, false=旋转
        Pos pos = XUP; // 仅 isStatic 有效
        Rot rot = XFWD; // 仅 !isStatic 有效
        size_t i0 = 0, i1 = 0; // [i0, i1) 样本区间
    };

    // 位置/旋转方位的中文标签：唯一真源（Pos/Rot 枚举即本类定义）。
    static const char *posLabel(Pos p) {
        static const char *s[6] = {"X上", "X下", "Y上", "Y下", "Z上", "Z下"};
        return s[static_cast<int>(p)];
    }
    static const char *rotLabel(Rot r) {
        static const char *s[6] = {"X正转", "X逆转", "Y正转", "Y逆转", "Z正转", "Z逆转"};
        return s[static_cast<int>(r)];
    }

    // 对单个已解析日志做分段检测。
    static std::vector<CalibSegment> detect(const ImuLog &log, const DetectParams &p = DetectParams{});

    // 六位置对应的机体比力方向（m/s^2）：X上 → +X。
    static Eigen::Vector3d posForce(Pos p, double g);

    static void rawMean(const ImuLog &log, Eigen::Vector3d &acc, Eigen::Vector3d &gyr);

    static double medianDt(const ImuLog &log, double fallback = 0.005);


    static void solveLinear(const std::vector<Eigen::Vector3d> &raw, const std::vector<Eigen::Vector3d> &f,
                            const std::vector<double> &w,
                            Eigen::Matrix3d &M, Eigen::Vector3d &b, double &rmsResid,
                            Eigen::Vector3d *scaleSigma = nullptr);


    static CalibReport calibrate(const CalibSegments &seg,
                                 double turnAngle,
                                 double g);
};
