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

    // 六位置对应的机体比力方向（m/s^2）：X上 → +X。
    static Eigen::Vector3d posForce(Pos p, double g);

    static void rawMean(const ImuLog &log, Eigen::Vector3d &acc, Eigen::Vector3d &gyr);

    static double medianDt(const ImuLog &log, double fallback = 0.005);


    // 逐输出轴 4 参数加权最小二乘。scaleSigma 非空时填比例因子 1σ
    // （后验协方差 σ̂²·N⁻¹ 的对角开方，σ̂² = vᵀWv/dof，见 SolverLSQ::sigma0）。
    static void solveLinear(const std::vector<Eigen::Vector3d> &raw, const std::vector<Eigen::Vector3d> &f,
                            const std::vector<double> &w,
                            Eigen::Matrix3d &M, Eigen::Vector3d &b, double &rmsResid,
                            Eigen::Vector3d *scaleSigma = nullptr);


    // turnAngle = 2π 是实验协议常量（转台整圈）；g（当地重力加速度）是站点相关
    // 输入，不设默认值：由调用方填写（GUI 取参数页，测试取 test/lab_site.h）。
    static CalibReport calibrate(const CalibSegments &seg,
                                 double turnAngle,
                                 double g);
};
