#pragma once
#include <vector>

#include "core/CoordConvert.h"  // AttQuat 四元数类：对准结果的姿态表示与矢量旋转
#include "core/InsMath.h"

struct Attitude {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// 欧拉角 → 四元数（q 表示 C_b^n，即 v^n = q⊗v^b⊗q*）。
// 对准结果送入 INS 姿态递推（AttQuat::update）时的起点形式。
inline AttQuat attitudeQuat(const Attitude &a) {
    return AttQuat::fromEuler(a.roll, a.pitch, a.yaw);
}

struct AttitudeSample {
    double tow = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// 粗对准质量诊断：把机体实测矢量按对准结果旋进导航系，与参考矢量比对。
// 静置时两项方向残差都应接近 0；残差大说明该段并非真正静置，或陀螺零偏没扣。
struct AlignQuality {
    Eigen::Vector3d axisX = Eigen::Vector3d::UnitX(); // 机体 X 轴在当地 NED 的单位方向
    Eigen::Vector3d axisY = Eigen::Vector3d::UnitY(); // 机体 Y 轴在当地 NED 的单位方向
    Eigen::Vector3d axisZ = Eigen::Vector3d::UnitZ(); // 机体 Z 轴在当地 NED 的单位方向
    double gravityErrDeg = 0.0;   // 重力方向残差（°）：0 = 载体在当地水平面内
    double earthRateErrDeg = 0.0; // 角速度方向残差（°）：0 = 实测角速度与 ω_ie^n 同向（罗盘一致性，
                                  // 由重力定姿后校验航向是否可信；不因 TRIAD 构造而恒 0）
    double earthRateErrRad = 0.0; // 角速度大小残差（rad/s）：仅当传入陀螺标度因数时有效；
                                  // 非 0 = 该段有残余转动（如对准段未真正静置）
};

// 静态解析粗对准
class Aligner {
public:
    // 对整段（或指定区间）求均值做一次静态对准。
    // 全部形参必填（无默认实参）：sp 是站点参数（当地重力/纬度/地球自转率），
    // core 不持有站点默认值，见 core/InsMath.h 顶部说明。
    static Attitude solve(const ImuLog &log,
                          const Eigen::Vector3d &gyroBiasCounts,
                          size_t begin, size_t end,
                          const SiteParams &sp);

    // 同上，但直接返回四元数（与 solve 同一 C_b^n；INS 递推起点用这个）
    static AttQuat solveQuat(const ImuLog &log,
                             const Eigen::Vector3d &gyroBiasCounts,
                             size_t begin, size_t end,
                             const SiteParams &sp);

    // 对准质量诊断（与 solve 同区间、同均值口径）
    static AlignQuality diagnose(const ImuLog &log,
                                 const Eigen::Vector3d &gyroBiasCounts,
                                 size_t begin, size_t end,
                                 const SiteParams &sp);

    // 同 diagnose，但直接吃段均值。流式面板只有累计和、拿不到整段 log 时用这个，
    // 避免面板里再抄一遍诊断逻辑（口径仍只此一处）。
    // gyroScaleCountsPerRadS：陀螺标度因数（counts/(rad/s)，即 ImuCalibration::gyroScale）。
    // 默认 0 = 只算方向残差，不算大小残差（未做陀螺标定时就这样用）。
    static AlignQuality diagnoseMeans(const Eigen::Vector3d &accMean, const Eigen::Vector3d &gyrMean,
                                      const Eigen::Vector3d &gyroBiasCounts,
                                      const SiteParams &sp,
                                      double gyroScaleCountsPerRadS = 0.0);

    // 滑动窗口序列
    static std::vector<AttitudeSample> alignSeries(const ImuLog &log, int win,
                                                   const Eigen::Vector3d &gyroBiasCounts,
                                                   const SiteParams &sp);

    // 流式滑窗粗对准
    struct AlignStream {
        int win = 200;
        SiteParams sp{};
        Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
        size_t count = 0;

        void init(int window, const SiteParams &site, const Eigen::Vector3d &gyroBiasCounts);

        void step(const ImuSample &s, AttitudeSample &out);
    };
};
