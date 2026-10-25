#pragma once
#include <vector>
#include "core/CoordConvert.h"
#include "core/InsMath.h"
#include "core/InsStruct.h"

struct AlignQuality {
    Eigen::Vector3d axisX = Eigen::Vector3d::UnitX(); // 机体 X 轴在当地 NED 的单位方向
    Eigen::Vector3d axisY = Eigen::Vector3d::UnitY(); // 机体 Y 轴在当地 NED 的单位方向
    Eigen::Vector3d axisZ = Eigen::Vector3d::UnitZ(); // 机体 Z 轴在当地 NED 的单位方向
    double gravityErrDeg = 0.0;   // 重力方向残差（°）
    double earthRateErrDeg = 0.0; // 角速度方向残差（°）
    double earthRateErrRad = 0.0; // 角速度大小残差（rad/s）
};

// 静态解析粗对准
class Aligner {
public:
    // 对整段（或指定区间）求均值做一次静态对准
    static Attitude solve(const ImuLog &log,
                          const Eigen::Vector3d &gyroBiasCounts,
                          size_t begin, size_t end,
                          const SiteParams &sp);

    static AttQuat solveQuat(const ImuLog &log,
                             const Eigen::Vector3d &gyroBiasCounts,
                             size_t begin, size_t end,
                             const SiteParams &sp);

    // 对准质量诊断
    static AlignQuality diagnose(const ImuLog &log,
                                 const Eigen::Vector3d &gyroBiasCounts,
                                 size_t begin, size_t end,
                                 const SiteParams &sp);

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
