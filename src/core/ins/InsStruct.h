#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "core/Const.h"
#include "core/geom/Attitude.h"
#include "i18n/I18n.h"


struct ImuScale {
    double accelMpsPerLsb = 0.0;
    double gyroRadPerLsb = 0.0;
    double rateHz = 0.0;

    [[nodiscard]] bool known() const {
        return accelMpsPerLsb > 0.0 && gyroRadPerLsb > 0.0 && rateHz > 0.0;
    }
};

struct ImuModel {
    int type{};
    const char *name{};
    ImuScale scale;
};


static constexpr double kFtToMps = 0.3048;
static constexpr double kAccelHG1700 = 0x1p-27 * kFtToMps;


inline const ImuModel kImuModels[] = {
    {8, "IMU-LN200", {0x1p-14, 0x1p-19, 200.0}},
    {11, "HG1700-AG58", {kAccelHG1700, 0x1p-33, 100.0}},
    {12, "HG1700-AG62", {0x1p-26 * kFtToMps, 0x1p-33, 100.0}},
    {16, "IMU-CPT (KVH)", {0.05 / 32768.0, 0.1 / 3600.0 / 256.0 * DEG_TO_RAD, 200.0}},
    {20, "HG1930-AA99", {kAccelHG1700, 0x1p-33, 100.0}},
    {26, "IMU-ISA-100C", {2.0e-8, 1.0e-9, 200.0}},
    {27, "HG1900-CA50", {kAccelHG1700, 0x1p-33, 100.0}},
    {28, "HG1930-CA50", {kAccelHG1700, 0x1p-33, 100.0}},
    {32, "OEM-IMU-STIM300", {0x1p-22, 0x1p-21 * DEG_TO_RAD, 125.0}},
    {33, "IMU-P1750", {0.05 / 32768.0, 0.1 / 3600.0 / 256.0 * DEG_TO_RAD, 200.0}},
    {
        41, "OEM-IMU-EG320N", {
            0.200 / 65536.0 * (9.80665 / 1000.0) / 125.0,
            0.008 / 65536.0 / 125.0 * DEG_TO_RAD, 125.0
        }
    },
    {52, "IMU-µIMU-IC", {2.0e-8, 1.0e-9, 200.0}},
    {56, "OEM-IMU-STIM300D", {0x1p-22, 0x1p-21 * DEG_TO_RAD, 125.0}},
    {58, "OEM-IMU-HG4930 AN01", {0x1p-29, 0x1p-33, 100.0}},
    {
        61, "OEM-IMU-EG370N", {
            0.400 / 65536.0 * (9.80665 / 1000.0) / 200.0,
            0.0151515 / 65536.0 / 200.0 * DEG_TO_RAD, 200.0
        }
    },
    {
        62, "OEM-IMU-EG320N (200Hz)", {
            0.200 / 65536.0 * (9.80665 / 1000.0) / 200.0,
            0.008 / 65536.0 / 200.0 * DEG_TO_RAD, 200.0
        }
    },
    {68, "OEM-IMU-HG4930 AN04", {0x1p-29, 0x1p-33, 100.0}},
    {69, "OEM-IMU-HG4930 AN04 (400Hz)", {0x1p-29, 0x1p-33, 400.0}},
    {71, "OEM-IMU-µIMU-IC-UART (400Hz)", {2.0e-8, 1.0e-9, 400.0}},
};

inline const ImuModel *findImuModel(const int type) {
    for (const auto &m: kImuModels)
        if (m.type == type) return &m;
    return nullptr;
}


struct ImuSample {
    int gnssWeek = 0;
    double tow = 0.0;
    std::int32_t gx = 0, gy = 0, gz = 0;
    std::int32_t ax = 0, ay = 0, az = 0;
};

using ImuLog = std::vector<ImuSample>;

struct ImuScaleGuess {
    const ImuModel *model = nullptr;
    int matches = 0;
    double accelMpsPerLsb = 0.0;
    double dt = 0.0;
    double rateHz = 0.0;
    std::string note;
};


inline double imuMedianDt(const ImuLog &log, const double fallback = 0.005) {
    if (log.size() < 3) return fallback;
    std::vector<double> d;
    d.reserve(log.size() - 1);
    for (std::size_t i = 1; i < log.size(); ++i) d.push_back(log[i].tow - log[i - 1].tow);
    std::sort(d.begin(), d.end());
    return d[d.size() / 2];
}

inline ImuScaleGuess guessImuScale(const ImuLog &staticLog, const double g) {
    ImuScaleGuess r;
    if (staticLog.size() < 200) {
        r.note = I18n::tr("ins.scale.too_few");
        return r;
    }
    if (!(g > 0.0)) {
        r.note = I18n::tr("ins.scale.bad_g");
        return r;
    }
    const double dt = imuMedianDt(staticLog, 0.0);
    if (!(dt > 1e-4) || dt > 0.1) {
        r.note = I18n::tr("ins.scale.bad_dt");
        return r;
    }
    double sum = 0.0, sum2 = 0.0;
    const std::size_t n = std::min<std::size_t>(staticLog.size(), 20000);
    for (std::size_t i = 0; i < n; ++i) {
        const auto &s = staticLog[i];
        const double m = std::sqrt(static_cast<double>(s.ax) * s.ax + static_cast<double>(s.ay) * s.ay +
                                   static_cast<double>(s.az) * s.az);
        sum += m;
        sum2 += m * m;
    }
    const double mean = sum / static_cast<double>(n);
    if (mean <= 0.0) {
        r.note = I18n::tr("ins.scale.no_accel");
        return r;
    }
    const double var = sum2 / static_cast<double>(n) - mean * mean;
    if (const double sd = std::sqrt(var > 0.0 ? var : 0.0); sd / mean > 0.02) {
        char pct[32];
        std::snprintf(pct, sizeof pct, "%.1f", 100.0 * sd / mean);
        r.note = I18n::tr("ins.scale.not_static", pct);
        return r;
    }

    r.dt = dt;
    r.rateHz = 1.0 / dt;
    r.accelMpsPerLsb = g * dt / mean;
    const ImuModel *hit = nullptr;
    bool allSame = true;
    for (const auto &m: kImuModels) {
        if (std::fabs(r.accelMpsPerLsb / m.scale.accelMpsPerLsb - 1.0) > 0.05) continue;
        if (std::fabs(r.rateHz / m.scale.rateHz - 1.0) > 0.10) continue;
        ++r.matches;
        if (!hit) {
            hit = &m;
        } else if (m.scale.accelMpsPerLsb != hit->scale.accelMpsPerLsb ||
                   m.scale.gyroRadPerLsb != hit->scale.gyroRadPerLsb ||
                   m.scale.rateHz != hit->scale.rateHz) {
            allSame = false;
        }
    }
    if (r.matches >= 1 && allSame) {
        r.model = hit;
        if (r.matches > 1) r.note = I18n::tr("ins.scale.multi_model", r.matches, hit->name);
        return r;
    }
    char scale[32];
    char hz[32];
    std::snprintf(scale, sizeof scale, "%.6g", r.accelMpsPerLsb);
    std::snprintf(hz, sizeof hz, "%.0f", r.rateHz);
    r.note = r.matches == 0 ? I18n::tr("ins.scale.no_model", scale, hz)
                            : I18n::tr("ins.scale.multi_match", scale, hz, r.matches);
    r.model = nullptr;
    return r;
}


/// 一段对准/解算的姿态采样：时标 + 三个角平铺，直接喂曲线。
struct AttitudeSample {
    double tow = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};


// 一个采样间隔的增量观测（机械编排的输入单位）
struct InsEpoch {
    double t = 0.0;
    Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();  // 角增量 (rad, b系 前-右-下)
    Eigen::Vector3d dvel = Eigen::Vector3d::Zero();    // 速度增量 (m/s, b系 前-右-下)
    double h = 0.0;                                    // 采样间隔 (s)
};

// 增量间隔有效性：重复时间戳与断档（超出一个采样周期）都不算真实推进。
inline bool validEpochGap(const double h) { return h > 1e-5 && h <= 1.0; }

// 导航状态（机械编排逐历元输出）
struct NavState {
    double t = 0.0;
    Eigen::Vector3d llh = Eigen::Vector3d::Zero();  // [lat, lon, h] (rad, rad, m)
    Eigen::Vector3d vned = Eigen::Vector3d::Zero();  // (m/s, NED)
    ATTQuat qbn;                                     // b->n (NED): 前-右-下 -> 北-东-地
};

// 初始条件（由静态对准或人工给定）
struct InsInit {
    double t0 = 0.0;
    Eigen::Vector3d llh = Eigen::Vector3d::Zero();  // [lat, lon, h] (rad, rad, m)
    Eigen::Vector3d vned = Eigen::Vector3d::Zero(); // (m/s, NED)
    double rollDeg = 0.0, pitchDeg = 0.0, headingDeg = 0.0; // 初始姿态角 (deg)
};

enum class ImuScaleSource { Record, Reverse, Manual };

