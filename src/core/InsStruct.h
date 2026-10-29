#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "Const.h"


struct ImuScale {
    double accelMpsPerLsb = 0.0; // (m/s)/LSB
    double gyroRadPerLsb = 0.0; // rad/LSB
    double rateHz = 0.0; // 标称采样率 (Hz)

    [[nodiscard]] bool known() const {
        return accelMpsPerLsb > 0.0 && gyroRadPerLsb > 0.0 && rateHz > 0.0;
    }
};

struct ImuModel {
    int type{}; // OEM7 IMU Type 码
    const char *name{}; // 型号名
    ImuScale scale; // 解码当量
};


static constexpr double kFtToMps = 0.3048; // ft/s/LSB → m/s/LSB
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

// 单条 IMU 原始采样
struct ImuSample {
    int gnssWeek = 0;
    double tow = 0.0; // GPS 周内秒
    int32_t gx = 0, gy = 0, gz = 0; // 陀螺 角增量原始计数
    int32_t ax = 0, ay = 0, az = 0; // 加速度计速度增量原始计数
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

/// 采样间隔中位数：由时间戳得到，比表里的标称采样率可信。
inline double imuMedianDt(const ImuLog &log, const double fallback = 0.005) {
    if (log.size() < 3) return fallback;
    std::vector<double> d;
    d.reserve(log.size() - 1);
    for (size_t i = 1; i < log.size(); ++i) d.push_back(log[i].tow - log[i - 1].tow);
    std::sort(d.begin(), d.end());
    return d[d.size() / 2];
}

inline ImuScaleGuess guessImuScale(const ImuLog &staticLog, const double g) {
    ImuScaleGuess r;
    if (staticLog.size() < 200) {
        r.note = "静置段样本太少（不足 200 个）";
        return r;
    }
    if (!(g > 0.0)) {
        r.note = "当地重力加速度 g 不是有效正数";
        return r;
    }
    const double dt = imuMedianDt(staticLog, 0.0);
    if (!(dt > 1e-4) || dt > 0.1) {
        r.note = "时间戳间隔异常";
        return r;
    }
    double sum = 0.0, sum2 = 0.0;
    const size_t n = std::min<size_t>(staticLog.size(), 20000);
    for (size_t i = 0; i < n; ++i) {
        const auto &s = staticLog[i];
        const double m = std::sqrt(static_cast<double>(s.ax) * s.ax + static_cast<double>(s.ay) * s.ay +
                                   static_cast<double>(s.az) * s.az);
        sum += m;
        sum2 += m * m;
    }
    const double mean = sum / static_cast<double>(n);
    if (mean <= 0.0) {
        r.note = "加计三轴无输出";
        return r;
    }
    const double var = sum2 / static_cast<double>(n) - mean * mean;
    if (const double sd = std::sqrt(var > 0.0 ? var : 0.0); sd / mean > 0.02) {
        char buf[128];
        std::snprintf(buf, sizeof buf, "这段不是静置（|f| 起伏 %.1f %%）", 100.0 * sd / mean);
        r.note = buf;
        return r;
    }

    r.dt = dt;
    r.rateHz = 1.0 / dt;
    r.accelMpsPerLsb = g * dt / mean;
    const ImuModel *hit = nullptr;
    bool allSame = true; // 候选们的当量是否完全一致（孪生型号）
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
        if (r.matches > 1) r.note = "有 " + std::to_string(r.matches) + " 个型号当量相同，取 " + hit->name; //NOLINT
        return r;
    }
    char buf[176];
    if (r.matches == 0) {
        std::snprintf(buf, sizeof buf, "反推当量 %.6g (m/s)/LSB（%.0f Hz）与内置型号表全都不符",
                      r.accelMpsPerLsb, r.rateHz);
    } else {
        std::snprintf(buf, sizeof buf, "反推当量 %.6g (m/s)/LSB（%.0f Hz）有 %d 个当量不同的型号都相符",
                      r.accelMpsPerLsb, r.rateHz, r.matches);
    }
    r.note = buf;
    r.model = nullptr;
    return r;
}

// 单历元姿态（欧拉角，rad）
struct Attitude {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// 带时间戳的姿态采样
struct AttitudeSample {
    double tow = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};
