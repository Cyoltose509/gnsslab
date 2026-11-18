#pragma once
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include "core/geom/Attitude.h"


struct ImuScale {
    double accelMpsPerLsb = 0.0;
    double gyroRadPerLsb = 0.0;
    double rateHz = 0.0;
    [[nodiscard]] bool known() const {
        return accelMpsPerLsb > 0.0 && gyroRadPerLsb > 0.0 && rateHz > 0.0;
    }
};

struct ImuSample {
    int gnssWeek = 0;
    double tow = 0.0;
    std::int32_t gx = 0, gy = 0, gz = 0;
    std::int32_t ax = 0, ay = 0, az = 0;
};

using ImuLog = std::vector<ImuSample>;

/// 一段对准/解算的姿态采样
struct AttitudeSample {
    double tow = 0.0;
    Euler att;
};


// 一个采样间隔的增量观测（机械编排的输入单位）
struct InsEpoch {
    double t = 0.0;
    Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();  // 角增量 (rad, b系 前-右-下)
    Eigen::Vector3d dvel = Eigen::Vector3d::Zero();    // 速度增量 (m/s, b系 前-右-下)
    double h = 0.0;                                    // 采样间隔 (s)
};

// 增量间隔有效性
inline bool validEpochGap(const double h) { return h > 1e-5 && h <= 1.0; }

// 导航状态
struct NavState {
    double t = 0.0;
    Eigen::Vector3d llh = Eigen::Vector3d::Zero();  // [lat, lon, h] (rad, rad, m)
    Eigen::Vector3d vned = Eigen::Vector3d::Zero();  // (m/s, NED)
    ATTQuat qbn;                                     // b->n (NED): 前-右-下 -> 北-东-地
};

