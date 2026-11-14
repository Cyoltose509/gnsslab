#pragma once

#include <vector>

#include <Eigen/Core>

#include "core/SatID.h"

namespace Velocity {
    struct DopplerSat {
        SatID sat;
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        double freqHz = 0.0;
        double dopplerHz = 0.0;
        double posWeight = 1.0;
    };

    struct DopplerResult {
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        double clockDrift = 0.0;
        Eigen::Vector3d sigma = Eigen::Vector3d::Zero();
        int used = 0;
        bool ok = false;
    };

    /// 4 颗及以上有效多普勒才解
    DopplerResult byDoppler(const Eigen::Vector3d &recvXyz, const std::vector<DopplerSat> &sats);

    /// 中心差分
    void byDifference(const std::vector<double> &timeSec, const std::vector<Eigen::Vector3d> &xyz,
                      std::vector<Eigen::Vector3d> &vel);
}
