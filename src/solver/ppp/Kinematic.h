#pragma once

#include "Core.h"

namespace PPP {

class Kinematic : public Core {
public:
    Kinematic() {
        mRequirePhaseForIF = true;
        mKinematic = true;
        mReanchorEveryEpoch = true;

        // 行为标志（不随参数页变）：钟差初值 σ 按运动学取大值
        ekf.clockInitSigma = 1.0E4;
    }

    ~Kinematic() override = default;

    bool processEpoch(ObsData &obs) override;

protected:
    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    Eigen::Vector3d reanchorSPP(const Eigen::Vector3d &seedIn, int *nOut = nullptr, double *rmsOut = nullptr,
                                double *pdopOut = nullptr, char sysOnly = 0) const;

    mutable Eigen::Vector3d mReanchorPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d mMotionRefPos = Eigen::Vector3d::Zero();
    Eigen::Vector3d mMotionRefVel = Eigen::Vector3d::Zero();
    CommonTime mMotionRefEpoch{};
    bool mHasMotionRef = false;
    Eigen::Vector3d mPrevEstPos = Eigen::Vector3d::Zero();
    CommonTime mPrevEpoch{};
    bool mHasPrev = false;
    double mDivergeJumpThresh = 500.0;
};

}
