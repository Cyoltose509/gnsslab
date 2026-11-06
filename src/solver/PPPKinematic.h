#pragma once

#include "PPPStatic.h"

class PPPKinematic : public PPPStatic {
public:
    PPPKinematic() {
        mRequirePhaseForIF = true;
        mKinematic = true;
        mReanchorEveryEpoch = true;

        // 行为标志（不随参数页变）：钟差初值方差按运动学取大值
        ekf.clockInitVar = 1.0E8;
    }

    ~PPPKinematic() override = default;

    bool processEpoch(ObsData &obs) override;

protected:
    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    Eigen::Vector3d reanchorSPP(const Eigen::Vector3d &seedIn, int *nOut = nullptr, double *rmsOut = nullptr) const;

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
