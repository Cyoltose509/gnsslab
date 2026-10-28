#pragma once

#include "PPPStatic.h"

class PPPKinematic : public PPPStatic {
public:
    PPPKinematic() {
        mRequirePhaseForIF = true;
        mKinematic = true;
        mReanchorEveryEpoch = true;

        ekf.clockInitVar = 1.0E8;
        mConfig.minSatsForUpdate = 4;
        mConfig.ambNoise = 1.0E-7;
        mConfig.sigmaPhase = 0.003;
        mConfig.firstEpochPosVar = 1.0;

        mConfig.sigIFCode = 0.1;
        mConfig.posNoise = 100.0;
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
