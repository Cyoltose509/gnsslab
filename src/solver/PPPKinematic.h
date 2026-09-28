#pragma once

#include "PPPStatic.h"

class PPPKinematic : public PPPStatic {
public:
    PPPKinematic() {
        mRequirePhaseForIF = true;
        mKinematic = true;
        mReanchor = true;
        mReanchorEveryEpoch = false;
        mEstimateConstOffset = false;
        mConfig.posNoise = 100.0;
    }

    ~PPPKinematic() override = default;

    bool processEpoch(ObsData &obs) override;

protected:
    Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    Vector3d reanchorSPP(const Vector3d &seedIn, int *nOut = nullptr, double *rmsOut = nullptr) const;

    mutable Vector3d mReanchorPos = Vector3d::Zero();
    Vector3d mMotionRefPos = Vector3d::Zero();
    Vector3d mMotionRefVel = Vector3d::Zero();
    CommonTime mMotionRefEpoch{};
    bool mHasMotionRef = false;
    Vector3d mPrevEstPos = Vector3d::Zero();
    CommonTime mPrevEpoch{};
    bool mHasPrev = false;
    double mDivergeJumpThresh = 500.0;
};
