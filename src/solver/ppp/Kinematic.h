#pragma once

#include "Static.h"

class PPPKinematic : public PPPStatic {
public:
    /// 加速度包络(m/s^2)：真实运动不可能超过它，用来给外推邻域定上界
    static constexpr double kAccelEnvelope = 9.0;
    /// 每历元重锚定的 Baarda 最大剔除轮数：BDS GEO 最多 4 颗，全部被剔也到此为止。
    static constexpr int kMaxBaardaRounds = 4;

    PPPKinematic() {
        mRequirePhaseForIF = true;
        mKinematic = true;
        mReanchorEveryEpoch = true;

        // 行为标志（不随参数页变）：钟差初值 σ 按运动学取大值
        ekf.clockInitSigma = 1.0E4;
    }

    ~PPPKinematic() override = default;

    bool processEpoch(ObsData &obs) override;

    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) override;

    Eigen::Vector3d reanchorSPP(const Eigen::Vector3d &seedIn, int *nOut = nullptr, double *rmsOut = nullptr,
                                double *pdopOut = nullptr, SatSystem sysOnly = SatSystem::Unknown);

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
