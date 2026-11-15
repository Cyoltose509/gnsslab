#pragma once

#include "solver/ppp/Kinematic.h"
#include "core/SatID.h"
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <Eigen/Eigen>


class LEO : public PPPKinematic {
public:
    LEO() {
        setTropo(false);
        mReanchorEveryEpoch = true;
        mMotionThresh = 100.0;

        ekf.clockInitSigma = 60.0;
        mConfig.firstEpochPosSigma = 1.0;
        mConfig.ambProcSigma = std::sqrt(1.0E-7); // 旧方差口径默认值 1e-7 的精确开方（σ 口径）
        mConfig.phaseSigma = 0.003;
        mConfig.minSatsForUpdate = 4;
        mApplySolidTide = false;
        mPinInitHeight = false;
        mUseCycleSlip = false;
    }

    ~LEO() override = default;


    /// 取值编排在 app 层，这里只管接过来用；缺的段落按 0 处理并落日志。
    void setAntennaOffset(const AntennaOffset &offset, const std::string &source);

    bool processEpoch(ObsData &obs) override;

    void applyOptions(const SolverOptions &o) override;

    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    void buildEquSys() override;

    Eigen::Vector3d recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch,
                                          const SatID &sat) const override;

    // 首历元引导
    void bindRefPosition() override;

private:
    mutable Eigen::Vector3d mLastGoodAnchor = Eigen::Vector3d::Zero(); // 质量门控重锚定：最近一个可信锚点
    mutable bool mSppGood = true; // 本历元 SPP 重锚定质量是否达标
    mutable bool mGeomWeak = false; // 本历元码观测几何不足以定轨
    mutable double mEpochPdop = 0.0; // 本历元 SPP 几何 PDOP
    bool mPrevEpochSolved = true; // 上一历元是否发布了位置解

    // 线性化点外推用的运动状态：全部由相邻历元解算结果差分得到，不含力模型
    Eigen::Vector3d mPrevVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d mRecvAcc = Eigen::Vector3d::Zero();
    CommonTime mPrevVelTime{};
    bool mHasPrevVel = false;
};
