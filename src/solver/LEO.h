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
    /// 4 星近共面时 PDOP 可上千，1 m 码噪声被放大成公里级，这样的 SPP 位置不能作线性化点
    static constexpr double kPdopLimit = 50.0;
    /// PDOP 到这个量级位置在法向完全不可观：码解实测带 70–140 m 误差，本历元不给解
    static constexpr double kPdopReject = 100.0;
    /// SPP 不可信时位置只剩相位撑着，偏离本历元锚点这么多就认为滤波已散
    static constexpr double kSppFailJumpM = 300.0;
    /// 判定解算结果可信的最长外推跨度(s)：低轨断档一分钟以上很常见
    static constexpr double kMotionTrustSpanS = 300.0;
    /// 相邻历元速度差分的加速度上限(m/s^2)：超过这个量级只可能是解算跳变，不是真实运动
    static constexpr double kAccelLimit = 15.0;
    /// 位置状态离开可信锚点运动外推超过这么多，就认定状态被钟跳带走了(m)
    static constexpr double kPosStateJumpM = 200.0;
    /// 断档恢复首历元的发布门限：PDOP × 码 σ 超过它就不发布该历元位置(m)
    static constexpr double kSppPosGateM = 10.0;
    /// 发布位置至少要 5 颗星：4 星恰好解 XYZ+钟，残差恒为 0 没有冗余；
    /// 再少就是纯状态外推，位置和多普勒速度会一起沿轨道漂，实测能累积到几十公里。
    static constexpr int kMinPublishSats = 5;

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
