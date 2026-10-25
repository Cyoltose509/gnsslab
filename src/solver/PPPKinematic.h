#pragma once

#include "PPPStatic.h"

class PPPKinematic : public PPPStatic {
public:
    PPPKinematic() {
        mRequirePhaseForIF = true;
        mKinematic = true;
        // 运动学逐历元用 code-SPP 重锚定线性化点（rtkpost 单一 PPP-Kinematic 模式靠每历元迭代重线性化
        // 覆盖慢/快接收机的等价机制）。旧 HEAD 通过 mReanchor=true 实现；rework 删掉该标志后若遗漏此行，
        // 运动学退化为"只在首历元重锚"，位置态逐历元脱离观测→缓慢漂移到数百米级垃圾解。
        // 这是本回归的根因，已修复。
        mReanchorEveryEpoch = true;

        // —— 运动学与 LEO 共用的 EKF / 先验预设 ——
        // 这些量对地面运动学(GPS 接收机动 <1m/历元)与 LEO(8km/s)都正确，故上提至基类，
        // 使纯运动学 PPP 与 LEO 共享同一组正确初值；LEO.h 仍可在自身构造函数叠加空基专有覆盖。
        // 钟差初方差必须放得很松(σ=10km)：PPP 钟差是 km 级未知量。若沿用静态默认 σ=100m(clockInitVar=1e4)，
        // 钟差误差会被吸收进位置→数米级系统偏置。LEO.h 同样置 1e8。
        ekf.clockInitVar = 1.0E8;
        mConfig.minSatsForUpdate = 4;   // 滤波更新最少卫星（对齐 harness / LEO）
        mConfig.ambNoise = 1.0E-7;     // 模糊度过程噪声：运动学比静态 1e-8 略松以吸收动态
        mConfig.sigmaPhase = 0.003;    // 运动学相位权值须比静态 0.01273 更紧，滤波才跟得上动态
        mConfig.firstEpochPosVar = 1.0; // 首历元位置先验方差(σ=1m，松→紧适中)

        // —— 保留 Oct-15 HEAD 的历史默认值（地面/LEO 共用基类初值）——
        // posNoise / sigIFCode / ztdInitVar 是「按场景」参数，不在此硬编码为地面最优值，
        // 以免破坏 LEO：LEO 以 8km/s 机动，posNoise 须远大于地面(地面最优 10 对 LEO 是灾难性过紧→
        // EKF 无法容纳每历元 7.6km 位移→首历元后即发散，leo_cli 仅解出 1 历元)。Oct-15 工作版即 posNoise=100。
        // 地面运动学由 ppp_cli 的 ini(pos_noise=10 / sig_if_code=0.3 / ztd_init_var=0.0144) 覆盖；
        // LEO 由 LEO.h + leo_run.ini 提供其专属值。
        mConfig.sigIFCode = 0.1;
        mConfig.posNoise = 100.0;
        // ztdInitVar 不在此设置，沿用 SPP 默认 0.36（地面由 ini 覆盖为 0.0144；LEO 无真实对流层，留默认即可）
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
