#pragma once

#include "PPPKinematic.h"
#include "Sp3OrbitReader.h"
#include <cstdlib>
#include <Eigen/Eigen>


class LEO : public PPPKinematic {
public:
    LEO() {
        setTropo(false);
        mReanchorEveryEpoch = true;
        mMotionThresh = 100.0;

        // 每历元重初始化的钟差先验方差：对齐 RTKLIB ppp.c VAR_CLK=SQR(60.)=3600(σ=60m)。
        // 曾用 1e8(σ=10km)：H 的零方向 δ=(cdt+1, codeBias−1, 所有 B−1) 只被 10km 先验钉住，
        // 模糊度边际方差永不收敛（σ_amb≈0.045·√clockInitVar：1e8→14.5m，3600→2.6m，100→0.45m），
        // AR 门(arSigGate=5)把全部星挡在门外 ⇒ LAMBDA 从不运行。Swarm-C 24h A/B：
        // 3600 vs 1e8 连续弧段 3D RMS 0.740 vs 0.738m（+0.3%，无回归），再紧(100/10)才伤解
        // （钟差↔径向位置退化：0.842/1.795m）。
        ekf.clockInitVar = 3600.0;
        // [实验旋钮] GNSSLAB_LEO_CLKVAR 覆盖钟差先验方差（A/B 用，定稿后删除）。
        if (const char *e = std::getenv("GNSSLAB_LEO_CLKVAR")) ekf.clockInitVar = std::atof(e);
        // 首历元位置先验方差 1.0 m²
        mConfig.firstEpochPosVar = 1.0;
        mConfig.ambNoise = 1.0E-7;
        mConfig.sigmaPhase = 0.003;
        mConfig.minSatsForUpdate = 4;
        mApplySolidTide = false;
        mPinInitHeight = false;
        mUseCycleSlip = false;
    }

    ~LEO() override = default;


    bool loadAuxFile(const std::string &path);

    bool loadLeoReference(const std::string &path, const SatID &sat = SatID());

    bool processEpoch(ObsData &obs) override;

public:
    /// 参考轨道给的是质心 CoM，本类解的是天线相位中心 ARP。
    /// 评测方(harness_leo)需要同口径比较时用它把 CoM 真值转到 ARP；无辅助文件则返回原点。
    Eigen::Vector3d comToArpECEF(const CommonTime &epoch) const {
        return recvAntennaOffsetECEF(mRefPos, epoch);
    }

protected:
    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    // 星载 CoG→ARP 杠杆臂(ECEF)
    Eigen::Vector3d recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const override;

    // 首历元引导
    void bootstrapFirstEpoch(ObsData &obsData) override;

private:
    Eigen::Vector3d mLeoAntOffset = Eigen::Vector3d::Zero(); // 机体杠杆臂
    bool mHasAux = false;
    Sp3OrbitReader mRefRdr; // 参考轨道
    SatID mRefSat; // 参考卫星
    bool mHasRef = false; // 是否加载了参考轨道

    mutable Eigen::Vector3d mLastGoodAnchor = Eigen::Vector3d::Zero(); // 质量门控重锚定：最近一个可信锚点
    mutable bool mSppGood = true; // 本历元 SPP 重锚定质量是否达标
};
