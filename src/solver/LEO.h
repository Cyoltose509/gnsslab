#pragma once

#include "PPPKinematic.h"
#include "Sp3OrbitReader.h"
#include <Eigen/Eigen>


class LEO : public PPPKinematic {
public:
    LEO() {
        setTropo(false);
        mReanchorEveryEpoch = true;
        mMotionThresh = 100.0;

        // 首历元钟差初始方差1e8
        ekf.clockInitVar = 1.0E8;
        // 首历元位置先验方差 1.0 m²
        mConfig.firstEpochPosVar = 1.0;
        mConfig.ambNoise = 1.0E-7;
        mConfig.sigmaPhase = 0.003;
        mConfig.minSatsForUpdate = 4;
        mApplySolidTide = false;
        mApplySatPCO = false;
        mFirstEpochSPP = true;
    }

    ~LEO() override = default;


    void setApproxPosition(const Vector3d &p) { mRefPos = p; }

    bool loadAuxFile(const std::string &path);

    bool loadLeoReference(const std::string &path, const SatID &sat = SatID());

    PVT getReferencePVT(const CommonTime &epoch) const;

    bool processEpoch(ObsData &obs) override;

protected:
    Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    // 星载 CoG→ARP 杠杆臂(ECEF)
    Vector3d recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const override;

    // 首历元引导
    void bootstrapFirstEpoch(ObsData &obsData) override;

    static Matrix3d BODY2ECEF(const Vector3d &leoPos, const Vector3d &leoVel);

private:
    Vector3d mLeoAntOffset = Vector3d::Zero(); // 机体杠杆臂
    bool mHasAux = false;
    Sp3OrbitReader mRefRdr; // 参考轨道
    SatID mRefSat; // 参考卫星
    bool mHasRef = false; // 是否加载了参考轨道

    mutable Vector3d mLastGoodAnchor = Vector3d::Zero(); // 质量门控重锚定：最近一个可信锚点
    mutable bool mSppGood = true; // 本历元 SPP 重锚定质量是否达标
};
