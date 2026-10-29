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

        ekf.clockInitVar = 3600.0;
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

    void applyOptions(const SolverOptions &o) override;

    Eigen::Vector3d comToArpECEF(const CommonTime &epoch) const {
        return recvAntennaOffsetECEF(mRefPos, epoch);
    }

protected:
    Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

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
