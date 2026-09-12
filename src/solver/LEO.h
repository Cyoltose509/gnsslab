#pragma once

#include "PPP.h"
#include "Sp3OrbitReader.h"
#include <Eigen/Eigen>


class LEO : public PPP {
public:
    LEO() {
        setKinematic(true); // 纯运动学
        setTropo(false); // LEO无对流层湿延迟

        // 首历元钟差初始方差1e8
        ekf.setClockInitVar(1.0E8);
        // 首历元位置先验方差 1.0 m²
        mFirstEpochPosVar = 1.0;
        mAmbNoise = 1.0E-7;
        mSigmaPhase = 0.003;
        mMinSatsForUpdate = 4;
        mApplySolidTide = false;
        mApplySatPCO = false;
        mFirstEpochSPP = true;
    }

    ~LEO() override = default;

    // 首历元 SPP 重锚定的初始种子
    void setApproxXYZ(const Vector3d &p) { mRefPos = p; }

    bool loadAuxFile(const std::string &path);

    bool loadLeoReference(const std::string &path, const SatID &sat = SatID());

    // 参考轨道在 epoch 的 PVT
    PVT getReferencePVT(const CommonTime &epoch) const;

    bool process(ObsData &obs) override;

protected:
    Vector3d predictPosition(double dt, const CommonTime &epoch) const override;

    // 星载 CoG→ARP 杠杆臂(ECEF)
    Vector3d recvAntennaOffsetECEF(const Vector3d &xyzEst, const CommonTime &epoch) const override;

    // 相位缠绕接收机天线基
    void recvWindupBasis(const SatID &sat, const Vector3d &recv, const CommonTime &epoch,
                         Vector3d &exr, Vector3d &eyr) const override;

    // 首历元引导
    void bootstrapFirstEpoch(ObsData &obsData) override;

    static Matrix3d BODY2ECEF(const Vector3d &leoPos, const Vector3d &leoVel);


private:
    Vector3d mLeoAntOffset = Vector3d::Zero(); // 机体杠杆臂
    bool mHasAux = false;
    Sp3OrbitReader mRefRdr; // 参考轨道
    SatID mRefSat; // 参考卫星
    bool mHasRef = false; // 是否加载了参考轨道

    mutable Vector3d mLeoVel = Vector3d::Zero(); // 有限差分速度
    mutable Vector3d mPrevEstPos = Vector3d::Zero(); // 上一历元估计位置
    mutable CommonTime mPrevEpoch; // 上一历元时刻
    mutable bool mHasPrev = false;

    mutable Vector3d mReanchorPos = Vector3d::Zero();
    mutable Vector3d mLastGoodAnchor = Vector3d::Zero();
    double mDivergeJumpThresh = 500.0;
    long mDivergeRecovers = 0;
    long mExcludedEpochs = 0;
    mutable bool mSppGood = true;

    const ObsData *mCurObs = nullptr;

    Vector3d applyReanchorResult(Vector3d xf, int n, double q, const Vector3d &fresh) const;

    std::tuple<Vector3d, int, double> sppReanchor(const Vector3d &seed) const;
};
