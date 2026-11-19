#pragma once

#include "core/equ/EquStruct.h"
#include "core/equ/Result.h"
#include "core/obs/ObsStruct.h"
#include "core/obs/Weight.h"
#include "core/geom/CoordStruct.h"
#include "core/geom/Ephemeris.h"
#include "core/SatID.h"
#include "GnssEKF.h"
#include "SolverLSQ.h"
#include "SolverOptions.h"
#include <map>
#include <memory>
#include <set>

struct Products;


class GnssSolver {
public:
    /// 超过这个间隔，运动外推已不代表本历元状态
    static constexpr double kMaxMotionGapS = 60.0;
    static constexpr double kDopplerWeightScale = 1.0 / 40.0;
    static constexpr int kMinDopplerSats = 4;
    /// 解算中断超过这么久就认为相位弧段不可信
    static constexpr double kAmbSolveGapReset = 5.0;

    struct DopplerResult {
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        double clockDrift = 0.0;
        Eigen::Vector3d sigma = Eigen::Vector3d::Zero();
        int used = 0;
        bool ok = false;
    };

    GnssSolver();

    virtual ~GnssSolver() = default;


    std::shared_ptr<Products> prod;

    Result result{};


    const ObsData *mCurObs = nullptr;


    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d mLastXYZ = Eigen::Vector3d::Zero();
    GeodeticFrame frame = Frame::WGS84;

    GnssEKF ekf;
    SolverLSQ lsq;
    EquSys curEq{};
    bool mUseFilter = false;
    bool mFilterInit = false;
    double sigma0 = 1.0;

    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};

    std::set<SatID> mHealthWarned;


    int mDirection = 1;
    bool mFilterFrozen = false;
    bool mTrackEpochGap = false; // 跨历元断档判定只对 PPP 系有意义
    bool mGapThisEpoch = false;
    bool mForceAmbReset = false;
    double mNominalInterval = -1.0;
    double mKeepAmbMaxGap = 0.0;
    CommonTime mLastEpochTime{};
    CommonTime mLastObsTime{};
    bool mHasLastEpoch = false;
    bool mHasLastObs = false;


    bool mUseCycleSlip = true;
    bool mUseGFCycleSlip = true;
    double mGFSlipMult = 3.0;
    int mGFCommonModeMinSats = 2;


    virtual void solve();

    virtual void applyOptions(const SolverOptions &o) = 0;

    virtual void prepareGeometry() = 0;
    virtual void publishResult() = 0;
    virtual void resetFilterState() = 0;
    virtual bool bootstrapSolution() = 0;
    virtual void initFilter() = 0;
    virtual void updateFilter() = 0;

    void trackEpochGap();

    virtual void estimatePosition();

    virtual DopplerResult estimateVelocity();

    void earthRotation(std::map<SatID, PVT> &dst, const Eigen::Vector3d &recv);

    void computeElevAzim(const std::map<SatID, PVT> &src, const Eigen::Vector3d &recv,
                         SatValueMap &elev, SatValueMap &azim) const;

    bool ephUnhealthy(const SatID &sat, const Ephemeris *eph, bool enabled);

    [[nodiscard]] Eigen::Matrix3d posCovariance() const;

    void readbackResidual(const EquSys &eq, const Eigen::VectorXd &residual);


    [[nodiscard]] virtual Weight dopplerWeight() const { return Weight{}; }

    [[nodiscard]] virtual bool wantDopplerSat(const SatID &, const TypeValueMap &) const {
        return true;
    }
};
