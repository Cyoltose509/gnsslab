#pragma once

#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/SatID.h"
#include "core/obs/FreqCombo.h"
#include "core/geom/EphemerisTable.h"
#include "core/geom/Ephemeris.h"
#include "core/obs/SlipDetector.h"
#include "GnssSolver.h"
#include "RTKConfig.h"
#include "SolverOptions.h"
#include <map>
#include <set>
#include <vector>
#include <optional>

class RTK : public GnssSolver {
public:
    static constexpr auto kRover = "rover";
    static constexpr double kPrefitGateSigma = 4.0;
    static constexpr std::size_t kGateMinObs = 8;

    RTK() = default;
    ~RTK() override = default;

    RTKConfig mConfig;

    Eigen::Vector3d mBase{0, 0, 0};
    Eigen::Vector3d mBaseUser{0, 0, 0};
    Eigen::Vector3d mBaselineFloat = Eigen::Vector3d::Zero();
    Eigen::Vector3d mBaselineFixed = Eigen::Vector3d::Zero();
    std::set<SatID> mUsedSats;

    bool processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph);
    void applyOptions(const SolverOptions &o) override;
    void resetFilterState() override;
    void prepareGeometry() override;
    void initFilter() override;
    bool bootstrapSolution() override;
    void updateFilter() override;
    void publishResult() override;

private:
    struct SdVar {
        double phase = 0.0, code = 0.0;
    };

    struct SysCombo {
        FreqCombo base, rover;
        bool ok = false;
    };

    double mPosProcSigma = 10.0;
    Eigen::Vector3d mRoverApprox{0, 0, 0};
    std::size_t mBaseEphSize = 0;
    std::size_t mRoverEphSize = 0;
    Eigen::Vector3d mBaselineSigmaFixed = Eigen::Vector3d::Zero();

    // 基类的 satPVTRecTime/satElevData/satAzimData 存流动站，下面两套存基准站。
    std::map<SatID, double> satBaseElevData;
    std::map<SatID, PVT> satPVTBaseRecTime;

    std::map<SatID, SdVar> mSdVar;
    Eigen::MatrixXd mWeightMatrix;
    bool mWeightReady = false;
    std::map<SatSystem, SatID> mRefSat;
    std::map<SatSystem, SatID> mPrevRefSat;

    const ObsData *mBaseObs = nullptr;
    std::map<SatSystem, SysCombo> mSysCombos;
    std::map<SatSystem, std::vector<SatID> > mPartSats;
    VariableDataMap mFixedMap;
    EphemerisTable *mEph = nullptr;
    std::set<SatID> mSlipExclude;
    std::map<SatID, SlipDetector> mSlipDetBase;
    std::map<SatID, SlipDetector> mSlipDetRover;

    bool preparePositions(const ObsData &base, const ObsData &rover, const EphemerisTable &eph);
    [[nodiscard]] Eigen::Vector3d estimateApproxPosition(const ObsData &obs, const EphemerisTable &eph) const;
    void detectFreqCombinations();
    void detectSlips();

    [[nodiscard]] std::optional<SatID> selectRefSatellite(const std::vector<SatID> &part) const;
    void computeSdVariance();
    void collectFilterVariables(VariableSet &varSet, std::vector<AmbReset> &resets) const;
    void rebaseAmbiguities();
    void buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, bool fixed);
    void buildWeightMatrix(const EquSys &eq);
    void extractAmbiguities(Eigen::VectorXd &b, Eigen::MatrixXd &Qb, std::vector<Variable> &aVars) const;
    bool tryFixAmbiguities(const std::vector<Variable> &aVars, const Eigen::VectorXd &b,
                           const Eigen::MatrixXd &Qb);
    void applyFixedSolution(double s0);
    [[nodiscard]] Eigen::Vector3d getEKFPosition() const;
};
