#pragma once

#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/SatID.h"
#include "SolverLSQ.h"
#include "core/obs/FreqCombo.h"
#include "core/geom/EphemerisTable.h"
#include "core/geom/Ephemeris.h"
#include "core/obs/CycleSlip.h"
#include "GnssEKF.h"
#include "GnssSolver.h"
#include "RTKConfig.h"
#include "SolverOptions.h"
#include <map>
#include <set>
#include <vector>
#include <optional>


class RTK : public GnssSolver {
public:
    RTK() = default;

    ~RTK() override = default;

    RTKConfig mConfig;

    void applyOptions(const SolverOptions &o);

    bool processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph);

    Eigen::Vector3d mBase{0, 0, 0}; // 基准站
    Eigen::Vector3d mBaseUser{0, 0, 0}; // 配置页显式填入的基站坐标

    Eigen::Vector3d mBaselineFloat = Eigen::Vector3d::Zero(); // 浮点基线
    Eigen::Vector3d mBaselineFixed = Eigen::Vector3d::Zero(); // 固定基线
    std::set<SatID> mUsedSats;

private:
    GnssEKF ekf;

    bool mUseFilter = false;
    bool mFilterInit = false;
    double mPosProcSigma = 10.0;

    Eigen::Vector3d mFilterXYZ{0, 0, 0};


    [[nodiscard]] std::optional<SatID> selectRefSatelite(const std::vector<SatID> &candidates) const;

    Eigen::Vector3d mRoverApprox{0, 0, 0};
    int mGfCommonMode = 2;
    std::size_t mBaseEphSize = 0;
    std::size_t mRoverEphSize = 0;
    Eigen::Vector3d mBaselineSigmaFixed = Eigen::Vector3d::Zero();

    std::map<SatID, double> satRoverElevData;
    std::map<SatID, PVT> satPVTRoverRecTime;


    struct SdVar {
        double phase = 0.0, code = 0.0;
    };

    std::map<SatID, SdVar> mSdVar;
    Eigen::MatrixXd mWeightMatrix;
    bool mWeightReady = false;

    std::map<char, SatID> mRefSat;

    struct SysCombo {
        FreqCombo base, rover;
        bool ok = false;
    };


    struct FloatState {
        EquSys eq{};
        Eigen::VectorXd x{};
        Eigen::MatrixXd P{};
        Eigen::VectorXd v{};
        std::map<Variable, int> index{};
        double sigma0 = 1.0;
    };


    struct DDCache {
        SatID sat, ref;
        char sys = 0;
        PVT pvtBase, pvtRover;
        double obsBase[4] = {0, 0, 0, 0};
        double obsRover[4] = {0, 0, 0, 0};
        double refObsBase[4] = {0, 0, 0, 0};
        double refObsRover[4] = {0, 0, 0, 0};
        double phaseVar = 0, codeVar = 0;
        double refPhaseVar = 0, refCodeVar = 0;
        Variable amb[2];
        double ambInit[2] = {0, 0};
        double lambda[2] = {0, 0};
        PVT refPvtBase, refPvtRover;
    };

    const ObsData *mRtkBase = nullptr;
    const ObsData *mRtkRover = nullptr;
    std::map<char, SysCombo> mRtkCombos;
    std::map<char, std::vector<SatID> > mPartSats;
    std::map<char, std::map<std::string, std::map<SatID, double> > > mRtkFixedMap;
    EphemerisTable *mEph = nullptr;
    std::set<SatID> mRtkSlipExclude;

    std::vector<DDCache> mDDCache;


    FloatState mFloat;


    bool solveFloat(EquSys &eq, Eigen::Vector3d &baseline);


    void initFilter();


    void updateFilter();


    void rebaseAmbiguities();


    void resetFilterState();


    void collectVars(EquSys &eq, bool fixed) const;


    void applyFixedSolution(double s0);


    [[nodiscard]] Eigen::Vector3d filteredPosition() const;


    [[nodiscard]] Eigen::Vector3d predictedPosition() const;


    [[nodiscard]] static Eigen::Matrix3d positionCov(const FloatState &fs);


    void captureFilteredState();


    static void extractAmbiguities(const FloatState &fs, Eigen::VectorXd &b, Eigen::MatrixXd &Qb,
                                   std::vector<Variable> &aVars);


    void computeSdVariance();


    void buildStochasticModel(const EquSys &eq);


    void buildWeightMatrix(const EquSys &eq);

    void solveIteration(SolverLSQ &solver, EquSys &eq) const;

    bool tryFixAmbiguities(const std::vector<Variable> &aVars, const Eigen::VectorXd &b,
                           const Eigen::MatrixXd &Qb);


    struct AmbSeed {
        std::map<Variable, double> init;
        VariableDataMap novar;
    };


    void collectEpochStates(VariableSet &varSet, AmbSeed &seed) const;


    void buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, bool fixed);

    void detectFreqCombinations();

    bool buildGeometry();

    void buildDDCache();


    bool estimatePosition();

    void readback(const EquSys &eqFloat, const Eigen::VectorXd &residual, const Eigen::Vector3d &posEcef);


    void estimateVelocity();

    void detectSlips();

    void addDoubleDiffEquation(EquSys &eq, const DDCache &c,
                               const Eigen::Vector3d &ddLos, double ddGeom,
                               int freq, bool isPhase, bool fixed) const;


    static const char *freqLabel(const int freq, const bool isPhase) {
        return freq == 1 ? (isPhase ? "L1" : "C1") : isPhase ? "L2" : "C2";
    }

    static Variable ambVar(const SatID &sat, const int freqIdx) {
        return makeAmbiguityVar("rover", sat, freqLabel(freqIdx, true));
    }

    static Eigen::Vector3d estimateApproxPosition(const ObsData &obs, const EphemerisTable &eph,
                                                  const std::set<char> &systems, double cutoffElevDeg);

    std::map<SatID, CycleSlip> mSlipDetBase;
    std::map<SatID, CycleSlip> mSlipDetRover;

    std::map<char, SatID> mPrevRefSat;
};
