#pragma once

#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/equ/Result.h"
#include "core/SatID.h"
#include "SolverLSQ.h"
#include "GnssSolver.h"
#include "core/geom/Ephemeris.h"
#include "core/geom/EphemerisTable.h"
#include "core/obs/FreqCombo.h"
#include "core/obs/Weight.h"
#include "SPPConfig.h"
#include "SolverOptions.h"
#include "TropoSite.h"
#include "GnssEKF.h"
#include "Products.h"
#include <Eigen/Eigen>
#include <memory>
#include <set>


using IFCodeTypes = std::map<char, FreqCombo>;

class SPP : public GnssSolver {
protected:
    SPPConfig mConfig;
    Weight mWeight;
    bool mWeightInit = false;

public:
    [[nodiscard]] bool tropoGridReady() const { return prod->tropo.readyFor(mConfig.tropoMap); }

    void detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    void setISBProcSigma(const double proc, const double init) {
        mConfig.isbProcSigma = proc;
        mConfig.isbInitSigma = init;
    }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }

    virtual void applyOptions(const SolverOptions &o);

    virtual bool processEpoch(ObsData &obsData);

    virtual void preprocess(ObsData &obsData);

protected:
    virtual void solve(ObsData &obsData);
    virtual void estimatePosition(ObsData &obsData);
    virtual void resetFilterState();
    virtual void initFilter(ObsData &obsData);
    virtual void updateFilter(ObsData &obsData);
    void bootstrapSolution(ObsData &obsData);

    void runFilter(ObsData &obsData);

    void readbackFiltered(const ObsData &obsData);

    [[nodiscard]] static Eigen::Matrix3d positionCovBlock(const Eigen::MatrixXd &P, const int id[3]);

public:
    virtual void computeSatPos(ObsData &obsData);

    void computeElevAzim();

    void earthRotation();

    [[nodiscard]] double getClockBias(const char sys) const {
        const auto it = rClockBias.find(sys);
        return it != rClockBias.end() ? it->second : 0.0;
    }

    virtual ~SPP() = default;

    Eigen::Vector3d xyz{0, 0, 0};
    GeodeticFrame frame = Frame::WGS84;

    GnssEKF ekf;
    int mDirection = 1;
    bool mUseFilter = false;
    bool mFilterInit = false;
    double sigma0 = 1.0;

    [[nodiscard]] virtual Eigen::Matrix3d posCovariance() const;
    std::set<SatID> satRejected{};

protected:
    bool mRequirePhaseForIF = false;

    std::map<char, double> rClockBias{};
    std::map<char, double> mPrevClockBias{};

    CommonTime mLastEpoch{};
    bool mHasLastEpoch = false;

    static inline std::map<char, Parameter> sysCdtParam = {
        {'G', Parameter::cdt},
        {'C', Parameter::cdt2},
        {'R', Parameter::cdt3},
        {'E', Parameter::cdt4},
    };

    [[nodiscard]] static char clockRefSys(const std::set<char> &active);

    bool mCommonClockActive = false;
    char mClockRefSys = 'G';
    std::map<char, Parameter> mCdtParam = sysCdtParam;

    [[nodiscard]] virtual bool wantCommonClock() const;

    void remapClockParams(char ref);

    void toCommonClockEquations(const ObsData &obsData);

    [[nodiscard]] Eigen::Vector3d currentEstimatedPosition() const;

    IFCodeTypes ifCodeTypes{};

    std::map<SatID, FreqCombo> mGloCombo;

    [[nodiscard]] const FreqCombo *freqComboFor(const SatID &sat) const {
        if (sat.system == 'R') {
            const auto it = mGloCombo.find(sat);
            return it == mGloCombo.end() ? nullptr : &it->second;
        }
        const auto it = ifCodeTypes.find(sat.system);
        return it == ifCodeTypes.end() ? nullptr : &it->second;
    }

    std::map<char, std::string> ifCodeSig;

    std::set<char> activeSystems;


    std::map<SatID, double> lastWStats;
    std::map<SatID, double> lastResidMag;


    virtual void buildResidualMap();

    void estimateVelocity(const ObsData &obsData);

    bool wantDopplerSat(const SatID &sat, const TypeValueMap &tv) const override;

private:
    TropoSite mTropoSite;


    Eigen::Vector3d mLastXyz{0, 0, 0};


    [[nodiscard]] Eigen::Vector3d roughInitialXyz(const ObsData &obsData) const;

    void updateTropoSite(const Eigen::Vector3d &xyzEcef, const CommonTime &epoch);

    [[nodiscard]] double tropoDelay(double elevRad) const;

    EquSys posEquations{};
    SolverLSQ posSolver{};

    void linearize(ObsData &obsData, int iter);

    void buildPosEquation(const SatID &sat, const PVT &pvt, const Eigen::Vector3d &los,
                          double rho, double trop,
                          const std::string &obsType, double obsVal, double weight,
                          const Variable &dx, const Variable &dy,
                          const Variable &dz, const Variable &cdtVar);
};
