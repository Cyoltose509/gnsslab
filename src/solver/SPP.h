#pragma once

#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/SatID.h"
#include "GnssSolver.h"
#include "core/geom/EphemerisTable.h"
#include "core/obs/FreqCombo.h"
#include "core/obs/Weight.h"
#include "SPPConfig.h"
#include "SolverOptions.h"
#include "TropoSite.h"
#include "Products.h"
#include <memory>
#include <set>


using IFCodeTypes = std::map<SatSystem, FreqCombo>;


class SPP : public GnssSolver {
public:
    ~SPP() override = default;

    void applyOptions(const SolverOptions &o) override;

    virtual bool processEpoch(ObsData &obsData);

    virtual void computeSatPos();

    void prepareGeometry() override;

    [[nodiscard]] bool tropoGridReady() const { return prod->tropo.readyFor(mConfig.tropoMap); }

    void detectIFCombinations(const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    void setISBProcSigma(const double proc, const double init) {
        mConfig.isbProcSigma = proc;
        mConfig.isbInitSigma = init;
    }

    void setEnabledSystems(const std::set<SatSystem> &s) { mConfig.enabledSystems = s; }

    [[nodiscard]] double getClockBias(const SatSystem sys) const {
        const auto it = rClockBias.find(sys);
        return it != rClockBias.end() ? it->second : 0.0;
    }

    void publishResult() override;

    void resetFilterState() override;

    void initFilter() override;

    void updateFilter() override;

    bool bootstrapSolution() override;

    void filterEpoch();

    DopplerResult estimateVelocity() override;

    [[nodiscard]] Weight dopplerWeight() const override;

    void selectClockModel(const std::set<SatSystem> &active);
    void addClockTerms(EquData &ed, const SatID &sat) const;
    void collectClockStates(EquSys &eq);

    [[nodiscard]] bool wantDopplerSat(const SatID &sat, const TypeValueMap &tv) const override;

    [[nodiscard]] Eigen::Vector3d currentEstimatedPosition() const;

    [[nodiscard]] const FreqCombo *freqComboFor(const SatID &sat) const {
        if (sat.system == SatSystem::R) {
            const auto it = mGloCombo.find(sat);
            return it == mGloCombo.end() ? nullptr : &it->second;
        }
        const auto it = ifCodeTypes.find(sat.system);
        return it == ifCodeTypes.end() ? nullptr : &it->second;
    }

    SPPConfig mConfig;
    Weight mWeight;
    bool mWeightInit = false;
    bool mRequirePhaseForIF = false;
    bool mKinematic = false;

    std::set<SatID> satRejected{};

    std::map<SatSystem, double> rClockBias{};
    std::map<SatSystem, double> mPrevClockBias{};

    bool mCommonClockActive = false;
    SatSystem mClockRefSys = SatSystem::G;
    std::map<SatSystem, Parameter> mCdtParam = sysCdtParams;

    IFCodeTypes ifCodeTypes{};
    std::map<SatID, FreqCombo> mGloCombo;
    std::map<SatSystem, std::string> ifCodeSig;
    std::set<SatSystem> activeSystems;
    std::map<SatID, double> lastWStats;

    virtual void buildEquSys();
    virtual void addSatEquation(const SatID &sat, const TypeValueMap &tv, bool probe);

private:

    TropoSite mTropoSite;
    Eigen::Vector3d mLastXyz{0, 0, 0};

    [[nodiscard]] Eigen::Vector3d estimateRoughPosition() const;

    [[nodiscard]] double tropoDelay(double elevRad) const;
};
