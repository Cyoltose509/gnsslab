#pragma once

#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/SatID.h"
#include "GnssSolver.h"
#include "core/geom/Ephemeris.h"
#include "core/geom/EphemerisTable.h"
#include "core/obs/FreqCombo.h"
#include "core/obs/Weight.h"
#include "SPPConfig.h"
#include "SolverOptions.h"
#include "TropoSite.h"
#include "Products.h"
#include <memory>
#include <set>


using IFCodeTypes = std::map<char, FreqCombo>;


class SPP : public GnssSolver {
public:
    ~SPP() override = default;

    void applyOptions(const SolverOptions &o) override;

    virtual bool processEpoch(ObsData &obsData);

    /// 发射时刻的广播星历卫星状态；PPP 用精密产品取代它
    virtual void computeSatPos();

    void prepareGeometry() override;

    [[nodiscard]] bool tropoGridReady() const { return prod->tropo.readyFor(mConfig.tropoMap); }

    /// 逐系统探测可用 IF 组合，结果落 ifCodeTypes / mGloCombo
    void detectIFCombinations(const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    void setISBProcSigma(const double proc, const double init) {
        mConfig.isbProcSigma = proc;
        mConfig.isbInitSigma = init;
    }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }

    [[nodiscard]] double getClockBias(const char sys) const {
        const auto it = rClockBias.find(sys);
        return it != rClockBias.end() ? it->second : 0.0;
    }

    void publishResult() override;

    void resetFilterState() override;

    void initFilter() override;

    void updateFilter() override;

    /// 首历元引导：IF 组合与钟模型定好后，用迭代 LSQ 解出位置与钟，写进 xyz / rClockBias。
    bool bootstrapSolution() override;

    void filterEpoch();

    DopplerResult estimateVelocity() override;

    [[nodiscard]] Weight dopplerWeight() const override;

    /// 本历元钟参数化唯一入口：公共钟开关、参考系统、各系统钟槽位一起定。
    void selectClockModel(const std::set<char> &active);

    /// 把本系统的钟系数写进方程：共钟写「公共钟 cdt1 + 非参考系统 ISB 槽」，否则写本系统槽位。
    /// SPP 的码方程与 PPP 的码/相位方程共用这一份，禁各写一遍。
    void addClockTerms(EquData &ed, const SatID &sat) const;

    /// 本历元的钟状态收进方程组：共钟只留公共钟 cdt1 与各非参考系统的 ISB 槽；独立钟各留自带槽位，
    /// 另用「ISB 与上一历元相同」的伪观测锚住其随机游走（系统间差异在方程内无约束）。
    void collectClockStates(EquSys &eq);

    [[nodiscard]] bool wantDopplerSat(const SatID &sat, const TypeValueMap &tv) const override;

    [[nodiscard]] Eigen::Vector3d currentEstimatedPosition() const;

    [[nodiscard]] const FreqCombo *freqComboFor(const SatID &sat) const {
        if (sat.system == 'R') {
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

    std::map<char, double> rClockBias{};
    std::map<char, double> mPrevClockBias{};

    bool mCommonClockActive = false;
    char mClockRefSys = 'G';
    std::map<char, Parameter> mCdtParam = sysCdtParams;

    IFCodeTypes ifCodeTypes{};
    std::map<SatID, FreqCombo> mGloCombo;
    std::map<char, std::string> ifCodeSig;
    std::set<char> activeSystems; // 本历元参与解算的系统：detectIFCombinations 按组合填，SPP 的 buildEquSys 再按实际建方程的星收窄
    std::map<SatID, double> lastWStats;

    /// 整历元建方程组：清 curEq、筛星、逐星调 addSatEquation、收 activeSystems 与状态集。
    virtual void buildEquSys();

    /// 给一颗星填本历元方程：SPP 只填码，PPP 覆写后填码+相位（probe 是预fit 探测趟）。
    virtual void addSatEquation(const SatID &sat, const TypeValueMap &tv, bool probe);

private:

    TropoSite mTropoSite;
    Eigen::Vector3d mLastXyz{0, 0, 0};

    /// 观测源没给近似坐标时的粗糙起点（方位对、量级对），供 LSQ 引导用。
    [[nodiscard]] Eigen::Vector3d estimateRoughPosition() const;

    [[nodiscard]] double tropoDelay(double elevRad) const;
};
