#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "FreqCombo.h"
#include "EphemerisTable.h"
#include "Ephemeris.h"
#include "CycleSlip.h"
#include "RTKConfig.h"
#include "SolverOptions.h"
#include <Eigen/Eigen>
#include <map>
#include <set>
#include <vector>
#include <optional>


class RTK {
public:
    RTK() = default;

    ~RTK() = default;

    RTKConfig mConfig;
    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }


    void setConfig(const RTKConfig &c) { mConfig = c; }

    void applyOptions(const SolverOptions &o);

    /// 选参考星
    [[nodiscard]] std::optional<SatID> selectRtkRefSat(const std::vector<SatID> &candidates) const;


    Eigen::Vector3d mBaseXyz{0, 0, 0}; // 基准站
    Eigen::Vector3d mBaseXyzUser{0, 0, 0}; // 配置页显式填入的基站坐标
    Eigen::Vector3d mRoverApprox{0, 0, 0}; // 流动站
    int mGfCommonMode = 2; // GF 共模抑制阈值

    std::size_t mBaseEphSize = 0;
    std::size_t mRoverEphSize = 0;


    Eigen::Vector3d mBaselineFloat = Eigen::Vector3d::Zero(); // 浮点基线
    Eigen::Vector3d mBaselineFixed = Eigen::Vector3d::Zero(); // 固定基线
    Eigen::Vector3d mBaselineSigmaFixed = Eigen::Vector3d::Zero(); // 固定基线标准差

    Result result;

    std::map<SatID, double> satElevData;
    std::map<SatID, double> satAzimData;
    std::map<SatID, double> satRoverElevData; // 流动站仰角：参考星按流动站高度角最大选取
    std::map<SatID, PVT> satPVTTransTime;
    std::map<SatID, PVT> satPVTRecTime; // 旋转到接收时刻 ECEF 框架，供仰角与双差几何使用

    // 站间单差方差
    struct SdVar {
        double phase = 0.0, code = 0.0;
    };

    std::map<SatID, SdVar> mSdVar;
    Eigen::MatrixXd mWeightMatrix;
    bool mWeightReady = false;

    bool processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph);


    std::set<SatID> mUsedSats;
    std::map<char, SatID> mRtkRefSat; // 各系统参考星

private:
    struct SysCombo {
        FreqCombo base, rover;
        bool ok = false;
    };

    // 双差方程缓存
    struct DDCache {
        SatID sat, ref;
        char sys = 0;
        PVT pvt;                         // 本星 PVT
        double obsBase[4] = {0, 0, 0, 0};   // [L1,L2,C1,C2] 基准站观测值
        double obsRover[4] = {0, 0, 0, 0};  // 流动站观测值
        double refObsBase[4] = {0, 0, 0, 0};
        double refObsRover[4] = {0, 0, 0, 0};
        double phaseVar = 0, codeVar = 0;
        double refPhaseVar = 0, refCodeVar = 0;
        Variable amb[2];                 // 相位 freq1/freq2 模糊度变量
        double lambda[2] = {0, 0};       // freq1/freq2 波长
        PVT refPvt;                      // 参考星 PVT（迭代内几何项重算用）
    };

    const ObsData *mRtkBase = nullptr;
    const ObsData *mRtkRover = nullptr;
    std::map<char, SysCombo> mRtkCombos;
    std::map<char, std::vector<SatID> > mRtkPartSats;
    std::map<char, std::map<std::string, std::map<SatID, double> > > mRtkFixedMap;
    EphemerisTable *mEph = nullptr;
    std::set<SatID> mRtkSlipExclude;

    std::vector<DDCache> mDDCache;
    std::map<char, PVT> mCachedRefPvt;

    bool solveFloat(EquSys &eq, SolverLSQ &solver, Eigen::Vector3d &baseline);

    /// 站间单差方差
    void computeSdVariance();

    /// 构造双差随机模型
    void buildStochasticModel(const EquSys &eq);

    /// 构造本历元满阵权：仅完整协方差模式需要，对角模式直接用方程标量权。
    void buildWeightMatrix(const EquSys &eq);

    void solveIteration(SolverLSQ &solver, EquSys &eq) const;

    bool tryFixAmbiguities(const EquSys &eqFloat, SolverLSQ &solverFloat);


    void buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, bool fixed);

    void detectFreqCombinations();

    bool buildGeometry();

    void buildDDCache();

    void readback(const EquSys &eqFloat, const SolverLSQ &solverFloat, const Eigen::Vector3d &posEcef);

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
};
