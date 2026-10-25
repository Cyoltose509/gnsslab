#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "FreqCombo.h"
#include "EphemerisTable.h"
#include "Ephemeris.h"
#include "CycleSlip.h"
#include "Weight.h"
#include <Eigen/Eigen>
#include <map>
#include <set>
#include <algorithm>
#include <vector>
#include <optional>


class RTK {
public:
    RTK() = default;

    ~RTK() = default;

    // 解算器配置
    struct Config {
        double cutoffElevRad = 10.0 * DEG_TO_RAD; // 截止高度角(rad)
        std::set<char> enabledSystems = {'G', 'R', 'E', 'C'}; // 参与解算的星座集合
        double arThreshold = 3.0; // LAMBDA 成功率阈值
        double minSinElev = 1e-3; // sin(el) 下限防除零
        double convEps = 1e-5; // 收敛阈值(m)
        int maxIterFloat = 8; // 浮点解最大迭代
        int maxIterFix = 6; // 固定解最大迭代
        int minSats = 4; // 双差解算最少卫星数
        bool useAR = true; // 模糊度固定开关
        bool useCycleSlip = true; // 周跳检测开关
        bool useGfCycleSlip = true; // GF 几何无关周跳检测

        double codeOverPhaseVar = 10000.0; // 伪距:相位 方差比
        bool fullCovariance = true; // 双差满阵 R 求逆做 GLS；false=对角近似
        bool excludeGeoRef = true; // 参考星排除北斗 GEO
        Weight phaseWeight{0.004, 0.003, 1e-3}; // 相位高度角方差模型

    };

    Config mConfig;
    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }


    void setConfig(const Config &c) { mConfig = c; }

    /// 选参考星
    static std::optional<SatID> selectRtkRefSat(const std::vector<SatID> &candidates,
                                                const std::map<SatID, double> &roverElev,
                                                bool excludeGeo);

    static double measStd(const double elev, const double sigma0, const double minSinElev) {
        const double s = (std::max)(std::sin(elev), minSinElev);
        return sigma0 * (1.0 + 1.0 / s);
    }


    Eigen::Vector3d mBaseXyz{0, 0, 0}; // 基准站
    Eigen::Vector3d mBaseXyzUser{0, 0, 0}; // 配置页显式填入的基站坐标
    Eigen::Vector3d mRoverApprox{0, 0, 0}; // 流动站
    int mGfCommonMode = 2; // GF 共模抑制阈值

    // SPP 自举精化记账（基准站/流动站线性化初值）。
    // 只要几何变好就重估，星历补全后即锁定。
    std::size_t mBaseEphSize = 0;   // 上次自举时已有星历的卫星数
    std::size_t mRoverEphSize = 0;


    Eigen::Vector3d mBaselineFloat = Eigen::Vector3d::Zero(); // 浮点基线
    Eigen::Vector3d mBaselineFixed = Eigen::Vector3d::Zero(); // 固定基线
    Eigen::Vector3d mBaselineSigmaFixed = Eigen::Vector3d::Zero(); // 固定基线标准差

    Result result;

    std::map<SatID, double> satElevData;
    std::map<SatID, double> satAzimData;
    std::map<SatID, double> satRoverElevData; // 流动站仰角：参考星按流动站高度角最大选取
    std::map<SatID, PVT> satPVTRecTime;

    // 站间单差方差
    struct SdVar {
        double phase = 0.0, code = 0.0;
    };

    std::map<SatID, SdVar> mSdVar;
    Eigen::MatrixXd mDDWeight; // 双差权阵 P = R⁻¹（块对角：每块 = 一个「系统+观测类型」）
    bool mDDWeightReady = false;
    std::vector<std::vector<int>> mDDWeightBlocks; // 上述块的行下标集合，供 SolverLSQ 逐块算 HᵀPH（等价加速）

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
        PVT pvt;                         // 本星 PVT（位置，迭代内几何项重算用）
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

    std::vector<DDCache> mDDCache;        // 本历元参与双差的非参考星缓存
    std::map<char, PVT> mCachedRefPvt;    // 各系统参考星 PVT

    bool solveFloat(EquSys &eq, SolverLSQ &solver, Eigen::Vector3d &baseline);

    /// 站间单差方差
    void computeSdVariance();

    /// 构造双差随机模型
    void buildStochasticModel(const EquSys &eq);

    /// 首次组装方程时构建满阵权并回填 ed.weight = P 的对角元。
    void setupFullWeight(EquSys &eq);

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
