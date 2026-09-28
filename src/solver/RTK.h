#pragma once

#include "GnssStruct.h"
#include "SolverLSQ.h"
#include "FreqCombo.h"
#include "EphemerisTable.h"
#include "Ephemeris.h"
#include "CoordConvert.h"
#include "CycleSlip.h"
#include "Weight.h"
#include <Eigen/Eigen>
#include <map>
#include <set>
#include <algorithm>
#include <vector>


class RTK {
public:
    RTK() = default;

    ~RTK() = default;

    // 解算器配置：集中魔法常量，替代散落的裸字面量与独立成员
    struct Config {
        double cutoffElevRad = 10.0 * PI / 180.0; // 截止高度角(rad)
        std::set<char> enabledSystems = {'G', 'C'}; // 参与解算的星座集合
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
        double offDiagScale = 1.0; // 同类型双差相关项(σ²_ref)缩放：1=老师完整模型，0=退化为对角
        bool excludeGeoRef = true; // 参考星排除北斗 GEO
        Weight phaseWeight{0.004, 0.003, 1e-3}; // 相位高度角方差模型（teacher 系数）
    };

    Config mConfig;
    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * PI / 180.0; }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }


    void setConfig(const Config &c) { mConfig = c; }

    /// 双站历元时间对齐
    static std::vector<std::pair<size_t, size_t> > matchEpochPairs(
        const std::vector<ObsData> &base, const std::vector<ObsData> &rover,
        double tolSec = 0.001);

    static double measStd(const double elev, const double sigma0, const double minSinElev) {
        const double s = (std::max)(std::sin(elev), minSinElev);
        return sigma0 * (1.0 + 1.0 / s);
    }


    Vector3d mBaseXyz{0, 0, 0}; // 基准站
    Vector3d mRoverApprox{0, 0, 0}; // 流动站
    int mGfCommonMode = 2; // GF 共模抑制阈值


    Vector3d mBaselineFloat = Vector3d::Zero(); // 浮点基线
    Vector3d mBaselineFixed = Vector3d::Zero(); // 固定基线
    Vector3d mBaselineSigmaFixed = Vector3d::Zero(); // 固定基线标准差

    Result result;

    std::map<SatID, double> satElevData;
    std::map<SatID, double> satAzimData;
    std::map<SatID, double> satRoverElevData; // 流动站仰角：参考星按流动站高度角最大选取
    std::map<SatID, PVT> satPVTRecTime;

    // 站间单差方差（本历元参与星，含参考星）：相位与伪距各一份
    struct SdVar {
        double phase = 0.0, code = 0.0;
    };

    std::map<SatID, SdVar> mSdVar;
    MatrixXd mDDWeight; // 双差权阵 P = R⁻¹（方程数×方程数）
    bool mDDWeightReady = false; // 卫星集合与仰角不变时复用，避免迭代内反复求逆

    bool processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph);


    std::set<SatID> mUsedSats;

private:
    struct SysCombo {
        FreqCombo base, rover;
        bool ok = false;
    };

    const ObsData *mRtkBase = nullptr;
    const ObsData *mRtkRover = nullptr;
    std::map<char, SysCombo> mRtkCombos;
    std::map<char, SatID> mRtkRefSat;
    std::map<char, std::vector<SatID> > mRtkPartSats;
    std::map<char, std::map<std::string, std::map<SatID, double> > > mRtkFixedMap;
    EphemerisTable *mEph = nullptr;
    std::set<SatID> mRtkSlipExclude;

    bool solveFloat(EquSys &eq, SolverLSQ &solver, Vector3d &baseline);

    /// 站间单差方差（每系统每星，含参考星）。
    void computeSdVariance();

    /// 构造双差随机模型：单差方差 → 双差方差协方差阵 R → 求逆得权阵 P（GLS）。
    /// 行序与 eq.obsEquData 的遍历顺序一致，结果存 mDDWeight。
    void buildStochasticModel(const EquSys &eq);

    /// 首次组装方程时构建满阵权并回填 ed.weight = P 的对角元。
    void setupFullWeight(EquSys &eq);

    bool tryFixAmbiguities(const EquSys &eqFloat, SolverLSQ &solverFloat);

    void buildEquSys(EquSys &eq, const Vector3d &roverPos, bool fixed);

    void detectFreqCombinations();

    bool buildGeometry();

    void readback(const EquSys &eqFloat, const SolverLSQ &solverFloat, const Vector3d &posEcef);

    void detectSlips();

    void addDoubleDiffEquation(EquSys &eq, const SatID &sat,
                               const Vector3d &ddLos, double ddGeom,
                               int freq, bool isPhase, bool fixed) const;


    static const char *freqLabel(const int freq, const bool isPhase) {
        return freq == 1 ? (isPhase ? "L1" : "C1") : (isPhase ? "L2" : "C2");
    }

    static Variable ambVar(const SatID &sat, const int freqIdx) {
        return makeAmbiguityVar("rover", sat, freqLabel(freqIdx, true));
    }

    std::map<SatID, CycleSlip> mSlipDetBase;
    std::map<SatID, CycleSlip> mSlipDetRover;
};
