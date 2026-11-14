#pragma once

#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/equ/Result.h"
#include "core/SatID.h"
#include "SolverLSQ.h"
#include "core/obs/FreqCombo.h"
#include "core/geom/EphemerisTable.h"
#include "core/geom/Ephemeris.h"
#include "core/obs/CycleSlip.h"
#include "GnssEKF.h"
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

    void applyOptions(const SolverOptions &o);

    bool processEpoch(const ObsData &base, const ObsData &rover, EphemerisTable &eph);

    Eigen::Vector3d mBaseXyz{0, 0, 0}; // 基准站
    Eigen::Vector3d mBaseXyzUser{0, 0, 0}; // 配置页显式填入的基站坐标

    Result result;

    Eigen::Vector3d mBaselineFloat = Eigen::Vector3d::Zero(); // 浮点基线
    Eigen::Vector3d mBaselineFixed = Eigen::Vector3d::Zero(); // 固定基线

    /// 结果侧的星历/几何快照：界面按历元取用，别在外部读内部状态。
    std::map<SatID, double> satElevData;
    std::map<SatID, double> satAzimData;
    std::map<SatID, PVT> satPVTTransTime;
    std::set<SatID> mUsedSats;

private:
    /// 滤波档：双差浮点解由 EKF 给（位置+模糊度跨历元约束），整周固定仍走 LSQ。
    /// 开关来自 `SolverOptions::filter`：none＝逐历元 LSQ，forward＝滤波；RTK 没有倒序重跑。
    GnssEKF ekf;
    bool mUseFilter = false;
    bool mFilterInit = false;
    double mPosProcSigma = 10.0; // 无速度态时的位置随机游走 σ (m/历元)
    /// 本历元滤波解（绝对 ECEF）；下一历元既是预测起点也是线性化点。
    Eigen::Vector3d mFilterXyz{0, 0, 0};

    /// 选参考星
    [[nodiscard]] std::optional<SatID> selectRtkRefSat(const std::vector<SatID> &candidates) const;

    Eigen::Vector3d mRoverApprox{0, 0, 0}; // 流动站概略坐标
    int mGfCommonMode = 2;                 // GF 共模抑制阈值
    std::size_t mBaseEphSize = 0;
    std::size_t mRoverEphSize = 0;
    Eigen::Vector3d mBaselineSigmaFixed = Eigen::Vector3d::Zero(); // 固定基线标准差

    std::map<SatID, double> satRoverElevData; // 流动站仰角：参考星按流动站高度角最大选取
    std::map<SatID, PVT> satPVTRecTime;       // 按基准站旋转到接收时刻 ECEF，供基准站端几何与仰角方位
    std::map<SatID, PVT> satPVTRoverRecTime;  // 按流动站旋转，供流动站端几何、仰角与多普勒

    // 站间单差方差
    struct SdVar {
        double phase = 0.0, code = 0.0;
    };

    std::map<SatID, SdVar> mSdVar;
    Eigen::MatrixXd mWeightMatrix;
    bool mWeightReady = false;

    std::map<char, SatID> mRtkRefSat; // 各系统参考星

    struct SysCombo {
        FreqCombo base, rover;
        bool ok = false;
    };

    /// 本历元浮点解的统一视图：LSQ 档来自 SolverLSQ、滤波档来自 ekf，回填与 AR 只认这一份。
    /// index 是变量在 x/P 里的下标（LSQ 按 varSet 顺序、滤波按 ekf 的索引），AR 取子块靠它。
    struct FloatState {
        EquSys eq{};
        Eigen::VectorXd x{};
        Eigen::MatrixXd P{};
        Eigen::VectorXd v{};
        std::map<Variable, int> index{};
        double sigma0 = 1.0;
    };

    // 双差方程缓存
    struct DDCache {
        SatID sat, ref;
        char sys = 0;
        PVT pvtBase, pvtRover;           // 本星按两站分别旋转到接收时刻
        double obsBase[4] = {0, 0, 0, 0};   // [L1,L2,C1,C2] 基准站观测值
        double obsRover[4] = {0, 0, 0, 0};  // 流动站观测值
        double refObsBase[4] = {0, 0, 0, 0};
        double refObsRover[4] = {0, 0, 0, 0};
        double phaseVar = 0, codeVar = 0;
        double refPhaseVar = 0, refCodeVar = 0;
        Variable amb[2];                 // 本星相对参考星的双差模糊度（周，参数为 −N）
        double ambInit[2] = {0, 0};      // 相位减伪距反算的双差模糊度初值（周）
        double lambda[2] = {0, 0};       // freq1/freq2 波长
        PVT refPvtBase, refPvtRover;     // 参考星 PVT，两站各一份
    };

    const ObsData *mRtkBase = nullptr;
    const ObsData *mRtkRover = nullptr;
    std::map<char, SysCombo> mRtkCombos;
    std::map<char, std::vector<SatID> > mRtkPartSats;
    std::map<char, std::map<std::string, std::map<SatID, double> > > mRtkFixedMap;
    EphemerisTable *mEph = nullptr;
    std::set<SatID> mRtkSlipExclude;

    std::vector<DDCache> mDDCache;

    /// 本历元浮点解：LSQ 档与滤波档都写这一份，回填与 AR 只读它。
    FloatState mFloat;

    bool solveFloat(EquSys &eq, SolverLSQ &solver, Eigen::Vector3d &baseline);

    /// 首历元（LSQ 档为每历元）引导：LSQ 求出浮点解写进 mFloat；滤波档再用它灌初值做一次量测更新。
    void initFilter();

    /// 滤波档后续历元：换基准 → 时间更新 → 按先验重建几何 → 抗差 → 量测更新。
    void updateFilter();

    /// 参考星切换：双差模糊度状态与协方差从旧基准搬到新基准，N'_{i,b} = N_{i,a} − N_{b,a}。
    void rebaseAmbiguities();

    /// LSQ 档每历元清掉跨历元状态；滤波档只在换档或断档时由 applyOptions 触发。
    void resetFilterState();

    /// 位置三态 + 双差模糊度变量（固定解只有位置），buildEquSys 与滤波档共用同一份。
    void collectVars(EquSys &eq, bool fixed) const;

    /// 固定解回填（浮点链路两条分支共用）。
    void applyFixedSolution(double s0);

    /// 从状态向量取本历元绝对位置。
    [[nodiscard]] Eigen::Vector3d filteredPosition() const;

    /// 本历元先验位置 x⁻（时间更新后、量测更新前），线性化点必须用它。
    [[nodiscard]] Eigen::Vector3d predictedPosition() const;

    /// 从浮点解的协方差阵按位置三态索引抠 3×3 块；索引缺失时给 1e12 单位阵。
    [[nodiscard]] Eigen::Matrix3d positionCov(const FloatState &fs) const;

    /// 量测更新后把 ekf 的状态搬进 mFloat，供回填与 AR 使用。
    void captureFilteredState();

    /// 取本历元的双差模糊度及其协方差：状态本身就是双差，按变量下标直接抠子块。
    void extractAmbiguities(const FloatState &fs, Eigen::VectorXd &b, Eigen::MatrixXd &Qb,
                            std::vector<Variable> &aVars) const;

    /// 站间单差方差
    void computeSdVariance();

    /// 构造双差随机模型
    void buildStochasticModel(const EquSys &eq);

    /// 构造本历元满阵权：仅完整协方差模式需要，对角模式直接用方程标量权。
    void buildWeightMatrix(const EquSys &eq);

    void solveIteration(SolverLSQ &solver, EquSys &eq) const;

    bool tryFixAmbiguities(const std::vector<Variable> &aVars, const Eigen::VectorXd &b,
                           const Eigen::MatrixXd &Qb);


    /// 新弧段/周跳的模糊度初值：buildEquSys 顺手反算，谁要灌滤波谁传 seed。
    struct AmbSeed {
        std::map<Variable, double> init; // 相位减伪距反算的双差初值（周，口径 −N）
        VariableDataMap novar;           // 2 = 用初值、方差放回 ambInitSigma
    };

    /// 双差方程；seed 非空时把「状态里还没有的模糊度」初值一并反算出来。
    void buildEquSys(EquSys &eq, const Eigen::Vector3d &roverPos, bool fixed, AmbSeed *seed = nullptr);

    void detectFreqCombinations();

    bool buildGeometry();

    void buildDDCache();

    /// 位置链路：双差浮点解 + 模糊度固定，回填 result 的位置/DOP/残差。
    bool estimatePosition();

    void readback(const EquSys &eqFloat, const Eigen::VectorXd &residual, const Eigen::Vector3d &posEcef);

    /// 速度链路：多普勒 4 未知数，与位置链路各写各的 result 字段；与 SPP::estimateVelocity 同一份实现。
    void estimateVelocity();

    void detectSlips();

    void addDoubleDiffEquation(EquSys &eq, const DDCache &c,
                               const Eigen::Vector3d &ddLos, double ddGeom,
                               int freq, bool isPhase, bool fixed, AmbSeed *seed);


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

    /// 上历元各系统的参考星：换基准时靠它构造 D 变换
    std::map<char, SatID> mPrevRefSat;
};
