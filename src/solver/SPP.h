#pragma once

#include "core/equ/EquStruct.h"
#include "core/obs/ObsStruct.h"
#include "core/equ/Result.h"
#include "core/SatID.h"
#include "SolverLSQ.h"
#include "core/geom/Ephemeris.h"
#include "core/geom/EphemerisTable.h"
#include "rinex/RINEXNavStore.h"
#include "core/obs/FreqCombo.h"
#include "core/obs/Weight.h"
#include "SPPConfig.h"
#include "SolverOptions.h"
#include "TropoGrid.h"
#include "TropoSite.h"
#include "GnssEKF.h"
#include <Eigen/Eigen>
#include <set>
#include <vector>


using IFCodeTypes = std::map<char, FreqCombo>;

class SPP {
protected:
    SPPConfig mConfig;
    Weight mWeight;
    bool mWeightInit = false;

public:
    bool loadBRDC(const std::string &path);

    /// 接管另一份同配置解算器已读好的产品：前向+后向档的倒序趟用它免二次读盘。
    /// 搬完后 src 的星历与格网已空，只能当废弃对象。
    virtual void adoptProducts(SPP &src);

    /// 外部对流层格网
    bool loadVMF3(const std::vector<std::string> &paths) { return mTropoGrid.loadVMF3(paths); }
    bool loadGPT3(const std::string &path) { return mTropoGrid.loadGPT3(path); }

    /// 所选对流层模型的格网是否已就位
    [[nodiscard]] bool tropoGridReady() const { return mTropoGrid.readyFor(mConfig.tropoMap); }

    void detectIFCombinations(const ObsData &obsData, const IFCodeTypes &defaultTypes = {});

    void setCutoffElevDeg(const double deg) { mConfig.cutoffElevRad = deg * DEG_TO_RAD; }

    /// ISB 估计模式
    void setISBProcSigma(const double proc, const double init) {
        mConfig.isbProcSigma = proc;
        mConfig.isbInitSigma = init;
    }

    void setEnabledSystems(const std::set<char> &s) { mConfig.enabledSystems = s; }

    /// 统一参数注入入口
    virtual void applyOptions(const SolverOptions &o);

    virtual bool processEpoch(ObsData &obsData);

    virtual void preprocess(ObsData &obsData);

protected:
    virtual void solve(ObsData &obsData);

    /// 位置链路统一骨架：只按引导标志分派引导 / 更新；LSQ 档靠逐历元重置落到引导分支。
    virtual void estimatePosition(ObsData &obsData);

    /// LSQ 档每历元清掉跨历元滤波状态；派生类在这里补自己的弧段与钟差先验。
    virtual void resetFilterState();

    /// 首历元（LSQ 档为每历元）引导：SPP 装 EKF 引导 + 量测更新，PPP 装精密产品与相位方程。
    virtual void initFilter(ObsData &obsData);

    /// 滤波档后续历元的量测更新。
    virtual void updateFilter(ObsData &obsData);

    /// 引导用的迭代伪距解，只给位置与钟差线性化点，不回写结果。
    void bootstrapSolution(ObsData &obsData);

    /// 滤波量测更新：首历元由 LSQ 引导，之后每历元用上历元后验线性化。
    void runFilter(ObsData &obsData);

    /// 滤波档的结果回写：协方差、DOP、σ、后验残差取自 ekf。
    void readbackFiltered(const ObsData &obsData);

    /// 从协方差阵按三个位置参数索引抠 3×3 块；索引不全或越界时给 1e12 单位阵。
    [[nodiscard]] static Eigen::Matrix3d positionCovBlock(const Eigen::MatrixXd &P, const int id[3]);

public:
    virtual void computeSatPos(ObsData &obsData);

    void computeElevAzim();

    void earthRotation();

    /// IF-code 解算的结果位置和钟差，供 UC 模式热启动
    [[nodiscard]] double getClockBias(const char sys) const {
        const auto it = rClockBias.find(sys);
        return it != rClockBias.end() ? it->second : 0.0;
    }

    virtual ~SPP() = default;

    Eigen::Vector3d xyz{0, 0, 0};
    GeodeticFrame frame = Frame::WGS84;
    Result result{};

    /// 滤波状态：LSQ 档不用，滤波档由 estimatePosition 驱动；PPP/LEO 继承后共用同一份。
    GnssEKF ekf;
    int mDirection = 1;
    bool mUseFilter = false;
    bool mFilterInit = false;                             // 滤波已由首历元 LSQ 引导
    double sigma0 = 1.0;                                  // 本历元验后方差因子，前向+后向融合读它

    /// 位置 3×3 后验协方差（物理量），前向+后向融合按它加权，也是结果回写的口径来源。
    [[nodiscard]] virtual Eigen::Matrix3d posCovariance() const;
    SatValueMap satElevData{};
    SatValueMap satAzimData{};
    std::set<SatID> satRejected{};
    std::map<SatID, PVT> satPVTTransTime{};
    std::map<SatID, PVT> satPVTRecTime{};


    EphemerisTable ephTable;
    /// 广播电文健康标志
    bool ephUnhealthy(const SatID &sat, const Ephemeris *eph);

protected:
    std::set<SatID> mHealthWarned; // 健康剔除日志去重
    bool isRover = true;
    bool mRequirePhaseForIF = false;

    std::map<char, double> rClockBias{};

    /// 上一历元各系统终值钟差 [m]
    std::map<char, double> mPrevClockBias{};

    /// 上一成功历元的时刻：常速度预测取它与当前历元的间隔作积分步长
    CommonTime mLastEpoch{};
    bool mHasLastEpoch = false;

    /// 系统→钟差参数映射；多系统各自独立钟差=吸收 ISB。
    static inline std::map<char, Parameter> sysCdtParam = {
        {'G', Parameter::cdt},
        {'C', Parameter::cdt2},
        {'R', Parameter::cdt3},
        {'E', Parameter::cdt4},
    };

    /// 公共钟/ISB 的参考系统：优先 GPS，无 GPS 时取参与星座里优先级最高的。
    [[nodiscard]] static char clockRefSys(const std::set<char> &active);

    /// 公共钟+ISB 状态模型（与 PPP 同口径）：参考系统占 cdt，其余系统占原槽位作 ISB。
    bool mCommonClockActive = false;
    /// 公共钟的参考系统。
    char mClockRefSys = 'G';
    /// 生效中的系统→钟参数表：公共钟下参考系统被挪进 cdt 槽（remapClockParams）。
    std::map<char, Parameter> mCdtParam = sysCdtParam;

    /// 公共钟+ISB 是否生效：开了 ISB 估计、参与系统全在 GREC、且 ≥2 个。
    [[nodiscard]] virtual bool wantCommonClock() const;

    /// 参考系统挪槽：ref 占 cdt，其余占原槽位作 ISB；无 GPS 参考时把 ref 挪进 cdt 并删 G。
    void remapClockParams(char ref);

    /// 把线性化出的独立钟方程改写成公共钟参数化，并删掉 ISB 伪观测（连续性交给状态约束）。
    void toCommonClockEquations(const ObsData &obsData);

    /// 从 EKF 后验状态按变量名读位置（不依赖状态序）；缺位置态时抛 InvalidRequest。
    [[nodiscard]] Eigen::Vector3d currentEstimatedPosition() const;

    IFCodeTypes ifCodeTypes{};

    std::map<SatID, FreqCombo> mGloCombo;

    /// 取卫星的 IF 频率组合
    [[nodiscard]] const FreqCombo *freqComboFor(const SatID &sat) const {
        if (sat.system == 'R') {
            const auto it = mGloCombo.find(sat);
            return it == mGloCombo.end() ? nullptr : &it->second;
        }
        const auto it = ifCodeTypes.find(sat.system);
        return it == ifCodeTypes.end() ? nullptr : &it->second;
    }

    /// 上一次 IF 码类型检测的"各系统可用码类型"签名；不变则跳过重扫
    std::map<char, std::string> ifCodeSig;

    /// 当前历元有卫星的系统集合
    std::set<char> activeSystems;

    /// 上一轮解算的标准化残差和原始残差，用于粗差探测
    std::map<SatID, double> lastWStats;
    std::map<SatID, double> lastResidMag;

    /// 解算完成后计算每颗卫星的 Baarda w 统计量
    virtual void buildResidualMap();

    /// 速度链路：位置已定后另解 4 个未知数（vx/vy/vz + 钟漂）填 `result.vel/sigmaVel`。
    /// 与 estimatePosition 并列，各写各的 result 字段；PPP/LEO 继承后直接用这一份。
    void estimateVelocity(const ObsData &obsData);

    TropoGrid mTropoGrid;

private:
    TropoSite mTropoSite;

    /// 上一次成功解算的位置 [m]
    Eigen::Vector3d mLastXyz{0, 0, 0};


    [[nodiscard]] Eigen::Vector3d roughInitialXyz(const ObsData &obsData) const;

    void updateTropoSite(const Eigen::Vector3d &xyzEcef, const CommonTime &epoch);

    [[nodiscard]] double tropoDelay(double elevRad) const;

    EquSys posEquations{};
    SolverLSQ posSolver{};

    void linearize(ObsData &obsData, int iter);

    /// 构建单颗卫星的位置观测方程（IF 组合）
    void buildPosEquation(const SatID &sat, const PVT &pvt, const Eigen::Vector3d &los,
                          double rho, double trop,
                          const std::string &obsType, double obsVal, double weight,
                          const Variable &dx, const Variable &dy,
                          const Variable &dz, const Variable &cdtVar);
};
