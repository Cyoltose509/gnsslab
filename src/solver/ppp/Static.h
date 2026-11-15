#pragma once

#include "SPP.h"
#include "GnssEKF.h"
#include "core/obs/CycleSlip.h"
#include <Eigen/Eigen>
#include <map>
#include <set>

class PPPStatic : public SPP {
public:
    PPPStatic() {
        mRequirePhaseForIF = true;
    }

    ~PPPStatic() override;

    /// 精密产品与辅助数据：SP3 / CLK / ATX / OSB / ERP / TRO / ATT / IONEX
    bool processEpoch(ObsData &obs) override;

    /// 位置 3x3 后验协方差，按 PPP 状态集索引取，前向+后向融合按它加权。
    [[nodiscard]] Eigen::Matrix3d posCovariance() const override;

    /// 对流层 ZTD 估计开关（σ 口径：initSigma/ztdProcSigma 均为 σ）
    void setTropo(const bool tropo, const double initVal = 0.0, const double initSigma = 0.2, const double ztdProcSigma = 1.0E-3) {
        mHasTropo = tropo;
        mConfig.ztdInitValue = initVal;
        mConfig.ztdInitSigma = initSigma;
        mConfig.ztdProcSigma = ztdProcSigma;
    }

    /// 观测 σ 配置
    void setMeasSigma(const double codeSigma, const double phaseSigma) {
        mConfig.codeSigma = codeSigma;
        mConfig.phaseSigma = phaseSigma;
    }

    void setFirstEpochPosSigma(const double v) { mConfig.firstEpochPosSigma = v; }

    /// 运动学 σ / 先验旋钮（σ 口径）
    void setKinematicSigma(const double clockInitSigma = 1.0E4,
                           const double firstEpochPosSigma = 1.0,
                           const double ambProcSigma = 3.16227766E-4,
                           const double phaseSigma = 0.003,
                           const int minSatsForUpdate = 4) {
        mConfig.firstEpochPosSigma = firstEpochPosSigma;
        mConfig.ambProcSigma = ambProcSigma;
        mConfig.phaseSigma = phaseSigma;
        mConfig.minSatsForUpdate = minSatsForUpdate;
        ekf.clockInitSigma = clockInitSigma;
    }

    void setKeepAmbMaxGap(const double sec) { mConfig.keepAmbMaxGap = sec; }

    void setPosFloorSigma(const double v) { mConfig.posFloorSigma = v; }

    void applyOptions(const SolverOptions &o) override;

    void setZTDProcSigma(const double q) { mConfig.ztdProcSigma = q; }

    /// ZTD 先验 σ 旋钮
    void setZTDPriorSigma(const double s) { mConfig.ztdPriorSigma = s; }

    /// TRO 产品 ZTD 约束 σ (m)
    void setTROZTDSigma(const double s) { mConfig.troZtdSigma = s; }

    void setAmbInitSigma(const double v) { mConfig.ambInitSigma = v; }

    void setAmbProcSigma(const double q) { mConfig.ambProcSigma = q; }

    void setCodeBiasProcSigma(const double q, const double initSigma) {
        mConfig.codeBiasProcSigma = q;
        mConfig.codeBiasInitSigma = initSigma;
    }

    void setGloIfcbProcSigma(const double q, const double initSigma) {
        mConfig.gloIfcbProcSigma = q;
        mConfig.gloIfcbInitSigma = initSigma;
    }

    /// 码偏差是否覆盖了这颗星/这个系统：GLONASS 的标定是逐测站的，站名对不上就交给 IFCB 状态。
    [[nodiscard]] bool osbCovers(const SatID &sat) const { return prod->osb.covers(sat, osbStation()); }
    [[nodiscard]] bool osbCoversSystem(const char sys) const { return prod->osb.coversSystem(sys, osbStation()); }
    [[nodiscard]] std::string osbStation() const { return mCurObs ? mCurObs->station : std::string(); }

    bool mKinematic = false;
    bool mReanchorEveryEpoch = false; // 每历元重锚定
    double mMotionThresh = 0.0;
    double mSppVsPredThresh = 300.0;
    bool mApplyWindup = true; // 相位缠绕总开关

    bool mApplySolidTide = true; // 固体潮

    bool mUseCycleSlip = true;
    double mGfSlipMult = 3.0; // GF 周跳阈值倍数（×单周跳阶跃）

    bool mHasTropo = true; // 估计 ZTD
    bool mUseTroGrad = true; // TRO 水平梯度固定改正开关（默认开；关=退化为仅 ZTD，便于 A/B）
    double mZtdEstimate = 0.0; // 当前 ZTD 状态估计(m)

    bool mPinInitHeight = true;

    bool mUseAR = true; // AR 开关
    double mArRatio = 0.0; // 最近一次 LAMBDA 成功率
    int mArNumFixed = 0; // 最近一次固定卫星数
    long mArAttempts = 0, mArSuccess = 0; // AR 累计计数
    long mDivergeRecovers = 0; // 运动学发散兜底触发次数（诊断）
protected:
    Eigen::Vector3d mRecvVel = Eigen::Vector3d::Zero();
    [[nodiscard]] virtual Eigen::Vector3d recvAntennaOffsetECEF(const Eigen::Vector3d &xyzEst,
                                                                const CommonTime &epoch,
                                                                const SatID &sat) const;
    [[nodiscard]] Eigen::Vector3d recvEffFor(const SatID &sat) const;

    // 相位缠绕基准
    void recvWindupBasis(const Eigen::Vector3d &recv, Eigen::Vector3d &exr, Eigen::Vector3d &eyr) const;

    [[nodiscard]] virtual Eigen::Vector3d predictPosition(double dt, const CommonTime &epoch) const;

    bool tryFixAmbiguities(ObsData &obsData);

    void initFilter(ObsData &obsData) override;
    void updateFilter(ObsData &obsData) override;
    void computeSatPos(ObsData &obsData) override;
    void solve(ObsData &obsData) override;
    void estimatePosition(ObsData &obsData) override;
    void resetFilterState() override;

    [[nodiscard]] std::pair<Eigen::Vector3d, bool> solveInitialPosition(ObsData &obsData);

    virtual void buildEquSys(ObsData &obsData, const Eigen::Vector3d &xyzEst, VariableDataMap &csData, bool firstIter);

    virtual void bootstrapFirstEpoch(ObsData &obsData);

    void applyPriorClocks();

    /// 天线相位中心变化改正(m)
    [[nodiscard]] double pcvCorrection(const SatID &sat) const;

    void addSatelliteEquations(const SatID &sat, const TypeValueMap &tv,
                               double mapW, double tropoFixed, bool probeOnly);

    /// 用本历元码/相位预fit 判粗差，先扣同系统中位数。
    void decidePrefitGate();

    /// 预fit 试算结果，供 decidePrefitGate() 决定哪几颗星要剔。
    struct PrefitRec {
        double code = 0.0;
        double phase = 0.0;
        bool freshAmb = false; // 本历元新弧段：相位预fit 已把钟差反算进模糊度
    };

    void resetSlipDetectors(const ObsData &obsData);

    /// 对流层 ZTD 状态 + 先验/外部产品约束方程
    void addZTDConstraint();

    [[nodiscard]] Eigen::Vector3d computeTideDisplacement(const Eigen::Vector3d &xyzEst, const CommonTime &epoch) const; // 固体潮 + 极潮

    [[nodiscard]] double computeIfAmbiguity(const SatID &sat, const TypeValueMap &tv,
                                            double map, double tropoFixed) const;

    void resetAmbiguity(const SatID &sat, const TypeValueMap &tv,
                        double map, double tropoFixed, bool isSlipReset);

    void processSatellite(const SatID &sat, const TypeValueMap &tv);

    void detectCycleSlip(const SatID &sat, const TypeValueMap &tv,
                         bool &slipReset, bool &resetAmb);

    void addObservationEquation(const EquID &eid, double prefit, const Eigen::Vector3d &los,
                                const Weight &w, bool hasAmb, double elev);

    // ---- 产品与改正量 ----
    std::string mRcvAntenna;
    std::map<SatID, double> mWindupPrev;
    std::map<SatID, double> mWindupAcc;
    std::set<SatID> mFallbackWarned; // 回退广播星历的日志去重

    std::map<SatID, double> mNwSmooth; // MW 宽巷平滑值(周)
    std::map<SatID, double> mNwVar; // MW 宽巷平滑方差(周²)

    // ---- 滤波器与权重 ----
    EquSys mEq;
    VariableSet mVarSet; // EKF 状态参数集
    VariableDataMap mEpochStates; // 本历元新增状态量，随 initFilter/updateFilter 走

    Weight mWeightCode;
    Weight mWeightPhase;
    bool mWeightInit = false;
    bool mFirstEpoch = true;
    Eigen::Vector3d mRefPos = Eigen::Vector3d::Zero();
    int mProcEpoch = 0; // 已处理历元计数
    bool mFilterFrozen = false;
    // 锚点不可信时（按截止角筛剩下的星不足 4 颗）本历元不筛星。
    bool mSkipElevCutoff = false;

    // ---- 状态回读 ----
    std::map<char, double> mClkBase;
    std::map<char, double> mClkEstimate;
    double mZtdBase = 0.0;
    bool mZtdBaseValid = false;
    std::map<char, double> mCbEstimate; // 每系统 IF 码偏状态估计(m)
    std::map<SatID, double> mIfcbEstimate; // 逐 GLONASS/BDS 星 IFCB 状态估计(m)
    std::map<SatID, double> mAmbEstimate;

    // ---- 本历元派生量 ----
    const ObsData *mCurObs = nullptr; // 当前历元观测
    Eigen::Vector3d mEpochXyz = Eigen::Vector3d::Zero(); // 本历元线性化点
    Eigen::Vector3d mEpochTide = Eigen::Vector3d::Zero(); // 固体潮+极潮位移(ECEF)
    TropoSite mEpochTropo; // 测站对流层快照（每历元一次，所有星共享）
    bool mEpochFirstIter = true; // 是否首次迭代

    std::map<SatID, double> mSatMap; // 每星湿映射函数
    std::map<SatID, double> mSatTropoFixed; // 每星对流层固定项(m)
    std::map<SatID, PrefitRec> mPrefitProbe; // 本历元预fit 试算结果
    std::set<SatID> mPrefitBad; // 本历元判为粗差、要剔除的星
    std::set<SatID> mPrefitPhaseBad; // 只剔除相位、码仍可用的星（整周跳变）
    std::set<char> mSysSeen; // 本历元出现过的系统
    std::set<SatID> mCurrentSatSet; // 本历元参与解算的星

    /// 本历元真正有 IF 组合的系统：systems 只在组合探测层过滤，观测全集不能用。
    [[nodiscard]] std::set<char> activeIfSystems() const {
        std::set<char> s;
        for (const auto &[sys, def]: ifCodeTypes) s.insert(sys);
        for (const auto &[sat, def]: mGloCombo) s.insert(sat.system);
        return s;
    }

    void selectClockReference();

    [[nodiscard]] bool wantCommonClock() const override;

    VariableDataMap *mCsData = nullptr; // 本历元待写入的状态量（由 solve 传入）
    CycleSlipEpoch mGfEpoch; // GF 周跳共模裁决簿记

    std::map<SatID, CycleSlip> mSlipDetectors;
    std::set<SatID> mPrevSatSet;
    int mGfCommonMode = 2; // GF 共模抑制阈值
    bool mForceAmbReset = false;
    CommonTime mLastEpochTime; // 上一成功解算历元时刻
    CommonTime mLastObsTime; // 上一观测历元时刻（解算失败也推进）
    double mNominalInterval = -1.0; // 标称历元间隔(自适应学习，秒)
    bool mHasLastEpoch = false;
    bool mHasLastObs = false;
    bool mGapThisEpoch = false; // 解算中断恢复首历元标志（只影响位置外推，不重置模糊度）
    long slipVerdicts = 0; // 本运行累计 Slip 判定数
    long ambResets = 0; // 本运行累计模糊度重置(newArc/Slip/Gap)数

private:
    /// 位置链路尾段：把 EKF 状态（位置、钟、码偏差、模糊度、残差、DOP）回读到 result。
    void readback(const ObsData &obsData);

    /// 钟差先验均值：EKF 已初始化取估计值，否则取 SPP 引导基值。
    [[nodiscard]] double clockPrior(const char sys) const {
        return mFilterInit
                   ? (mClkEstimate.count(sys) ? mClkEstimate.at(sys) : 0.0)
                   : mClkBase.count(sys)
                         ? mClkBase.at(sys)
                         : 0.0;
    }

    /// ZTD 先验均值：EKF 已初始化取估计值，否则取 SPP 引导基值/配置初值。
    [[nodiscard]] double ztdPrior() const {
        return mFilterInit ? mZtdEstimate : mZtdBaseValid ? mZtdBase : mConfig.ztdInitValue;
    }
};
